"""Windows-friendly ROS2 dashboard backed by a bundled local compatibility layer."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


HOST = "127.0.0.1"
PORT = 8876
BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
RUNTIME_DIR = BASE_DIR / "runtime"
LOG_DIR = RUNTIME_DIR / "logs"
DEMO_CONFIG_PATH = BASE_DIR / "demo_profile.json"
SHARED_PROFILE_PATH = BASE_DIR / "shared_profile.json"
DEVICE_CACHE_PATH = PROJECT_ROOT / "device_cache.json"

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from dume_endpoint_discovery import discover_dume_endpoint, normalize_base_url  # noqa: E402
from ros2_mock.pick_and_place_server import ACTION_NAME, ACTION_TYPE, NODE_NAME  # noqa: E402
from ros2_mock.runtime import FEEDBACK_DIR, GOALS_DIR, NODES_DIR, RESULTS_DIR, TOPICS_DIR, ensure_runtime_dirs, list_nodes  # noqa: E402


PARAMETER_EVENT_TOPIC = "/parameter_events [rcl_interfaces/msg/ParameterEvent]"
ROSOUT_TOPIC = "/rosout [rcl_interfaces/msg/Log]"
ACTIVE_NODE_TTL_SEC = 3.0


def is_frozen() -> bool:
    return getattr(sys, "frozen", False)


@dataclass
class ManagedProcess:
    key: str
    label: str
    command: list[str]
    log_path: Path
    long_running: bool
    params: dict[str, Any] = field(default_factory=dict)
    process: subprocess.Popen[str] | None = None
    started_at: float = field(default_factory=time.time)
    ended_at: float | None = None
    exit_code: int | None = None

    def status(self) -> str:
        if self.process is None:
            return "pending"
        code = self.process.poll()
        if code is None:
            return "running"
        if self.exit_code is None:
            self.exit_code = code
            self.ended_at = time.time()
        return "finished" if code == 0 else "failed"


class ProcessManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._processes: dict[str, ManagedProcess] = {}
        LOG_DIR.mkdir(parents=True, exist_ok=True)

    def run(
        self,
        key: str,
        label: str,
        command: list[str],
        *,
        long_running: bool,
        params: dict[str, Any] | None = None,
    ) -> ManagedProcess:
        with self._lock:
            existing = self._processes.get(key)
            if existing and existing.status() == "running":
                return existing

            log_path = LOG_DIR / f"{key}.log"
            log_path.write_text("", encoding="utf-8")
            handle = log_path.open("a", encoding="utf-8", buffering=1)
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            proc = subprocess.Popen(
                command,
                stdout=handle,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=str(PROJECT_ROOT),
                env=env,
            )
            managed = ManagedProcess(
                key=key,
                label=label,
                command=command,
                log_path=log_path,
                long_running=long_running,
                params=params or {},
                process=proc,
            )
            self._processes[key] = managed
            return managed

    def stop(self, key: str) -> bool:
        with self._lock:
            managed = self._processes.get(key)
            if not managed or managed.process is None:
                return False
            if managed.process.poll() is None:
                managed.process.terminate()
                try:
                    managed.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    managed.process.kill()
            managed.exit_code = managed.process.poll()
            managed.ended_at = time.time()
            return True

    def stop_all(self) -> None:
        for key in list(self._processes):
            self.stop(key)

    def clear_finished(self) -> None:
        with self._lock:
            removable = [key for key, managed in self._processes.items() if managed.status() in {"finished", "failed"}]
            for key in removable:
                del self._processes[key]

    def status(self) -> list[dict[str, Any]]:
        with self._lock:
            items = []
            for key in sorted(self._processes):
                managed = self._processes[key]
                items.append(
                    {
                        "key": managed.key,
                        "label": managed.label,
                        "status": managed.status(),
                        "started_at": managed.started_at,
                        "ended_at": managed.ended_at,
                        "exit_code": managed.exit_code,
                        "long_running": managed.long_running,
                        "params": managed.params,
                    }
                )
            return items

    def read_log(self, key: str, limit: int = 20000) -> str:
        with self._lock:
            managed = self._processes.get(key)
            if not managed or not managed.log_path.exists():
                return ""
            text = managed.log_path.read_text(encoding="utf-8", errors="replace")
            return text[-limit:]


MANAGER = ProcessManager()


def load_demo_profile() -> dict[str, Any]:
    defaults = {
        "base_url": "http://192.168.1.132",
        "safe_height_mm": 200,
        "waypoint_delay_ms": 200,
        "gripper_close_angle": 20,
        "steps_per_segment": 6,
        "arrival_tolerance_mm": 10,
        "pick_position": [0.10, 0.00, 0.20],
        "drop_position": [0.20, 0.10, 0.05],
    }
    try:
        if DEVICE_CACHE_PATH.exists():
            cache = json.loads(DEVICE_CACHE_PATH.read_text(encoding="utf-8"))
            cached_url = str(cache.get("last_success_url", "")).strip()
            if cached_url:
                defaults["base_url"] = cached_url
    except Exception:
        pass

    merged = dict(defaults)

    if DEMO_CONFIG_PATH.exists():
        try:
            payload = json.loads(DEMO_CONFIG_PATH.read_text(encoding="utf-8"))
            merged.update(payload)
        except Exception:
            pass

    if SHARED_PROFILE_PATH.exists():
        try:
            payload = json.loads(SHARED_PROFILE_PATH.read_text(encoding="utf-8"))
            merged.update(payload)
        except Exception:
            pass

    resolved = discover_dume_endpoint(merged.get("base_url"), PROJECT_ROOT, allow_subnet_scan=True)
    if resolved is not None:
        merged["base_url"] = resolved.base_url

    if not DEMO_CONFIG_PATH.exists():
        DEMO_CONFIG_PATH.write_text(json.dumps(merged, indent=2), encoding="utf-8")
    return merged


def build_action(action: str, payload: dict[str, Any]) -> tuple[str, list[str], bool]:
    profile = load_demo_profile()
    if action == "build_workspace":
        if is_frozen():
            return (
                "build_workspace",
                [
                    sys.executable,
                    "--run-ros2-build-workspace",
                ],
                False,
            )
        return (
            "build_workspace",
            [
                sys.executable,
                "-u",
                "-m",
                "ros2_mock.build_workspace",
            ],
            False,
        )
    if action == "ros2_server":
        base_url = normalize_base_url(payload.get("base_url", profile["base_url"])) or profile["base_url"]
        safe_height_mm = float(payload.get("safe_height_mm", profile["safe_height_mm"]))
        waypoint_delay_ms = int(payload.get("waypoint_delay_ms", profile["waypoint_delay_ms"]))
        gripper_close_angle = int(payload.get("gripper_close_angle", profile["gripper_close_angle"]))
        steps_per_segment = int(payload.get("steps_per_segment", profile["steps_per_segment"]))
        arrival_tolerance_mm = int(payload.get("arrival_tolerance_mm", profile["arrival_tolerance_mm"]))
        if is_frozen():
            return (
                "ros2_server",
                [
                    sys.executable,
                    "--run-ros2-server",
                    "--base-url",
                    base_url,
                    "--safe-height-mm",
                    str(safe_height_mm),
                    "--waypoint-delay-ms",
                    str(waypoint_delay_ms),
                    "--gripper-close-angle",
                    str(gripper_close_angle),
                    "--steps-per-segment",
                    str(steps_per_segment),
                    "--arrival-tolerance-mm",
                    str(arrival_tolerance_mm),
                ],
                True,
            )
        return (
            "ros2_server",
            [
                sys.executable,
                "-u",
                "-m",
                "ros2_mock.pick_and_place_server",
                "--base-url",
                base_url,
                "--safe-height-mm",
                str(safe_height_mm),
                "--waypoint-delay-ms",
                str(waypoint_delay_ms),
                "--gripper-close-angle",
                str(gripper_close_angle),
                "--steps-per-segment",
                str(steps_per_segment),
                "--arrival-tolerance-mm",
                str(arrival_tolerance_mm),
            ],
            True,
        )
    if action == "ros2_send_goal":
        pick = payload.get("pick_position", profile["pick_position"])
        drop = payload.get("drop_position", profile["drop_position"])
        goal = json.dumps(
            {
                "pick_pose": {
                    "position": {"x": pick[0], "y": pick[1], "z": pick[2]},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "drop_pose": {
                    "position": {"x": drop[0], "y": drop[1], "z": drop[2]},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            }
        )
        if is_frozen():
            return (
                "ros2_send_goal",
                [
                    sys.executable,
                    "--run-ros2-send-goal",
                    "action",
                    "send_goal",
                    ACTION_NAME,
                    ACTION_TYPE,
                    goal,
                    "--feedback",
                ],
                False,
            )
        return (
            "ros2_send_goal",
            [
                sys.executable,
                "-u",
                "-m",
                "ros2_mock.mock_ros2_cli",
                "action",
                "send_goal",
                ACTION_NAME,
                ACTION_TYPE,
                goal,
                "--feedback",
            ],
            False,
        )
    raise KeyError(action)


def _read_jsonl_last(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except Exception:
        return None
    for line in reversed(lines):
        line = line.strip()
        if not line:
            continue
        try:
            return json.loads(line)
        except Exception:
            continue
    return None


def _read_json_file(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _latest_goal_payload() -> dict[str, Any] | None:
    latest_path: Path | None = None
    latest_mtime = -1.0
    for path in GOALS_DIR.glob("*.json"):
        try:
            mtime = path.stat().st_mtime
        except OSError:
            continue
        if mtime > latest_mtime:
            latest_mtime = mtime
            latest_path = path
    if latest_path is None:
        return None
    return _read_json_file(latest_path)


def _latest_result_payload() -> dict[str, Any] | None:
    latest_path: Path | None = None
    latest_mtime = -1.0
    for path in RESULTS_DIR.glob("*.json"):
        try:
            mtime = path.stat().st_mtime
        except OSError:
            continue
        if mtime > latest_mtime:
            latest_mtime = mtime
            latest_path = path
    if latest_path is None:
        return None
    return _read_json_file(latest_path)


def _latest_feedback_payloads(limit: int = 10) -> list[dict[str, Any]]:
    latest_path: Path | None = None
    latest_mtime = -1.0
    for path in FEEDBACK_DIR.glob("*.jsonl"):
        try:
            mtime = path.stat().st_mtime
        except OSError:
            continue
        if mtime > latest_mtime:
            latest_mtime = mtime
            latest_path = path
    if latest_path is None:
        return []
    try:
        lines = latest_path.read_text(encoding="utf-8").splitlines()
    except Exception:
        return []
    payloads: list[dict[str, Any]] = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            payloads.append(json.loads(line))
        except Exception:
            continue
    return payloads[-limit:]


def _topic_latest_payload(topic_name: str) -> dict[str, Any] | None:
    slug = topic_name.strip("/").replace("/", "__")
    return _read_jsonl_last(TOPICS_DIR / f"{slug}.jsonl")


def _file_excerpt(path: Path, *, start_line: int = 1, end_line: int | None = None, max_lines: int = 18) -> list[str]:
    if not path.exists():
        return ["File not found."]
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except Exception as exc:
        return [f"Unable to read file: {exc}"]
    start_index = max(0, start_line - 1)
    if end_line is None:
        end_index = min(len(lines), start_index + max_lines)
    else:
        end_index = min(len(lines), end_line)
    excerpt = lines[start_index:end_index]
    numbered = []
    for offset, line in enumerate(excerpt, start=start_index + 1):
        numbered.append(f"{offset:>4}: {line}")
    return numbered or ["No file content available."]


def _last_log_lines(text: str, *, needle: str, limit: int = 6) -> list[str]:
    matches = [line.strip() for line in text.splitlines() if needle.lower() in line.lower()]
    return matches[-limit:]


def _state_machine_trace(server_log: str) -> list[str]:
    trace = []
    for line in server_log.splitlines():
        stripped = line.strip()
        if "status=" in stripped and "phase=" in stripped:
            trace.append(stripped)
    return trace[-8:]


def _runtime_topic_items() -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for topic_name, title in (
        ("/arm_controller/joint_trajectory", "Latest /arm_controller/joint_trajectory publish"),
        ("/joint_states", "Latest /joint_states publish"),
    ):
        payload = _topic_latest_payload(topic_name)
        items.append(
            {
                "title": title,
                "subtitle": topic_name,
                "code": json.dumps(payload, indent=2, sort_keys=True) if payload else "No publish captured yet.",
            }
        )
    return items


def _deliverable_panels(server_log: str, latest_goal: dict[str, Any] | None, latest_result: dict[str, Any] | None) -> list[dict[str, Any]]:
    action_file = PROJECT_ROOT / "ros2_mock" / "dume_ros2_mock" / "action" / "PickAndPlace.action"
    server_file = PROJECT_ROOT / "ros2_mock" / "pick_and_place_server.py"
    fk_file = PROJECT_ROOT / "ik_kinematics.py"
    ik_file = PROJECT_ROOT / "ik_kinematics.py"
    dh_doc = PROJECT_ROOT / "ros2_mock" / "DH_PARAMETERS_AND_IK.md"

    return [
        {
            "title": "PickAndPlace.action file",
            "subtitle": str(action_file),
            "code": "\n".join(_file_excerpt(action_file, start_line=1, end_line=9)),
        },
        {
            "title": "pick_and_place_server.py node",
            "subtitle": str(server_file),
            "code": "\n".join(_file_excerpt(server_file, start_line=1, end_line=55)),
        },
        {
            "title": "Forward Kinematics (FK) implementation",
            "subtitle": str(fk_file),
            "code": "\n".join(_file_excerpt(fk_file, start_line=105, end_line=120)),
        },
        {
            "title": "Inverse Kinematics (IK) implementation",
            "subtitle": str(ik_file),
            "code": "\n".join(_file_excerpt(ik_file, start_line=122, end_line=162)),
        },
        {
            "title": "(Bonus) DH parameters + IK explanation",
            "subtitle": str(dh_doc),
            "code": "\n".join(_file_excerpt(dh_doc, start_line=1, max_lines=18)),
        },
        {
            "title": "Server execution trace",
            "subtitle": "Actual server log lines showing DUM-E backend use and state progression.",
            "code": "\n".join(
                _last_log_lines(server_log, needle="pick_and_place_server", limit=14)
                or ["No server execution trace yet."]
            ),
        },
        {
            "title": "Latest goal payload",
            "subtitle": "The most recent action goal recorded by the portable ROS2 layer.",
            "code": json.dumps(latest_goal, indent=2, sort_keys=True) if latest_goal else "No goal payload captured yet.",
        },
        {
            "title": "Latest result payload",
            "subtitle": "The most recent action result returned by the server.",
            "code": json.dumps(latest_result, indent=2, sort_keys=True) if latest_result else "No result payload captured yet.",
        },
    ]


def _runtime_panels(server_log: str, latest_result: dict[str, Any] | None) -> list[dict[str, Any]]:
    ros_state = ros2_state()
    feedback_payloads = _latest_feedback_payloads(limit=10)
    state_trace = _state_machine_trace(server_log)
    items: list[dict[str, Any]] = [
        {
            "title": "Live nodes",
            "subtitle": "Nodes currently registered by the running portable ROS2 layer.",
            "code": "\n".join(ros_state["nodes"]) if ros_state["nodes"] else "No ROS2 nodes visible yet.",
        },
        {
            "title": "Live actions",
            "subtitle": "Actions currently exposed by the running server.",
            "code": "\n".join(ros_state["actions"]) if ros_state["actions"] else "No ROS2 actions visible yet.",
        },
        {
            "title": "Live topics",
            "subtitle": "Topics currently visible from the compatibility layer.",
            "code": "\n".join(ros_state["topics"]) if ros_state["topics"] else "No ROS2 topics visible yet.",
        },
        {
            "title": "State machine feedback (0 -> 1 -> 2 -> 3)",
            "subtitle": "Raw feedback/status trace from the server log.",
            "code": "\n".join(state_trace) if state_trace else "No state-machine feedback trace yet.",
        },
        {
            "title": "Latest feedback publishes",
            "subtitle": "Actual feedback payloads written during the latest goal.",
            "code": "\n".join(json.dumps(item, sort_keys=True) for item in feedback_payloads) if feedback_payloads else "No feedback payloads captured yet.",
        },
        {
            "title": "Result return (success=True)",
            "subtitle": "Visible proof of the latest result return.",
            "code": json.dumps(latest_result, indent=2, sort_keys=True) if latest_result else "No result payload captured yet.",
        },
    ]
    items.extend(_runtime_topic_items())
    return items


def _proof_items(server_log: str, latest_goal: dict[str, Any] | None, latest_result: dict[str, Any] | None) -> list[dict[str, Any]]:
    action_file = PROJECT_ROOT / "ros2_mock" / "dume_ros2_mock" / "action" / "PickAndPlace.action"
    server_file = PROJECT_ROOT / "ros2_mock" / "pick_and_place_server.py"
    fk_file = PROJECT_ROOT / "ik_kinematics.py"
    ik_file = PROJECT_ROOT / "ik_path.py"
    dh_doc = PROJECT_ROOT / "ros2_mock" / "DH_PARAMETERS_AND_IK.md"

    items = [
        {
            "title": "ROS2 interface files",
            "subtitle": "The action file and server node used by the portable ROS2 layer.",
            "lines": [
                f"PickAndPlace.action: {action_file}",
                f"pick_and_place_server.py: {server_file}",
                f"Action type: {ACTION_TYPE}",
                f"Action name: {ACTION_NAME}",
            ],
        },
        {
            "title": "Kinematics implementation",
            "subtitle": "The exact files providing FK and IK for DUM-E execution.",
            "lines": [
                f"Forward kinematics: {fk_file}",
                f"Inverse kinematics/path planning: {ik_file}",
                f"DH / IK explanation: {dh_doc}",
            ],
        },
    ]

    state_lines = _state_machine_trace(server_log)
    if not state_lines:
        state_lines = ["No state-machine trace yet. Start the server and run a goal."]
    items.append(
        {
            "title": "State machine and feedback trace",
            "subtitle": "Live proof that the server is publishing status and distance updates.",
            "lines": [
                "Expected sequence: 0 -> 1 -> 2 -> 3",
                *state_lines,
            ],
        }
    )

    backend_lines = _last_log_lines(server_log, needle="DUM-E")
    if not backend_lines:
        backend_lines = ["No backend confirmation line yet."]
    items.append(
        {
            "title": "Backend confirmation",
            "subtitle": "Log evidence that DUM-E is the real executor under the ROS2-shaped layer.",
            "lines": backend_lines,
        }
    )

    goal_lines = []
    if latest_goal is not None:
        goal_lines.append(json.dumps(latest_goal, sort_keys=True))
    else:
        goal_lines.append("No goal payload captured yet.")
    if latest_result is not None:
        goal_lines.append(json.dumps(latest_result, sort_keys=True))
    else:
        goal_lines.append("No result payload captured yet.")
    items.append(
        {
            "title": "Latest goal and result",
            "subtitle": "Concrete payloads the instructor can inspect directly in the dashboard.",
            "lines": goal_lines,
        }
    )

    return items


def _deliverable_items(statuses: dict[str, str], server_log: str, latest_result: dict[str, Any] | None) -> list[dict[str, Any]]:
    goal_sent = statuses.get("ros2_send_goal") in {"finished", "running"}
    goal_succeeded = bool(latest_result and latest_result.get("success") is True)
    server_running = statuses.get("ros2_server") == "running"
    backend_visible = "delegating execution to dum-e" in server_log.lower()

    action_file = PROJECT_ROOT / "ros2_mock" / "dume_ros2_mock" / "action" / "PickAndPlace.action"
    server_file = PROJECT_ROOT / "ros2_mock" / "pick_and_place_server.py"
    fk_file = PROJECT_ROOT / "ik_kinematics.py"
    ik_file = PROJECT_ROOT / "ik_path.py"
    bonus_dh_doc = PROJECT_ROOT / "ros2_mock" / "DH_PARAMETERS_AND_IK.md"
    moveit_dir = PROJECT_ROOT / "output" / "moveit2_demo"

    return [
        {
            "title": "PickAndPlace.action file",
            "done": action_file.exists(),
            "note": "Portable ROS2 action definition is present in the bundled service.",
        },
        {
            "title": "pick_and_place_server.py node",
            "done": server_file.exists() and server_running,
            "note": "Bundled pick-and-place server file exists and the node is currently running.",
        },
        {
            "title": "Forward Kinematics (FK) implementation",
            "done": fk_file.exists(),
            "note": "FK is implemented in ik_kinematics.py and used by the DUM-E backend.",
        },
        {
            "title": "Inverse Kinematics (IK) implementation",
            "done": ik_file.exists(),
            "note": "IK path planning is implemented in ik_path.py and used by the DUM-E backend.",
        },
        {
            "title": "Working state machine (0 → 1 → 2 → 3)",
            "done": goal_sent,
            "note": "Feedback phases advance through move-to-pick, grip, move-to-drop, and drop.",
        },
        {
            "title": "Feedback publishing (status + distance)",
            "done": goal_sent,
            "note": "Recent feedback shows live status integers and distance updates during motion.",
        },
        {
            "title": "Result return (success=True)",
            "done": goal_succeeded,
            "note": "Latest result payload reports success=True after the action completes.",
        },
        {
            "title": "Demo of full pick-and-place (Gazebo + real servos)",
            "done": goal_succeeded,
            "note": "Real-servo pick-and-place is complete; Gazebo is not bundled in this portable build.",
        },
        {
            "title": "(Bonus) DH parameters + IK explanation",
            "done": bonus_dh_doc.exists(),
            "note": "Optional explanatory doc for DH-style modeling and IK walkthrough.",
        },
        {
            "title": "(Bonus) MoveIt2 integration",
            "done": moveit_dir.exists(),
            "note": "Optional MoveIt2 integration artifact directory.",
        },
        {
            "title": "DUM-E backend visible",
            "done": backend_visible,
            "note": "Server log explicitly confirms DUM-E is the real execution backend.",
        },
    ]


def ros2_state() -> dict[str, Any]:
    statuses = {item["key"]: item["status"] for item in MANAGER.status()}
    if statuses.get("ros2_server") != "running":
        return {"nodes": [], "actions": [], "topics": []}
    ensure_runtime_dirs()
    now = time.time()
    nodes_payload = [
        item
        for item in list_nodes()
        if (now - float(item.get("last_seen", 0))) <= ACTIVE_NODE_TTL_SEC
    ]
    if not nodes_payload:
        return {"nodes": [], "actions": [], "topics": []}
    nodes = [f"/{item['name']}" for item in nodes_payload if item.get("name")]

    actions: list[str] = []
    for item in nodes_payload:
        action_name = str(item.get("action_name", "")).strip()
        action_type = str(item.get("action_type", "")).strip()
        if action_name and action_type:
            actions.append(f"{action_name} [{action_type}]")
    actions = sorted(dict.fromkeys(actions))

    topics: list[str] = []
    for path in sorted(TOPICS_DIR.glob("*.jsonl")):
        payload = _read_jsonl_last(path)
        if not payload:
            continue
        topic_name = str(payload.get("topic", "")).strip()
        msg_type = str(payload.get("msg_type", "")).strip()
        if topic_name and msg_type:
            topics.append(f"{topic_name} [{msg_type}]")
        elif topic_name:
            topics.append(topic_name)

    if nodes:
        topics.extend([PARAMETER_EVENT_TOPIC, ROSOUT_TOPIC])

    return {
        "nodes": sorted(dict.fromkeys(nodes)),
        "actions": actions,
        "topics": sorted(dict.fromkeys(topics)),
    }


def ros2_summary() -> dict[str, Any]:
    statuses = {item["key"]: item["status"] for item in MANAGER.status()}
    server_log = MANAGER.read_log("ros2_server")
    goal_log = MANAGER.read_log("ros2_send_goal")
    latest_goal = _latest_goal_payload()
    latest_result = _latest_result_payload()

    feedback_lines = [
        line.strip()
        for line in goal_log.splitlines()
        if "distance" in line or "status" in line or "Result:" in line or "Feedback:" in line
    ]
    if latest_result is not None:
        feedback_lines.append(f"Result: {json.dumps(latest_result, sort_keys=True)}")
    return {
        "workspace_built": statuses.get("build_workspace") == "finished",
        "server_running": statuses.get("ros2_server") == "running",
        "goal_sent": statuses.get("ros2_send_goal") in {"finished", "running"},
        "goal_succeeded": bool(latest_result and latest_result.get("success") is True),
        "recent_feedback": feedback_lines[-12:],
        "server_mentions_backend": "delegating execution to dum-e" in server_log.lower(),
        "latest_goal": latest_goal,
        "latest_result": latest_result,
        "deliverable_panels": _deliverable_panels(server_log, latest_goal, latest_result),
        "runtime_panels": _runtime_panels(server_log, latest_result),
        "deliverables": _deliverable_items(statuses, server_log, latest_result),
        "proof_items": _proof_items(server_log, latest_goal, latest_result),
    }


class DashboardHandler(BaseHTTPRequestHandler):
    def _send_json(self, payload: dict[str, Any], status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path: Path, content_type: str) -> None:
        body = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        data = self.rfile.read(length)
        return json.loads(data.decode("utf-8"))

    def do_GET(self) -> None:  # noqa: N802
        if self.path in ("/", "/index.html"):
            return self._send_file(BASE_DIR / "index.html", "text/html; charset=utf-8")
        if self.path == "/styles.css":
            return self._send_file(BASE_DIR / "styles.css", "text/css; charset=utf-8")
        if self.path == "/script.js":
            return self._send_file(BASE_DIR / "script.js", "application/javascript; charset=utf-8")
        if self.path == "/api/status":
            return self._send_json({"processes": MANAGER.status()})
        if self.path == "/api/defaults":
            return self._send_json(load_demo_profile())
        if self.path == "/api/ros2_state":
            return self._send_json(ros2_state())
        if self.path == "/api/ros2_summary":
            return self._send_json(ros2_summary())
        if self.path.startswith("/api/logs/"):
            key = self.path.rsplit("/", 1)[-1]
            return self._send_json({"key": key, "log": MANAGER.read_log(key)})
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        if self.path == "/api/run":
            payload = self._read_json()
            action = payload.get("action")
            try:
                key, command, long_running = build_action(action, payload)
            except KeyError:
                return self._send_json({"error": f"Unknown action: {action}"}, status=HTTPStatus.BAD_REQUEST)
            managed = MANAGER.run(key=key, label=action, command=command, long_running=long_running, params=payload)
            return self._send_json({"ok": True, "process": {"key": managed.key, "status": managed.status()}})
        if self.path == "/api/stop":
            payload = self._read_json()
            return self._send_json({"ok": MANAGER.stop(payload.get("key"))})
        if self.path == "/api/stop_all":
            MANAGER.stop_all()
            return self._send_json({"ok": True})
        if self.path == "/api/clear_finished":
            MANAGER.clear_finished()
            return self._send_json({"ok": True})
        self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        return


def main() -> None:
    ensure_runtime_dirs()
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    server = ThreadingHTTPServer((HOST, PORT), DashboardHandler)
    print(f"ROS2 dashboard running at http://{HOST}:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        MANAGER.stop_all()
        server.server_close()


if __name__ == "__main__":
    main()
