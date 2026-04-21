from __future__ import annotations

import json
from pathlib import Path

from ros2_mock.pick_and_place_server import ACTION_NAME, ACTION_TYPE, NODE_NAME
from ros2_mock.runtime import FEEDBACK_DIR, GOALS_DIR, NODES_DIR, RESULTS_DIR, TOPICS_DIR, ensure_runtime_dirs


def _print_path_status(label: str, path: Path) -> None:
    status = "OK" if path.exists() else "MISSING"
    print(f"[build_workspace] {label}: {status}")
    print(f"[build_workspace]   path: {path}")


def _print_file_excerpt(label: str, path: Path, *, max_lines: int = 8) -> None:
    print(f"[build_workspace] {label} excerpt:")
    if not path.exists():
        print("[build_workspace]   file missing")
        return
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except Exception as exc:
        print(f"[build_workspace]   unreadable: {exc}")
        return
    excerpt = lines[:max_lines]
    for index, line in enumerate(excerpt, start=1):
        print(f"[build_workspace]   {index:>2}: {line}")


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    ensure_runtime_dirs()

    action_file = root / "ros2_mock" / "dume_ros2_mock" / "action" / "PickAndPlace.action"
    server_file = root / "ros2_mock" / "pick_and_place_server.py"
    fk_file = root / "ik_kinematics.py"
    ik_file = root / "ik_path.py"
    dh_doc = root / "ros2_mock" / "DH_PARAMETERS_AND_IK.md"
    demo_profile = root / "ros2_dashboard" / "demo_profile.json"
    shared_profile = root / "ros2_dashboard" / "shared_profile.json"

    print("[build_workspace] Portable ROS2 runtime prepared.")
    print(f"[build_workspace] Node name: /{NODE_NAME}")
    print(f"[build_workspace] Action name: {ACTION_NAME}")
    print(f"[build_workspace] Action type: {ACTION_TYPE}")
    print("[build_workspace] Runtime directories validated:")
    for path in (GOALS_DIR, FEEDBACK_DIR, RESULTS_DIR, NODES_DIR, TOPICS_DIR):
        print(f"[build_workspace]   {path}")

    _print_path_status("PickAndPlace.action", action_file)
    _print_path_status("pick_and_place_server.py", server_file)
    _print_path_status("FK implementation", fk_file)
    _print_path_status("IK implementation", ik_file)
    _print_path_status("DH/IK explanation", dh_doc)
    _print_path_status("dashboard demo profile", demo_profile)
    _print_path_status("dashboard shared profile", shared_profile)

    _print_file_excerpt("PickAndPlace.action", action_file, max_lines=9)
    _print_file_excerpt("pick_and_place_server.py", server_file, max_lines=12)

    if demo_profile.exists():
        try:
            payload = json.loads(demo_profile.read_text(encoding="utf-8"))
            print("[build_workspace] Demo profile:")
            print(json.dumps(payload, indent=2, sort_keys=True))
        except Exception as exc:
            print(f"[build_workspace] Demo profile unreadable: {exc}")


if __name__ == "__main__":
    main()
