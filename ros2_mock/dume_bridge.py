from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import requests

from ik_kinematics import Pose3D, ServoPose, forward_kinematics, inverse_kinematics, load_geometry, mechanical_to_servo, servo_to_mechanical
from ros2_mock.runtime import APP_ROOT, publish_topic


IK_JOINTS = ("base", "shoulder", "elbow")


@dataclass(frozen=True)
class MotionCommand:
    base: int
    shoulder: int
    elbow: int


@dataclass(frozen=True)
class GoalPose:
    x_mm: float
    y_mm: float
    z_mm: float


class DumeBridge:
    def __init__(self, base_url: str | None = None) -> None:
        self.base_url = self._resolve_base_url(base_url)
        self.geometry = load_geometry(APP_ROOT / "config" / "arm_geometry.json")

    def fetch_state(self) -> dict:
        response = requests.get(f"{self.base_url}/api/state", timeout=8)
        response.raise_for_status()
        payload = response.json()
        if not payload.get("ok", True):
            raise RuntimeError(payload.get("error", "state_request_failed"))
        return payload

    def current_servo_pose(self) -> dict[str, int]:
        payload = self.fetch_state()
        joints = {
            joint["name"]: int(joint["position"])
            for joint in payload.get("joints", [])
            if joint.get("name") in IK_JOINTS
        }
        missing = [name for name in IK_JOINTS if name not in joints]
        if missing:
            raise RuntimeError(f"Missing DUM-E joints: {', '.join(missing)}")
        return joints

    def current_cartesian_pose(self) -> Pose3D:
        servo = self.current_servo_pose()
        mechanical = servo_to_mechanical(
            self.geometry,
            ServoPose(base=servo["base"], shoulder=servo["shoulder"], elbow=servo["elbow"]),
        )
        return forward_kinematics(self.geometry, mechanical)

    def gripper_limits(self) -> tuple[int, int]:
        payload = self.fetch_state()
        for joint in payload.get("joints", []):
            if joint.get("name") == "gripper":
                minimum = int(joint.get("min_angle", joint.get("stored_min_angle", 20)))
                maximum = int(joint.get("max_angle", joint.get("stored_max_angle", 160)))
                return minimum, maximum
        raise RuntimeError("Gripper joint not found in DUM-E state.")

    def move_gripper(self, angle: int, *, dry_run: bool = False) -> dict | None:
        params = {"cmd": "move", "joint": "gripper", "value": int(angle)}
        publish_topic(
            "/arm_controller/joint_trajectory",
            {
                "msg_type": "trajectory_msgs/msg/JointTrajectory",
                "joint_names": ["gripper"],
                "points": [{"positions": [int(angle)]}],
            },
        )
        if dry_run:
            return None
        return self._run_command("/api/joint", params)

    def open_gripper(self, *, dry_run: bool = False) -> int:
        _, maximum = self.gripper_limits()
        self.move_gripper(maximum, dry_run=dry_run)
        return maximum

    def close_gripper(self, close_angle: int | None = None, *, dry_run: bool = False) -> int:
        minimum, maximum = self.gripper_limits()
        target = minimum if close_angle is None else max(minimum, min(maximum, int(close_angle)))
        self.move_gripper(target, dry_run=dry_run)
        return target

    def build_motion_commands(
        self,
        target: GoalPose,
        *,
        safe_height_mm: float,
        steps_per_segment: int,
        use_safe_height: bool = True,
    ) -> list[MotionCommand]:
        current = self.current_cartesian_pose()
        waypoints = self._build_waypoints(current, target, safe_height_mm, use_safe_height)
        commands: list[MotionCommand] = []
        for point in self._interpolate_waypoints(waypoints, max(2, int(steps_per_segment))):
            self._validate_workspace_point(point)
            solution = inverse_kinematics(self.geometry, point)
            servo = mechanical_to_servo(self.geometry, solution)
            cmd = MotionCommand(base=servo.base, shoulder=servo.shoulder, elbow=servo.elbow)
            if not commands or cmd != commands[-1]:
                commands.append(cmd)
        return commands

    def execute_motion_commands(
        self,
        commands: list[MotionCommand],
        *,
        delay_after_ms: int,
        dry_run: bool,
        feedback_cb: Callable[[Pose3D], None] | None = None,
    ) -> None:
        delay_seconds = max(0, int(delay_after_ms)) / 1000.0
        for command in commands:
            payload = {
                "msg_type": "trajectory_msgs/msg/JointTrajectory",
                "joint_names": list(IK_JOINTS),
                "points": [{"positions": [command.base, command.shoulder, command.elbow]}],
            }
            publish_topic("/arm_controller/joint_trajectory", payload)
            for joint_name in IK_JOINTS:
                params = {"cmd": "move", "joint": joint_name, "value": int(getattr(command, joint_name))}
                if not dry_run:
                    self._run_command("/api/joint", params)
            if not dry_run and delay_seconds > 0:
                time.sleep(delay_seconds)
            current_pose = self.current_cartesian_pose() if not dry_run else self._pose_from_command(command)
            publish_topic(
                "/joint_states",
                {
                    "msg_type": "sensor_msgs/msg/JointState",
                    "name": list(IK_JOINTS),
                    "position": [command.base, command.shoulder, command.elbow],
                    "cartesian_pose_mm": {"x": current_pose.x, "y": current_pose.y, "z": current_pose.z},
                },
            )
            if feedback_cb is not None:
                feedback_cb(current_pose)

    def distance_to_goal_mm(self, current: Pose3D, target: GoalPose) -> int:
        distance = math.sqrt(
            ((current.x - target.x_mm) ** 2)
            + ((current.y - target.y_mm) ** 2)
            + ((current.z - target.z_mm) ** 2)
        )
        return int(round(distance))

    def _build_waypoints(self, current: Pose3D, target: GoalPose, safe_height_mm: float, use_safe_height: bool) -> list[Pose3D]:
        target_pose = Pose3D(target.x_mm, target.y_mm, target.z_mm)
        if not use_safe_height:
            return [target_pose]
        current_lift = Pose3D(current.x, current.y, max(current.z, safe_height_mm))
        target_lift = Pose3D(target.x_mm, target.y_mm, max(target.z_mm, safe_height_mm))
        return [current_lift, target_lift, target_pose]

    def _interpolate_waypoints(self, waypoints: list[Pose3D], steps_per_segment: int) -> list[Pose3D]:
        points: list[Pose3D] = []
        for index in range(len(waypoints) - 1):
            start = waypoints[index]
            end = waypoints[index + 1]
            for step in range(1, steps_per_segment + 1):
                alpha = step / float(steps_per_segment)
                point = Pose3D(
                    x=start.x + (end.x - start.x) * alpha,
                    y=start.y + (end.y - start.y) * alpha,
                    z=start.z + (end.z - start.z) * alpha,
                )
                if not points or point != points[-1]:
                    points.append(point)
        return points

    def _validate_workspace_point(self, point: Pose3D) -> None:
        half_width = self.geometry.platform_width_mm / 2.0
        half_depth = self.geometry.platform_depth_mm / 2.0
        inside_platform = (-half_width <= point.x <= half_width) and (-half_depth <= point.y <= half_depth)
        if inside_platform and point.z < self.geometry.platform_safe_clearance_mm:
            raise ValueError(
                f"Waypoint ({point.x:.1f}, {point.y:.1f}, {point.z:.1f}) mm intersects the platform footprint."
            )

    def _pose_from_command(self, command: MotionCommand) -> Pose3D:
        mechanical = servo_to_mechanical(
            self.geometry,
            ServoPose(base=command.base, shoulder=command.shoulder, elbow=command.elbow),
        )
        return forward_kinematics(self.geometry, mechanical)

    def _run_command(self, endpoint: str, params: dict) -> dict:
        response = requests.get(f"{self.base_url}{endpoint}", params=params, timeout=8)
        response.raise_for_status()
        payload = response.json()
        if not payload.get("ok", True):
            raise RuntimeError(payload.get("error", "device_request_failed"))
        return payload

    def _resolve_base_url(self, explicit: str | None) -> str:
        if explicit:
            return explicit.rstrip("/")
        cache_path = APP_ROOT / "device_cache.json"
        if cache_path.exists():
            try:
                payload = json.loads(cache_path.read_text(encoding="utf-8"))
                last = payload.get("last_device", {})
                base_url = str(last.get("base_url", "")).strip()
                if base_url:
                    return base_url.rstrip("/")
            except Exception:
                pass
        return "http://192.168.1.132"
