from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from ros2_mock.dume_bridge import DumeBridge, GoalPose
from ros2_mock.runtime import GOALS_DIR, ensure_runtime_dirs, heartbeat_node, publish_feedback, register_node, read_json, write_result


NODE_NAME = "pick_and_place_server"
ACTION_NAME = "/pick_and_place"
ACTION_TYPE = "dume_ros2_mock/action/PickAndPlace"


class PickAndPlaceServer:
    def __init__(
        self,
        *,
        base_url: str | None,
        safe_height_mm: float,
        steps_per_segment: int,
        waypoint_delay_ms: int,
        arrival_tolerance_mm: int,
        gripper_close_angle: int | None,
        dry_run: bool,
    ) -> None:
        ensure_runtime_dirs()
        register_node(
            NODE_NAME,
            {
                "action_name": ACTION_NAME,
                "action_type": ACTION_TYPE,
                "base_url": base_url or "auto",
            },
        )
        self.bridge = DumeBridge(base_url)
        self.safe_height_mm = safe_height_mm
        self.steps_per_segment = steps_per_segment
        self.waypoint_delay_ms = waypoint_delay_ms
        self.arrival_tolerance_mm = arrival_tolerance_mm
        self.gripper_close_angle = gripper_close_angle
        self.dry_run = dry_run

    def serve_forever(self, poll_interval: float = 0.5) -> None:
        print(f"[{NODE_NAME}] ready on {ACTION_NAME} ({ACTION_TYPE})")
        print(f"[{NODE_NAME}] delegating execution to DUM-E at {self.bridge.base_url}")
        print(f"[{NODE_NAME}] state machine: 0 -> 1 -> 2 -> 3")
        print(f"[{NODE_NAME}] FK/IK backend active via DUM-E kinematics modules")
        while True:
            heartbeat_node(NODE_NAME)
            for goal_file in sorted(GOALS_DIR.glob("*.json")):
                goal = read_json(goal_file)
                if goal.get("status") != "pending":
                    continue
                self._handle_goal(goal_file, goal)
            time.sleep(poll_interval)

    def _handle_goal(self, goal_file: Path, goal: dict) -> None:
        goal_id = goal["goal_id"]
        print(f"[{NODE_NAME}] accepting goal {goal_id}")
        goal["status"] = "active"
        goal["started_at"] = time.time()
        goal_file.write_text(json.dumps(goal, indent=2), encoding="utf-8")

        pick = self._goal_pose(goal["pick_pose"])
        drop = self._goal_pose(goal["drop_pose"])
        print(f"[{NODE_NAME}] goal {goal_id} received pick={pick} drop={drop}")

        try:
            self._phase_move(goal_id, status=0, target=pick)
            publish_feedback(goal_id, {"status": 1, "distance": 0, "phase": "gripping"})
            print(f"[{NODE_NAME}] goal {goal_id} status=1 phase=gripping")
            close_value = self.bridge.close_gripper(self.gripper_close_angle, dry_run=self.dry_run)
            publish_feedback(goal_id, {"status": 1, "distance": 0, "phase": "gripping", "gripper_close_value": close_value})
            if not self.dry_run:
                time.sleep(max(self.waypoint_delay_ms, 250) / 1000.0)

            self._phase_move(goal_id, status=2, target=drop)
            publish_feedback(goal_id, {"status": 3, "distance": 0, "phase": "dropping"})
            print(f"[{NODE_NAME}] goal {goal_id} status=3 phase=dropping")
            open_value = self.bridge.open_gripper(dry_run=self.dry_run)
            publish_feedback(goal_id, {"status": 3, "distance": 0, "phase": "dropping", "gripper_open_value": open_value})
            if not self.dry_run:
                time.sleep(max(self.waypoint_delay_ms, 250) / 1000.0)

            write_result(
                goal_id,
                {
                    "goal_id": goal_id,
                    "success": True,
                    "finished_at": time.time(),
                },
            )
            goal["status"] = "done"
            goal_file.write_text(json.dumps(goal, indent=2), encoding="utf-8")
            print(f"[{NODE_NAME}] goal {goal_id} completed result={{\"success\": true}}")
        except Exception as exc:
            write_result(
                goal_id,
                {
                    "goal_id": goal_id,
                    "success": False,
                    "error": str(exc),
                    "finished_at": time.time(),
                },
            )
            goal["status"] = "failed"
            goal["error"] = str(exc)
            goal_file.write_text(json.dumps(goal, indent=2), encoding="utf-8")
            print(f"[{NODE_NAME}] goal {goal_id} failed result={{\"success\": false, \"error\": {json.dumps(str(exc))}}}")

    def _phase_move(self, goal_id: str, *, status: int, target: GoalPose) -> None:
        commands = self.bridge.build_motion_commands(
            target,
            safe_height_mm=self.safe_height_mm,
            steps_per_segment=self.steps_per_segment,
            use_safe_height=True,
        )

        last_pose = None

        def emit_feedback(current_pose) -> None:
            nonlocal last_pose
            last_pose = current_pose
            distance = self.bridge.distance_to_goal_mm(current_pose, target)
            phase = "moving_to_object" if status == 0 else "moving_to_drop"
            print(f"[{NODE_NAME}] goal {goal_id} status={status} distance={distance} phase={phase}")
            publish_feedback(goal_id, {"status": status, "distance": distance, "phase": phase})

        self.bridge.execute_motion_commands(
            commands,
            delay_after_ms=self.waypoint_delay_ms,
            dry_run=self.dry_run,
            feedback_cb=emit_feedback,
        )
        if last_pose is None:
            last_pose = self.bridge.current_cartesian_pose()
        final_distance = self.bridge.distance_to_goal_mm(last_pose, target)
        if final_distance > self.arrival_tolerance_mm:
            raise RuntimeError(
                f"Arrival tolerance exceeded for status {status}: {final_distance} mm > {self.arrival_tolerance_mm} mm."
            )

    def _goal_pose(self, payload: dict) -> GoalPose:
        position = payload["position"]
        return GoalPose(
            x_mm=float(position["x"]) * 1000.0,
            y_mm=float(position["y"]) * 1000.0,
            z_mm=float(position["z"]) * 1000.0,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mock ROS2 pick-and-place server backed by DUM-E.")
    parser.add_argument("--base-url", default=None, help="DUM-E base URL. Defaults to device_cache.json or http://192.168.1.132")
    parser.add_argument("--safe-height-mm", type=float, default=200.0)
    parser.add_argument("--steps-per-segment", type=int, default=6)
    parser.add_argument("--waypoint-delay-ms", type=int, default=200)
    parser.add_argument("--arrival-tolerance-mm", type=int, default=10)
    parser.add_argument("--gripper-close-angle", type=int, default=20)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    server = PickAndPlaceServer(
        base_url=args.base_url,
        safe_height_mm=args.safe_height_mm,
        steps_per_segment=args.steps_per_segment,
        waypoint_delay_ms=args.waypoint_delay_ms,
        arrival_tolerance_mm=args.arrival_tolerance_mm,
        gripper_close_angle=args.gripper_close_angle,
        dry_run=args.dry_run,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
