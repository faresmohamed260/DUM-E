from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from ros2_mock.pick_and_place_server import ACTION_NAME, ACTION_TYPE, NODE_NAME
from ros2_mock.runtime import create_goal_id, ensure_runtime_dirs, feedback_path, goal_path, list_nodes, read_json, result_path, topic_path, write_json


def cmd_node_list(_: argparse.Namespace) -> int:
    nodes = list_nodes()
    if not nodes:
        print("(no mock ROS2 nodes registered)")
        return 0
    for node in nodes:
        print(node["name"])
    return 0


def cmd_action_send_goal(args: argparse.Namespace) -> int:
    ensure_runtime_dirs()
    if args.action_name != ACTION_NAME:
        raise SystemExit(f"Unsupported action name: {args.action_name}")
    if args.action_type != ACTION_TYPE:
        raise SystemExit(f"Unsupported action type: {args.action_type}")
    payload = json.loads(args.goal_json)
    goal_id = create_goal_id()
    goal = {
        "goal_id": goal_id,
        "action_name": args.action_name,
        "action_type": args.action_type,
        "pick_pose": payload["pick_pose"],
        "drop_pose": payload["drop_pose"],
        "status": "pending",
        "created_at": time.time(),
    }
    write_json(goal_path(goal_id), goal)
    print(f"Waiting for an action server to become available...")
    print(f"Sending goal:\n{json.dumps(goal, indent=2)}")

    feedback_file = feedback_path(goal_id)
    seen = 0
    while not result_path(goal_id).exists():
        if args.feedback and feedback_file.exists():
            lines = feedback_file.read_text(encoding="utf-8").splitlines()
            for line in lines[seen:]:
                print(f"Feedback: {line}")
            seen = len(lines)
        time.sleep(0.25)

    result = read_json(result_path(goal_id))
    print(f"Result:\n{json.dumps(result, indent=2)}")
    return 0 if result.get("success") else 1


def cmd_topic_echo(args: argparse.Namespace) -> int:
    path = topic_path(args.topic_name)
    print(f"echoing {args.topic_name} from {path}")
    seen = 0
    while True:
        if path.exists():
            lines = path.read_text(encoding="utf-8").splitlines()
            for line in lines[seen:]:
                print(line)
            seen = len(lines)
        if not args.follow:
            break
        time.sleep(0.25)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS2-like CLI for the Lab 3 DUM-E wrapper.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    node_parser = subparsers.add_parser("node")
    node_sub = node_parser.add_subparsers(dest="node_command", required=True)
    node_list = node_sub.add_parser("list")
    node_list.set_defaults(func=cmd_node_list)

    action_parser = subparsers.add_parser("action")
    action_sub = action_parser.add_subparsers(dest="action_command", required=True)
    send_goal = action_sub.add_parser("send_goal")
    send_goal.add_argument("action_name")
    send_goal.add_argument("action_type")
    send_goal.add_argument("goal_json")
    send_goal.add_argument("--feedback", action="store_true")
    send_goal.set_defaults(func=cmd_action_send_goal)

    topic_parser = subparsers.add_parser("topic")
    topic_sub = topic_parser.add_subparsers(dest="topic_command", required=True)
    echo = topic_sub.add_parser("echo")
    echo.add_argument("topic_name")
    echo.add_argument("--follow", action="store_true")
    echo.set_defaults(func=cmd_topic_echo)

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    raise SystemExit(args.func(args))


if __name__ == "__main__":
    main()
