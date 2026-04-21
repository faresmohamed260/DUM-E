from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any
from uuid import uuid4


APP_ROOT = Path(__file__).resolve().parent.parent
RUNTIME_ROOT = APP_ROOT / "output" / "ros2_mock"
GOALS_DIR = RUNTIME_ROOT / "actions" / "goals"
FEEDBACK_DIR = RUNTIME_ROOT / "actions" / "feedback"
RESULTS_DIR = RUNTIME_ROOT / "actions" / "results"
NODES_DIR = RUNTIME_ROOT / "nodes"
TOPICS_DIR = RUNTIME_ROOT / "topics"


def ensure_runtime_dirs() -> None:
    for path in (GOALS_DIR, FEEDBACK_DIR, RESULTS_DIR, NODES_DIR, TOPICS_DIR):
        path.mkdir(parents=True, exist_ok=True)


def create_goal_id() -> str:
    return uuid4().hex[:12]


def goal_path(goal_id: str) -> Path:
    return GOALS_DIR / f"{goal_id}.json"


def feedback_path(goal_id: str) -> Path:
    return FEEDBACK_DIR / f"{goal_id}.jsonl"


def result_path(goal_id: str) -> Path:
    return RESULTS_DIR / f"{goal_id}.json"


def topic_path(topic_name: str) -> Path:
    slug = topic_name.strip("/").replace("/", "__")
    return TOPICS_DIR / f"{slug}.jsonl"


def write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def append_jsonl(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload) + "\n")


def publish_topic(topic_name: str, payload: dict) -> None:
    message = {"topic": topic_name, "published_at": time.time(), **payload}
    append_jsonl(topic_path(topic_name), message)


def publish_feedback(goal_id: str, payload: dict) -> None:
    append_jsonl(feedback_path(goal_id), payload)


def write_result(goal_id: str, payload: dict) -> None:
    write_json(result_path(goal_id), payload)


def register_node(node_name: str, metadata: dict | None = None) -> None:
    ensure_runtime_dirs()
    payload = {
        "name": node_name,
        "pid": os.getpid(),
        "registered_at": time.time(),
        "last_seen": time.time(),
    }
    if metadata:
        payload.update(metadata)
    write_json(NODES_DIR / f"{node_name}.json", payload)


def heartbeat_node(node_name: str) -> None:
    path = NODES_DIR / f"{node_name}.json"
    if not path.exists():
        register_node(node_name)
        return
    payload = read_json(path)
    payload["last_seen"] = time.time()
    write_json(path, payload)


def list_nodes() -> list[dict]:
    ensure_runtime_dirs()
    items: list[dict] = []
    for path in sorted(NODES_DIR.glob("*.json")):
        try:
            items.append(read_json(path))
        except Exception:
            continue
    return items
