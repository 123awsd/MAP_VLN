#!/usr/bin/env python3
"""Use Qwen to assign semantic names to already segmented rooms.

OccuSG owns room geometry and adjacency.  This script only sends the contents
of each existing region (Boxer labels, confidence, relative box geometry and
region metadata) to Qwen, then writes an auditable scene-graph copy.  Qwen is
not allowed to create rooms, move objects, or generate coordinates.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import tempfile
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
ENDPOINT = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
MODEL = "qwen3.7-plus"
PROMPT_VERSION = "stage1-room-semantic-v1"
INPUT_CNY_PER_MILLION = 2.0
OUTPUT_CNY_PER_MILLION = 8.0
ROOM_TYPES = {
    "bedroom", "living_room", "kitchen", "dining_room", "bathroom",
    "office", "laundry_utility", "entry_hall", "corridor", "unknown",
}
ALIASES = {
    "living room": "living_room",
    "livingroom": "living_room",
    "bed room": "bedroom",
    "dining room": "dining_room",
    "bath room": "bathroom",
    "study": "office",
    "study_room": "office",
    "laundry room": "laundry_utility",
    "utility room": "laundry_utility",
    "hallway": "entry_hall",
    "hall": "entry_hall",
    "corridor": "corridor",
    "unknown": "unknown",
}


SYSTEM_PROMPT = """你是室内场景图的房间语义分类器。几何分割已经由 OccuSG 完成，你不能修改房间边界、合并房间、拆分房间或生成任何坐标。
现在只根据每个已有房间中的 Boxer 物体清单、物体检测置信度、相对位置/尺寸、房间面积和邻接度，判断该房间的语义类型。
允许的 room_type 只有：bedroom、living_room、kitchen、dining_room、bathroom、office、laundry_utility、entry_hall、corridor、unknown。
规则：
- 证据不足时必须输出 unknown，不要猜测；
- 只引用输入中已有的 object_id 作为 evidence_object_ids；
- corridor 主要由狭长几何和多邻接关系判断；
- 不要把一个孤立的 chair、table、cabinet 或 television 当成足够的房间证据；
- confidence 表示房间语义判断置信度，不是 Boxer 检测置信度；
- 每个输入 room_id 必须且只能返回一次；
- 只输出严格 JSON，不要 Markdown。
格式：
{"rooms":[{"room_id":"3","room_type":"bedroom","confidence":0.88,"evidence_object_ids":["boxer_2"],"alternative_types":["office"],"reason":"bed and nightstand are strong bedroom evidence"}]}"""


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", dir=path.parent, delete=False, encoding="utf-8"
    ) as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        temporary = Path(handle.name)
    temporary.replace(path)


def extract_json(text: str) -> dict[str, Any]:
    text = text.strip()
    fenced = re.fullmatch(
        r"```(?:json)?\s*(.*?)\s*```", text, re.DOTALL | re.IGNORECASE
    )
    return json.loads(fenced.group(1) if fenced else text)


def room_payload(rooms: list[dict[str, Any]]) -> list[dict[str, Any]]:
    payload = []
    for room in rooms:
        centroid = room.get("centroid_xy_m", [0.0, 0.0])
        objects = []
        for obj in room.get("objects", []):
            center = obj.get("center_xyz_m", [0.0, 0.0, 0.0])
            objects.append({
                "object_id": str(obj["id"]),
                "label": str(obj.get("label", "unknown")).strip().lower(),
                "detection_confidence": round(float(obj.get("probability", 0.0)), 4),
                "relative_xy_m": [
                    round(float(center[0]) - float(centroid[0]), 3),
                    round(float(center[1]) - float(centroid[1]), 3),
                ],
                "size_xyz_m": [round(float(value), 3) for value in obj.get("size_xyz_m", [])],
                "assignment": obj.get("room_assignment", "inside_polygon"),
            })
        payload.append({
            "room_id": str(room["id"]),
            "space_role": str(room.get("space_role", "room")),
            "area_m2": round(float(room.get("area_m2", 0.0)), 3),
            "adjacent_room_count": len(room.get("adjacent_room_ids", [])),
            "object_count": len(objects),
            "objects": objects,
        })
    return payload


def normalize_decisions(
    value: dict[str, Any], rooms: list[dict[str, Any]], confidence_threshold: float,
) -> list[dict[str, Any]]:
    if not isinstance(value, dict) or not isinstance(value.get("rooms"), list):
        raise ValueError("Qwen room response must contain a rooms array")
    expected = {str(room["id"]): room for room in rooms}
    decisions = {}
    for raw in value["rooms"]:
        if not isinstance(raw, dict):
            raise ValueError("each Qwen room decision must be an object")
        room_id = str(raw.get("room_id", ""))
        if room_id not in expected:
            raise ValueError(f"Qwen returned unknown room_id: {room_id}")
        if room_id in decisions:
            raise ValueError(f"Qwen returned duplicate room_id: {room_id}")
        raw_type = str(raw.get("room_type", "unknown")).strip().lower()
        room_type = ALIASES.get(raw_type, raw_type)
        if room_type not in ROOM_TYPES:
            room_type = "unknown"
        confidence = float(raw.get("confidence", 0.0))
        if not 0.0 <= confidence <= 1.0:
            raise ValueError(f"invalid room confidence for {room_id}")
        valid_object_ids = {
            str(obj["id"]) for obj in expected[room_id].get("objects", [])
        }
        evidence = [
            str(object_id) for object_id in raw.get("evidence_object_ids", [])
            if str(object_id) in valid_object_ids
        ]
        alternatives = []
        for alternative in raw.get("alternative_types", []):
            normalized = ALIASES.get(str(alternative).strip().lower(), str(alternative).strip().lower())
            if normalized in ROOM_TYPES and normalized != room_type and normalized not in alternatives:
                alternatives.append(normalized)
        reason = str(raw.get("reason", "")).strip()[:500]
        decisions[room_id] = {
            "room_id": int(room_id),
            "room_type": room_type,
            "confidence": confidence,
            "accepted": bool(room_type != "unknown" and confidence >= confidence_threshold),
            "evidence_object_ids": evidence,
            "alternative_types": alternatives,
            "reason": reason or "model returned no explanation",
        }
    for room_id, room in expected.items():
        if room_id not in decisions:
            decisions[room_id] = {
                "room_id": int(room_id),
                "room_type": "unknown",
                "confidence": 0.0,
                "accepted": False,
                "evidence_object_ids": [],
                "alternative_types": [],
                "reason": "model omitted this room; fail-safe unknown",
            }
    return [decisions[key] for key in sorted(decisions, key=lambda item: int(item))]


class QwenRoomSemanticClassifier:
    def __init__(self, budget_cny: float, confidence_threshold: float):
        self.budget_cny = float(budget_cny)
        self.confidence_threshold = float(confidence_threshold)
        self.api_key_path = ROOT / ".secrets/dashscope_api_key"
        self.cache_dir = ROOT / "outputs/stage1/room_semantic_cache"
        self.ledger_path = ROOT / "outputs/stage1/api_usage.json"

    def classify(self, rooms: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        payload_rooms = room_payload(rooms)
        cache_payload = {
            "prompt_version": PROMPT_VERSION,
            "model": MODEL,
            "confidence_threshold": self.confidence_threshold,
            "rooms": payload_rooms,
        }
        digest = hashlib.sha256(
            json.dumps(cache_payload, ensure_ascii=False, sort_keys=True).encode("utf-8")
        ).hexdigest()
        cache_path = self.cache_dir / f"{digest}.json"
        if cache_path.is_file():
            cached = json.loads(cache_path.read_text(encoding="utf-8"))
            return normalize_decisions(
                {"rooms": cached["rooms"]}, rooms, self.confidence_threshold
            ), {
                **cached.get("provenance", {}),
                "cache_hit": True,
            }

        ledger = (
            json.loads(self.ledger_path.read_text(encoding="utf-8"))
            if self.ledger_path.is_file()
            else {"format": "pre_map_vln.api_usage.v1", "total_estimated_cny": 0.0, "calls": []}
        )
        previous = float(ledger.get("total_estimated_cny", 0.0))
        if previous >= self.budget_cny:
            raise RuntimeError(f"Qwen budget reached: {previous:.4f} CNY")
        api_key = self.api_key_path.read_text(encoding="utf-8").strip()
        request_payload = {
            "model": MODEL,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {
                    "role": "user",
                    "content": "已有房间及其内部物体清单：\n"
                    + json.dumps(payload_rooms, ensure_ascii=False),
                },
            ],
            "response_format": {"type": "json_object"},
            "enable_thinking": False,
            "temperature": 0.0,
            "max_tokens": 4000,
        }
        request = urllib.request.Request(
            ENDPOINT,
            data=json.dumps(request_payload).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=180) as response:
                body = json.load(response)
        except urllib.error.HTTPError as error:
            detail = error.read().decode("utf-8", "replace")[:1000]
            raise RuntimeError(f"DashScope HTTP {error.code}: {detail}") from error
        usage = body.get("usage") or {}
        input_tokens = int(usage.get("prompt_tokens", 0))
        output_tokens = int(usage.get("completion_tokens", 0))
        cost = (
            input_tokens * INPUT_CNY_PER_MILLION
            + output_tokens * OUTPUT_CNY_PER_MILLION
        ) / 1_000_000.0
        if previous + cost > self.budget_cny:
            raise RuntimeError("Qwen room-semantic response exceeded configured budget")
        ledger["authorized_budget_cny"] = self.budget_cny
        ledger["total_estimated_cny"] = round(previous + cost, 8)
        ledger.setdefault("calls", []).append({
            "model": MODEL,
            "purpose": "room_semantic_classification",
            "prompt_tokens": input_tokens,
            "completion_tokens": output_tokens,
            "estimated_cny": round(cost, 8),
        })
        atomic_json(self.ledger_path, ledger)
        raw = extract_json(body["choices"][0]["message"]["content"])
        decisions = normalize_decisions(raw, rooms, self.confidence_threshold)
        provenance = {
            "classifier": "qwen_llm",
            "model": body.get("model", MODEL),
            "prompt_version": PROMPT_VERSION,
            "cache_hit": False,
            "usage": usage,
            "estimated_cny": round(cost, 8),
            "confidence_threshold": self.confidence_threshold,
        }
        atomic_json(cache_path, {"rooms": raw["rooms"], "provenance": provenance})
        return decisions, provenance


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-graph", type=Path, required=True)
    parser.add_argument("--output-graph", type=Path, required=True)
    parser.add_argument("--output-decisions", type=Path, required=True)
    parser.add_argument("--budget-cny", type=float, default=20.0)
    parser.add_argument("--confidence-threshold", type=float, default=0.65)
    args = parser.parse_args()
    graph = json.loads(args.scene_graph.read_text(encoding="utf-8"))
    rooms = graph.get("rooms", [])
    if not rooms:
        raise ValueError("scene graph contains no rooms")
    classifier = QwenRoomSemanticClassifier(args.budget_cny, args.confidence_threshold)
    decisions, provenance = classifier.classify(rooms)
    by_id = {int(item["room_id"]): item for item in decisions}
    graph = json.loads(json.dumps(graph, ensure_ascii=False))
    for room in graph["rooms"]:
        old_type = room.get("semantic_type", "unknown")
        decision = by_id[int(room["id"])]
        room["rule_semantic_type"] = old_type
        room["qwen_semantic_type"] = decision["room_type"]
        room["qwen_semantic_score"] = decision["confidence"]
        room["qwen_semantic"] = decision
        if room.get("space_role") == "transition_space":
            room["semantic_type"] = "corridor"
            room["semantic_score"] = 1.0
        elif decision["accepted"]:
            room["semantic_type"] = decision["room_type"]
            room["semantic_score"] = decision["confidence"]
        else:
            room["semantic_type"] = "unknown"
            room["semantic_score"] = decision["confidence"]
    graph.setdefault("sources", {})["qwen_room_semantics"] = str(args.output_decisions)
    graph["room_semantics"] = {
        "format": "pre_map_vln.room_semantics_qwen.v1",
        "provenance": provenance,
        "confidence_threshold": args.confidence_threshold,
        "decisions": decisions,
        "note": "Qwen labels existing OccuSG regions from their assigned Boxer inventory; it does not create geometry or coordinates.",
    }
    graph.setdefault("summary", {})["qwen_accepted_room_count"] = sum(
        item["accepted"] and item["room_type"] != "corridor" for item in decisions
    )
    args.output_graph.parent.mkdir(parents=True, exist_ok=True)
    args.output_graph.write_text(
        json.dumps(graph, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    args.output_decisions.parent.mkdir(parents=True, exist_ok=True)
    args.output_decisions.write_text(
        json.dumps(graph["room_semantics"], ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "output_graph": str(args.output_graph),
        "output_decisions": str(args.output_decisions),
        "room_count": len(rooms),
        "accepted_room_count": graph["summary"]["qwen_accepted_room_count"],
        "decisions": decisions,
        "provenance": provenance,
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
