"""VLM-ranked, geometry-validated recovery search over a global scene graph."""

from __future__ import annotations

import copy
import hashlib
import json
import os
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

from .candidate_poses import generate_candidates
from .io_utils import atomic_json, load_json


ROOT = Path(__file__).resolve().parents[1]
ENDPOINT = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
MODEL = "qwen3.7-plus"
PROMPT_VERSION = "semantic-recovery-v2"
INPUT_CNY_PER_MILLION = 2.0
OUTPUT_CNY_PER_MILLION = 8.0
RANK_PENALTY = {"high": 0.0, "medium": 0.6, "low": 1.2}

SYSTEM_PROMPT = """你是家庭机器人目标恢复搜索规划器。目标在预期位置经过充分观察后未找到。
根据给定的全局房间和物体清单，提出最多3个尚未失败的搜索位置；只要清单中还有可用锚点，就应尽量给满3个。同类房间不足时，可按目标常见支撑物、收纳物和相邻功能空间扩展。只输出严格JSON，不要Markdown。
每个假设必须引用清单中真实存在的 room_id 和 anchor_object_id。优先选择与目标类别、房间用途、支撑物和日常放置习惯语义相关的位置；不要生成坐标，不要虚构房间或物体。
relevance只能是high/medium/low。输出：
{"target_label":"...","hypotheses":[{"room_id":1,"anchor_object_id":"boxer_1","relation":"near|on|inside|around","relevance":"high|medium|low","reason":"简短中文理由"}]}"""


def recovery_inventory(scene_graph: dict[str, Any]) -> dict[str, Any]:
    return {
        "rooms": [
            {
                "room_id": room["id"],
                "room_type": room.get("semantic_type", "unknown"),
                "space_role": room.get("space_role", "room"),
                "objects": [
                    {"id": obj["id"], "label": obj["label"]}
                    for obj in room.get("objects", [])
                ],
            }
            for room in scene_graph.get("rooms", [])
            if room.get("space_role") not in {"transition_space", "room_fragment"}
        ]
    }


def validate_hypotheses(
    value: dict[str, Any], scene_graph: dict[str, Any], failed_object_ids: set[str], maximum: int,
) -> dict[str, Any]:
    objects = {
        obj["id"]: (int(room["id"]), obj)
        for room in scene_graph.get("rooms", []) for obj in room.get("objects", [])
    }
    result = []
    seen = set()
    for source in value.get("hypotheses", []):
        object_id = str(source.get("anchor_object_id", ""))
        if object_id in seen or object_id in failed_object_ids or object_id not in objects:
            continue
        actual_room, obj = objects[object_id]
        if int(source.get("room_id", actual_room)) != actual_room:
            continue
        relevance = str(source.get("relevance", "medium")).lower()
        if relevance not in RANK_PENALTY:
            continue
        result.append({
            "id": f"recovery_{len(result) + 1}",
            "room_id": actual_room,
            "anchor_object_id": object_id,
            "anchor_label": obj["label"],
            "relation": str(source.get("relation", "near")).lower(),
            "relevance": relevance,
            "reason": str(source.get("reason", ""))[:240],
        })
        seen.add(object_id)
        if len(result) >= maximum:
            break
    return {"target_label": str(value.get("target_label", "")).lower(), "hypotheses": result}


class QwenSemanticRecoveryPlanner:
    def __init__(self, budget_cny: float = 20.0, api_key_path: Path | None = None):
        self.budget_cny = float(budget_cny)
        self.api_key_path = api_key_path or ROOT / ".secrets/dashscope_api_key"
        self.cache_dir = ROOT / "outputs/stage2/recovery_cache"
        self.ledger_path = ROOT / "outputs/stage2/api_usage.json"

    def _cache_path(self, target_label: str, failed: set[str], inventory: dict[str, Any]) -> Path:
        data = json.dumps({
            "version": PROMPT_VERSION, "target": target_label,
            "failed": sorted(failed), "inventory": inventory,
        }, sort_keys=True, ensure_ascii=False).encode()
        return self.cache_dir / f"{hashlib.sha256(data).hexdigest()}.json"

    def plan(
        self, target_label: str, instruction: str, scene_graph: dict[str, Any],
        failed_object_ids: set[str], maximum_hypotheses: int = 3, use_cache: bool = True,
    ) -> dict[str, Any]:
        inventory = recovery_inventory(scene_graph)
        cache_path = self._cache_path(target_label, failed_object_ids, inventory)
        if use_cache and cache_path.exists():
            result = validate_hypotheses(load_json(cache_path), scene_graph, failed_object_ids, maximum_hypotheses)
            result["provenance"] = {"planner": "qwen_vlm", "model": MODEL, "cache_hit": True}
            return result

        ledger = load_json(self.ledger_path) if self.ledger_path.exists() else {
            "format": "pre_map_vln.api_usage.v1", "total_estimated_cny": 0.0, "calls": []
        }
        previous = float(ledger.get("total_estimated_cny", 0.0))
        if previous >= self.budget_cny:
            raise RuntimeError("Qwen recovery-search budget reached")
        prompt = {
            "original_instruction": instruction,
            "missing_target": target_label,
            "failed_object_ids": sorted(failed_object_ids),
            "global_scene_inventory": inventory,
        }
        payload = {
            "model": MODEL,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": json.dumps(prompt, ensure_ascii=False)},
            ],
            "response_format": {"type": "json_object"},
            "enable_thinking": False,
            "temperature": 0.0,
            "max_tokens": 1200,
        }
        request = urllib.request.Request(
            ENDPOINT, data=json.dumps(payload).encode(), method="POST",
            headers={
                "Authorization": f"Bearer {self.api_key_path.read_text().strip()}",
                "Content-Type": "application/json",
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=120) as response:
                body = json.load(response)
        except urllib.error.HTTPError as error:
            detail = error.read().decode("utf-8", "replace")[:1000]
            raise RuntimeError(f"DashScope HTTP {error.code}: {detail}") from error
        raw = json.loads(body["choices"][0]["message"]["content"])
        result = validate_hypotheses(raw, scene_graph, failed_object_ids, maximum_hypotheses)
        usage = body.get("usage") or {}
        cost = (
            int(usage.get("prompt_tokens", 0)) * INPUT_CNY_PER_MILLION
            + int(usage.get("completion_tokens", 0)) * OUTPUT_CNY_PER_MILLION
        ) / 1_000_000.0
        if previous + cost > self.budget_cny:
            raise RuntimeError("Qwen recovery-search response exceeded budget")
        ledger["authorized_budget_cny"] = self.budget_cny
        ledger["total_estimated_cny"] = round(previous + cost, 8)
        ledger["calls"].append({
            "model": MODEL, "purpose": "semantic_recovery_search",
            "prompt_tokens": int(usage.get("prompt_tokens", 0)),
            "completion_tokens": int(usage.get("completion_tokens", 0)),
            "estimated_cny": round(cost, 8),
        })
        atomic_json(self.ledger_path, ledger)
        atomic_json(cache_path, result)
        result["provenance"] = {
            "planner": "qwen_vlm", "model": body.get("model", MODEL),
            "prompt_version": PROMPT_VERSION, "cache_hit": False,
            "estimated_cny": round(cost, 8),
        }
        return result


def materialize_recovery_candidates(
    grid, scene_graph: dict[str, Any], task: dict[str, Any], search_plan: dict[str, Any],
    max_candidates_per_hypothesis: int = 4,
) -> list[dict[str, Any]]:
    objects = {
        obj["id"]: obj
        for room in scene_graph.get("rooms", []) for obj in room.get("objects", [])
    }
    values = []
    for hypothesis in search_plan.get("hypotheses", []):
        object_id = hypothesis["anchor_object_id"]
        obj = objects.get(object_id)
        if obj is None:
            continue
        branch_task = copy.deepcopy(task)
        branch_task["target"] = {
            "label": str(obj["label"]).lower(), "room": None, "reference": None,
        }
        branch_task["spatial_constraints"]["relation"] = None
        generated = generate_candidates(
            grid, scene_graph, branch_task, max_candidates=max_candidates_per_hypothesis,
            allowed_object_ids={object_id}, prefer_room=False,
        )
        for candidate in generated:
            candidate["terminal_cost"] += RANK_PENALTY[hypothesis["relevance"]]
            candidate["recovery_search"] = copy.deepcopy(hypothesis)
            candidate["location_hypothesis_id"] = hypothesis["id"]
            candidate["candidate_source"] = "vlm_semantic_recovery"
        values.extend(generated)
    return values
