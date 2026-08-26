"""Qwen-backed, auditable object policy for structural room maps."""

from __future__ import annotations

import csv
import hashlib
import json
import os
import re
import tempfile
import urllib.request
from pathlib import Path
from typing import Any

from stage2.qwen_http import open_json_with_retry


ROOT = Path(__file__).resolve().parents[1]
ENDPOINT = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
MODEL = "qwen3.7-plus"
PROMPT_VERSION = "stage1-structure-policy-v2"
INPUT_CNY_PER_MILLION = 2.0
OUTPUT_CNY_PER_MILLION = 8.0
ROLES = {"structural_boundary", "opening_boundary", "interior_object", "fixed_fixture", "uncertain"}


SYSTEM_PROMPT = """你是室内机器人结构地图分析器。根据一批三维检测框，判断每个实例在“房间结构分割地图”中的拓扑角色。
结构地图只用于识别房间边界，不用于避障；导航地图始终保留所有物体。
角色定义：
- structural_boundary：墙、柱、永久隔断等建筑结构；
- opening_boundary：门、门框等标识房间出入口的边界；
- interior_object：床、桌椅、沙发、灯、电视、普通架子等室内陈设；
- fixed_fixture：橱柜、浴缸、厨房台面等固定设施，但它们通常不是房间边界；
- uncertain：仅凭检测框无法安全判断。
房间分割时仅删除高置信度的 interior_object 和 fixed_fixture。不要因为物体体积大就把家具判断为建筑结构。
输出 confidence 表示“拓扑角色判断”的语义置信度，不是输入中的目标检测置信度；例如已经识别成 bed 时，对其属于 interior_object 通常应有很高的角色置信度。目标检测置信度只表示框本身的可靠程度。
必须为输入中的每个 instance_id 返回且只返回一次结果。只输出严格 JSON，不要 Markdown：
{"instances":[{"instance_id":"boxer_0","role":"interior_object","confidence":0.95,"reason":"简短原因"}]}"""


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False, encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        temporary = Path(handle.name)
    temporary.replace(path)


def read_boxes(path: Path) -> list[dict[str, Any]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    boxes = []
    for index, row in enumerate(rows):
        boxes.append({
            "instance_id": f"boxer_{index}",
            "label": str(row["name"]).strip().lower(),
            "center_xyz_m": [float(row[key]) for key in ("tx_world_object", "ty_world_object", "tz_world_object")],
            "size_xyz_m": [float(row[key]) for key in ("scale_x", "scale_y", "scale_z")],
            "detection_confidence": float(row["prob"]),
        })
    if not boxes:
        raise ValueError(f"no boxes found in {path}")
    return boxes


def _extract_json(text: str) -> dict[str, Any]:
    text = text.strip()
    fenced = re.fullmatch(r"```(?:json)?\s*(.*?)\s*```", text, re.DOTALL | re.IGNORECASE)
    return json.loads(fenced.group(1) if fenced else text)


def normalize_structure_policy(
    value: dict[str, Any], boxes: list[dict[str, Any]], minimum_remove_confidence: float = 0.65,
) -> dict[str, Any]:
    if not isinstance(value, dict) or not isinstance(value.get("instances"), list):
        raise ValueError("structure policy must contain an instances array")
    expected = {box["instance_id"]: box for box in boxes}
    decisions: dict[str, dict[str, Any]] = {}
    for raw in value["instances"]:
        if not isinstance(raw, dict):
            raise ValueError("each structure-policy decision must be an object")
        instance_id = str(raw.get("instance_id", ""))
        if instance_id not in expected:
            raise ValueError(f"unknown structure-policy instance: {instance_id}")
        if instance_id in decisions:
            raise ValueError(f"duplicate structure-policy instance: {instance_id}")
        role = str(raw.get("role", "")).strip().lower()
        if role not in ROLES:
            raise ValueError(f"invalid topology role for {instance_id}: {role}")
        confidence = float(raw.get("confidence", -1.0))
        if not 0.0 <= confidence <= 1.0:
            raise ValueError(f"invalid confidence for {instance_id}")
        reason = str(raw.get("reason", "")).strip()
        # Preserve a valid role/confidence decision if one explanation is
        # omitted in a large response, while making the omission auditable.
        if not reason:
            reason = "model returned no explanation; role retained for audit"
        reason = reason[:300]
        box = expected[instance_id]
        decisions[instance_id] = {
            **box,
            "role": role,
            "confidence": confidence,
            "remove_from_structure_map": bool(
                role in {"interior_object", "fixed_fixture"}
                and confidence >= minimum_remove_confidence
            ),
            "keep_in_navigation_map": True,
            "reason": reason,
        }
    for instance_id, box in expected.items():
        if instance_id not in decisions:
            decisions[instance_id] = {
                **box,
                "role": "uncertain",
                "confidence": 0.0,
                "remove_from_structure_map": False,
                "keep_in_navigation_map": True,
                "reason": "model omitted this instance; retained by the fail-safe policy",
            }
    return {
        "format": "pre_map_vln.structure_policy.v1",
        "minimum_remove_confidence": float(minimum_remove_confidence),
        "instances": [decisions[key] for key in sorted(decisions, key=lambda item: int(item.split("_")[-1]))],
    }


class QwenStructurePolicy:
    def __init__(self, api_key_path: Path | None = None, budget_cny: float = 20.0):
        self.api_key_path = api_key_path or ROOT / ".secrets/dashscope_api_key"
        self.budget_cny = float(budget_cny)
        self.cache_dir = ROOT / "outputs/stage1/structure_policy_cache"
        self.ledger_path = ROOT / "outputs/stage1/api_usage.json"

    def _ledger(self) -> dict[str, Any]:
        if self.ledger_path.exists():
            return json.loads(self.ledger_path.read_text(encoding="utf-8"))
        return {"format": "pre_map_vln.api_usage.v1", "total_estimated_cny": 0.0, "calls": []}

    def _cache_path(self, boxes: list[dict[str, Any]]) -> Path:
        payload = json.dumps(
            {"version": PROMPT_VERSION, "model": MODEL, "boxes": boxes},
            ensure_ascii=False, sort_keys=True,
        ).encode("utf-8")
        return self.cache_dir / f"{hashlib.sha256(payload).hexdigest()}.json"

    def classify(self, boxes: list[dict[str, Any]], use_cache: bool = True) -> dict[str, Any]:
        cache_path = self._cache_path(boxes)
        if use_cache and cache_path.exists():
            result = normalize_structure_policy(
                json.loads(cache_path.read_text(encoding="utf-8")), boxes
            )
            result["provenance"] = {
                **json.loads(cache_path.read_text(encoding="utf-8")).get("provenance", {}),
                "cache_hit": True,
            }
            return result

        ledger = self._ledger()
        previous = float(ledger.get("total_estimated_cny", 0.0))
        if previous >= self.budget_cny:
            raise RuntimeError(f"Qwen budget reached: {previous:.4f} CNY")
        api_key = self.api_key_path.read_text(encoding="utf-8").strip()
        payload = {
            "model": MODEL,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": "三维检测框：\n" + json.dumps(boxes, ensure_ascii=False)},
            ],
            "response_format": {"type": "json_object"},
            "enable_thinking": False,
            "temperature": 0.0,
            "max_tokens": 5000,
        }
        request = urllib.request.Request(
            ENDPOINT,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
            method="POST",
        )
        body, request_attempts = open_json_with_retry(request, timeout=180)
        usage = body.get("usage") or {}
        input_tokens = int(usage.get("prompt_tokens", 0))
        output_tokens = int(usage.get("completion_tokens", 0))
        cost = (input_tokens * INPUT_CNY_PER_MILLION + output_tokens * OUTPUT_CNY_PER_MILLION) / 1_000_000.0
        if previous + cost > self.budget_cny:
            raise RuntimeError("Qwen structure-policy response exceeded configured budget")
        ledger["authorized_budget_cny"] = self.budget_cny
        ledger["total_estimated_cny"] = round(previous + cost, 8)
        ledger["calls"].append({
            "model": MODEL, "purpose": "structure_object_classification",
            "prompt_tokens": input_tokens, "completion_tokens": output_tokens,
            "estimated_cny": round(cost, 8),
        })
        atomic_json(self.ledger_path, ledger)
        result = normalize_structure_policy(
            _extract_json(body["choices"][0]["message"]["content"]), boxes
        )
        result["provenance"] = {
            "classifier": "qwen_llm", "model": body.get("model", MODEL),
            "prompt_version": PROMPT_VERSION, "cache_hit": False,
            "usage": usage, "estimated_cny": round(cost, 8),
            "request_attempts": request_attempts,
        }
        atomic_json(cache_path, result)
        return result
