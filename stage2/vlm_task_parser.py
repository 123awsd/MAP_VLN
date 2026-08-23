"""Qwen3.7-Plus backed long-instruction parser with caching and cost guard."""

from __future__ import annotations

import hashlib
import json
import re
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

from .io_utils import atomic_json, load_json
from .task_graph import normalize_and_validate_task_graph


ROOT = Path(__file__).resolve().parents[1]
ENDPOINT = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
MODEL = "qwen3.7-plus"
PROMPT_VERSION = "stage2-task-graph-v1"
INPUT_CNY_PER_MILLION = 2.0
OUTPUT_CNY_PER_MILLION = 8.0


SYSTEM_PROMPT = """你是室内无人机长时程任务规划器。把中文长指令转换为严格 JSON，不要输出解释或 Markdown。
只保留语义上真正必要的先后关系，不要把原句出现顺序当作约束。条件任务必须单独建 task，并通过 conditional_rules 激活。
每个任务必须有可观测的目标物体。空间关系 relation 只能是 front/behind/left/right/above/below/between/near/facing 或 null。
输出结构：
{
  "format":"pre_map_vln.task_graph.v1",
  "instruction":"原指令",
  "tasks":[{
    "id":"英文snake_case唯一ID",
    "action":"inspect|find|observe|deliver|approach",
    "target":{"label":"与场景物体标签尽量一致","room":"房间或null","reference":"参照物或null"},
    "spatial_constraints":{"relation":null,"distance_m":[0.8,1.8],"height_m":null,"face_target":true,"visibility_required":true},
    "prerequisites":[],"active_initially":true,"success_outcome":"found|done"
  }],
  "conditional_rules":[{"source_task_id":"...","if_outcome":"not_found","activate_task_ids":["..."],"skip_task_ids":[]}]
}
deliver 任务的 target 是交付终点物体，reference 可写被运送物体。检查“有没有”使用 inspect，找到目标后的 outcome 为 found。"""


def compact_inventory(scene_graph: dict[str, Any]) -> dict[str, Any]:
    rooms = []
    labels: dict[str, int] = {}
    for room in scene_graph.get("rooms", []):
        room_labels = []
        for obj in room.get("objects", []):
            label = str(obj.get("label", "")).lower()
            if label:
                labels[label] = labels.get(label, 0) + 1
                room_labels.append(label)
        rooms.append({
            "id": room.get("id"),
            "semantic_type": room.get("semantic_type", "unknown"),
            "object_labels": sorted(set(room_labels)),
        })
    return {"object_label_counts": labels, "rooms": rooms}


def _extract_json(text: str) -> dict[str, Any]:
    text = text.strip()
    fenced = re.fullmatch(r"```(?:json)?\s*(.*?)\s*```", text, re.DOTALL | re.IGNORECASE)
    if fenced:
        text = fenced.group(1)
    return json.loads(text)


class QwenTaskParser:
    def __init__(self, api_key_path: Path | None = None, budget_cny: float = 20.0):
        self.api_key_path = api_key_path or ROOT / ".secrets/dashscope_api_key"
        self.budget_cny = float(budget_cny)
        self.cache_dir = ROOT / "outputs/stage2/vlm_cache"
        self.ledger_path = ROOT / "outputs/stage2/api_usage.json"

    def _ledger(self) -> dict[str, Any]:
        if self.ledger_path.exists():
            return load_json(self.ledger_path)
        return {"format": "pre_map_vln.api_usage.v1", "total_estimated_cny": 0.0, "calls": []}

    def _cache_path(self, instruction: str, inventory: dict[str, Any]) -> Path:
        encoded = json.dumps(
            {"version": PROMPT_VERSION, "model": MODEL, "instruction": instruction, "inventory": inventory},
            ensure_ascii=False,
            sort_keys=True,
        ).encode("utf-8")
        return self.cache_dir / f"{hashlib.sha256(encoded).hexdigest()}.json"

    def parse(self, instruction: str, scene_graph: dict[str, Any], use_cache: bool = True) -> dict[str, Any]:
        inventory = compact_inventory(scene_graph)
        cache_path = self._cache_path(instruction, inventory)
        if use_cache and cache_path.exists():
            cached = normalize_and_validate_task_graph(load_json(cache_path), instruction=instruction)
            cached.setdefault("provenance", {})
            cached["provenance"]["cache_hit"] = True
            return cached

        ledger = self._ledger()
        if float(ledger.get("total_estimated_cny", 0.0)) >= self.budget_cny:
            raise RuntimeError(f"Qwen budget reached: {ledger['total_estimated_cny']:.4f} CNY")
        api_key = self.api_key_path.read_text(encoding="utf-8").strip()
        user_prompt = "场景物体清单：\n" + json.dumps(inventory, ensure_ascii=False) + "\n\n长指令：\n" + instruction
        payload = {
            "model": MODEL,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt},
            ],
            "response_format": {"type": "json_object"},
            "enable_thinking": False,
            "temperature": 0.0,
            "max_tokens": 3000,
        }
        request = urllib.request.Request(
            ENDPOINT,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=120) as response:
                body = json.load(response)
        except urllib.error.HTTPError as error:
            detail = error.read().decode("utf-8", "replace")[:1000]
            raise RuntimeError(f"DashScope HTTP {error.code}: {detail}") from error
        content = body["choices"][0]["message"]["content"]
        graph = normalize_and_validate_task_graph(_extract_json(content), instruction=instruction)
        usage = body.get("usage") or {}
        input_tokens = int(usage.get("prompt_tokens", 0))
        output_tokens = int(usage.get("completion_tokens", 0))
        estimated_cost = (
            input_tokens * INPUT_CNY_PER_MILLION + output_tokens * OUTPUT_CNY_PER_MILLION
        ) / 1_000_000.0
        previous = float(ledger.get("total_estimated_cny", 0.0))
        if previous + estimated_cost > self.budget_cny:
            raise RuntimeError("Qwen response would exceed the authorized cost ceiling")
        ledger["calls"].append({
            "model": MODEL,
            "prompt_tokens": input_tokens,
            "completion_tokens": output_tokens,
            "estimated_cny": round(estimated_cost, 8),
        })
        ledger["total_estimated_cny"] = round(previous + estimated_cost, 8)
        atomic_json(self.ledger_path, ledger)
        graph["provenance"] = {
            "parser": "qwen_vlm",
            "model": body.get("model", MODEL),
            "prompt_version": PROMPT_VERSION,
            "cache_hit": False,
            "usage": usage,
            "estimated_cny": round(estimated_cost, 8),
        }
        atomic_json(cache_path, graph)
        return graph
