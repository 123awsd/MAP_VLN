"""Qwen3.7-Plus backed long-instruction parser with caching and cost guard."""

from __future__ import annotations

import hashlib
import json
import os
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
PROMPT_VERSION = "stage2-task-graph-v10-semantic-observation-region"
INPUT_CNY_PER_MILLION = 2.0
OUTPUT_CNY_PER_MILLION = 8.0


SYSTEM_PROMPT = """你是室内无人机长时程任务规划器。把中文长指令转换为严格 JSON，不要输出解释或 Markdown。
只保留语义上真正必要的先后关系，不要把原句出现顺序当作约束。严格解释连接词：
- “先A，再B”只建立 A→B；
- “顺便C”“还有D”“同时E”默认是独立任务，不得依赖前一句，也不得互相串联；
- “最后F”表示 F 依赖此前所有需要执行的非条件任务；
- 条件备用任务必须单独建 task，并只通过 conditional_rules 激活，不要为它添加会阻止激活的 prerequisite。
每个任务必须有可观测的目标物体。空间关系 relation 只能是 front/behind/left/right/above/below/on/between/near/facing 或 null。
输出结构：
{
  "format":"pre_map_vln.task_graph.v1",
  "instruction":"原指令",
  "tasks":[{
    "id":"英文snake_case唯一ID",
    "action":"inspect|find|observe|deliver|approach",
    "target":{"label":"与场景物体标签尽量一致","room":"房间或null","floor_id":"楼层整数或null","reference":"参照物或null","reference_secondary":"第二参照物或null","references":[]},
    "verification_label":"最终需要在RGB中确认的物体类别",
    "intent":{"goal_type":"verify_presence|locate_target|execute_action","not_found_policy":"report_absent|semantic_recovery|explicit_branch"},
    "spatial_constraints":{"relation":null,"distance_m":null,"height_m":null,"height_range_m":null,"region_type":"auto","observation_detail":"normal","vertical_fov_deg":70.0,"horizontal_fov_deg":90.0,"yaw_tolerance_deg":55.0,"face_target":true,"visibility_required":true},
    "prerequisites":[],"active_initially":true,"success_outcome":"found|done",
    "search_policy":{"mode":"fixed|semantic_recovery","on_exhaustion":"finish|qwen_semantic_recovery","maximum_location_hypotheses":3}
  }],
  "conditional_rules":[{"source_task_id":"...","if_outcome":"not_found","activate_task_ids":["..."],"skip_task_ids":[]}]
}
deliver 任务的 target 是交付终点物体，reference 可写被运送物体。只要待确认物体在用户指定的位置范围内已有历史实例，target.label 必须是待确认物体本身；例如“台面上的微波炉”应写 target.label=microwave、verification_label=microwave、reference=countertop、relation=on。reference 描述目标与参照物的语义关系，不是让无人机站到参照物上。若待确认物体虽在全局历史清单里出现、但不在用户明确指定的楼层/房间/锚点范围内，则初始 target 必须使用用户指定位置中真实存在的大型锚点，verification_label 写真正要找的类别；例如“一楼洗手台旁找卫生纸”而一楼没有卫生纸历史实例时，target.label=vanity、floor_id=1、verification_label=toilet paper。检查“床边的灯”且该处已有灯时，target.label 和 verification_label 都是 lamp，reference 是 bed。
指令明确指定楼层时填写 floor_id；未指定时必须为 null，不能猜楼层。relation 表示目标相对 reference 的搜索区域语义，用于生成观察位姿，不作为检测目标后的硬成功条件；使用大型锚点代替未知目标时，不要再把锚点写成自己的 reference。between 必须提供两个参照物。region_type 不确定时使用 auto。distance_m 默认必须为 null，只有用户明确给出观察距离或距离范围时才填写；不要自行输出通用默认距离。observation_detail 只表达普通或精细观察，不直接决定坐标。
普通开放式“找一下某物”且没有指定位置时，使用 mode=semantic_recovery、on_exhaustion=qwen_semantic_recovery。明确指定位置时必须 mode=fixed：如果语义是必须找到的可移动/可消耗物体，且指定位置没有该目标的历史实例，则 on_exhaustion=qwen_semantic_recovery；普通检查或确认任务使用 on_exhaustion=finish。用户显式 if/如果 分支只能写 conditional_rules，不能改写成自主恢复。
必须显式判断用户意图：“看看/确认某处有没有”是 verify_presence + report_absent，没看到本身就是有效答案；“帮我找/一定要找到，应该在某处，没有就继续找”是 locate_target + semantic_recovery；用户明确说“如果没有就去某处”是 locate_target + explicit_branch，并建立 conditional_rules。execute_action 只用于 deliver/approach 等非视觉动作。intent.not_found_policy 必须与 search_policy.on_exhaustion 一致：semantic_recovery 对应 qwen_semantic_recovery，其余对应 finish。
输出前必须自检：若原句含“最后”，该任务的 prerequisites 是否包含它之前所有 active_initially=true 的任务；若任务指定了 floor_id、room 或 reference，search_policy.mode 是否为 fixed；任务目的和未找到处理是否符合用户原意；是否把自主恢复和用户显式条件分支严格分开；若原句含“顺便/还有”，这些任务之间是否没有被错误串联。"""


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
            "floor_id": room.get("floor_id"),
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
    def __init__(self, api_key_path: Path | None = None, budget_cny: float | None = None):
        self.api_key_path = api_key_path or ROOT / ".secrets/dashscope_api_key"
        self.budget_cny = float(
            budget_cny if budget_cny is not None else os.environ.get("PRE_MAP_VLN_QWEN_BUDGET_CNY", "1000")
        )
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
        ledger["authorized_budget_cny"] = self.budget_cny
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
            "purpose": "task_graph_parsing",
            "prompt_tokens": input_tokens,
            "completion_tokens": output_tokens,
            "estimated_cny": round(estimated_cost, 8),
        })
        ledger["total_estimated_cny"] = round(previous + estimated_cost, 8)
        atomic_json(self.ledger_path, ledger)
        graph = normalize_and_validate_task_graph(_extract_json(content), instruction=instruction)
        graph["provenance"] = {
            "parser": "qwen_vlm",
            "model": body.get("model", MODEL),
            "prompt_version": PROMPT_VERSION,
            "cache_hit": False,
            "usage": usage,
            "estimated_cny": round(estimated_cost, 8),
        }
        graph["parser_raw_response"] = content
        atomic_json(cache_path, graph)
        return graph

    def repair(
        self, instruction: str, scene_graph: dict[str, Any], graph: dict[str, Any],
        validation_error: str,
    ) -> dict[str, Any]:
        """Ask Qwen to repair only a rejected task graph, preserving valid semantics."""
        ledger = self._ledger()
        previous = float(ledger.get("total_estimated_cny", 0.0))
        if previous >= self.budget_cny:
            raise RuntimeError("Qwen budget reached before task-graph repair")
        payload = {
            "model": MODEL,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": json.dumps({
                    "instruction": instruction,
                    "scene_inventory": compact_inventory(scene_graph),
                    "rejected_task_graph": graph,
                    "deterministic_validation_error": validation_error,
                    "request": "只修复审计指出的问题，保留其他正确任务、关系、前置约束和条件分支；输出完整严格JSON。",
                }, ensure_ascii=False)},
            ],
            "response_format": {"type": "json_object"},
            "enable_thinking": False, "temperature": 0.0, "max_tokens": 3000,
        }
        request = urllib.request.Request(
            ENDPOINT, data=json.dumps(payload).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self.api_key_path.read_text(encoding='utf-8').strip()}",
                "Content-Type": "application/json",
            }, method="POST",
        )
        with urllib.request.urlopen(request, timeout=120) as response:
            body = json.load(response)
        usage = body.get("usage") or {}
        estimated_cost = (
            int(usage.get("prompt_tokens", 0)) * INPUT_CNY_PER_MILLION
            + int(usage.get("completion_tokens", 0)) * OUTPUT_CNY_PER_MILLION
        ) / 1_000_000.0
        if previous + estimated_cost > self.budget_cny:
            raise RuntimeError("Qwen repair response would exceed the authorized cost ceiling")
        ledger.setdefault("calls", []).append({
            "model": MODEL, "purpose": "task_graph_repair",
            "prompt_tokens": int(usage.get("prompt_tokens", 0)),
            "completion_tokens": int(usage.get("completion_tokens", 0)),
            "estimated_cny": round(estimated_cost, 8),
        })
        ledger["total_estimated_cny"] = round(previous + estimated_cost, 8)
        atomic_json(self.ledger_path, ledger)
        content = body["choices"][0]["message"]["content"]
        repaired = normalize_and_validate_task_graph(_extract_json(content), instruction=instruction)
        repaired["provenance"] = {
            "parser": "qwen_vlm", "model": body.get("model", MODEL),
            "prompt_version": PROMPT_VERSION, "cache_hit": False,
            "repair_round": 1, "validation_error": validation_error,
            "usage": usage, "estimated_cny": round(estimated_cost, 8),
        }
        repaired["parser_raw_response"] = content
        return repaired
