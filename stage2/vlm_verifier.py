"""Qwen3-VL task-terminal image verification with strict JSON and cost audit."""

from __future__ import annotations

import base64
import hashlib
import json
import os
import re
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

from .io_utils import atomic_json, load_json


ROOT = Path(__file__).resolve().parents[1]
ENDPOINT = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
MODEL = "qwen3-vl-plus"
PROMPT_VERSION = "stage4-terminal-verifier-v1"
INPUT_CNY_PER_MILLION = 1.0
OUTPUT_CNY_PER_MILLION = 10.0


SYSTEM_PROMPT = """You verify whether an indoor robot has visually completed an observation task.
Use only visible RGB evidence. Do not infer that an object exists merely from the task text.
Return one strict JSON object with no Markdown:
{
  "found": true,
  "confidence": 0.0,
  "evidence": "short visible evidence",
  "visible_objects": ["object"],
  "target_bbox_0_1000": [x1,y1,x2,y2] or null
}
found is true only when the requested verification target is visibly identifiable. Coordinates are normalized to 0..1000."""


def _extract_json(text: str) -> dict[str, Any]:
    text = text.strip()
    fenced = re.fullmatch(r"```(?:json)?\s*(.*?)\s*```", text, re.DOTALL | re.IGNORECASE)
    return json.loads(fenced.group(1) if fenced else text)


def normalize_verification(value: dict[str, Any], target_label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError("VLM verification must be a JSON object")
    if not isinstance(value.get("found"), bool):
        raise ValueError("VLM verification found must be boolean")
    confidence = float(value.get("confidence", -1.0))
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("VLM verification confidence must be in [0,1]")
    evidence = str(value.get("evidence", "")).strip()
    if not evidence or len(evidence) > 500:
        raise ValueError("VLM verification evidence is empty or too long")
    visible = value.get("visible_objects") or []
    if not isinstance(visible, list) or not all(isinstance(item, str) for item in visible):
        raise ValueError("visible_objects must be a string array")
    bbox = value.get("target_bbox_0_1000")
    if bbox is not None:
        if not isinstance(bbox, list) or len(bbox) != 4:
            raise ValueError("target_bbox_0_1000 must contain four values")
        bbox = [float(item) for item in bbox]
        if not all(0.0 <= item <= 1000.0 for item in bbox) or bbox[0] >= bbox[2] or bbox[1] >= bbox[3]:
            raise ValueError("target bbox is invalid")
    return {
        "target_label": target_label.strip().lower(),
        "found": value["found"],
        "confidence": confidence,
        "evidence": evidence,
        "visible_objects": sorted(set(item.strip().lower() for item in visible if item.strip())),
        "target_bbox_0_1000": bbox,
    }


class QwenImageVerifier:
    def __init__(
        self,
        api_key_path: Path | None = None,
        budget_cny: float | None = None,
        max_single_call_cny: float = 5.0,
    ):
        self.api_key_path = api_key_path or ROOT / ".secrets/dashscope_api_key"
        self.budget_cny = float(
            budget_cny if budget_cny is not None else os.environ.get("PRE_MAP_VLN_QWEN_BUDGET_CNY", "1000")
        )
        self.max_single_call_cny = float(max_single_call_cny)
        self.cache_dir = ROOT / "outputs/stage2/vlm_image_cache"
        self.ledger_path = ROOT / "outputs/stage2/api_usage.json"

    def _ledger(self) -> dict[str, Any]:
        if self.ledger_path.exists():
            return load_json(self.ledger_path)
        return {"format": "pre_map_vln.api_usage.v1", "total_estimated_cny": 0.0, "calls": []}

    def _cache_path(self, image_paths: list[Path], task: dict[str, Any]) -> Path:
        digest = hashlib.sha256()
        digest.update(PROMPT_VERSION.encode())
        digest.update(MODEL.encode())
        digest.update(json.dumps({
            "task_id": task["id"], "action": task["action"],
            "target": task["target"], "verification_label": task["verification_label"],
        }, ensure_ascii=False, sort_keys=True).encode())
        for path in image_paths:
            digest.update(path.read_bytes())
        return self.cache_dir / f"{digest.hexdigest()}.json"

    def verify(self, image_paths: list[Path], task: dict[str, Any], use_cache: bool = True) -> dict[str, Any]:
        if not image_paths or len(image_paths) > 4:
            raise ValueError("provide one to four RGB evidence frames")
        image_paths = [Path(path) for path in image_paths]
        cache_path = self._cache_path(image_paths, task)
        if use_cache and cache_path.exists():
            cached = load_json(cache_path)
            result = normalize_verification(cached, task["verification_label"])
            result["cache_hit"] = True
            return result

        ledger = self._ledger()
        ledger["authorized_budget_cny"] = self.budget_cny
        previous = float(ledger.get("total_estimated_cny", 0.0))
        if previous >= self.budget_cny:
            raise RuntimeError(f"Qwen budget reached: {previous:.4f} CNY")
        api_key = self.api_key_path.read_text(encoding="utf-8").strip()
        content = []
        for path in image_paths:
            suffix = path.suffix.lower()
            mime = "image/png" if suffix == ".png" else "image/jpeg"
            encoded = base64.b64encode(path.read_bytes()).decode("ascii")
            content.append({"type": "image_url", "image_url": {"url": f"data:{mime};base64,{encoded}"}})
        content.append({
            "type": "text",
            "text": (
                f"Task id: {task['id']}\nAction: {task['action']}\n"
                f"Navigation anchor: {task['target']['label']}\n"
                f"Verification target: {task['verification_label']}\n"
                "Decide whether the verification target is visibly present in the supplied frame(s)."
            ),
        })
        payload = {
            "model": MODEL,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": content},
            ],
            "response_format": {"type": "json_object"},
            "enable_thinking": False,
            "temperature": 0.0,
            "max_tokens": 500,
        }
        request = urllib.request.Request(
            ENDPOINT,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
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
        estimated_cost = (
            input_tokens * INPUT_CNY_PER_MILLION + output_tokens * OUTPUT_CNY_PER_MILLION
        ) / 1_000_000.0
        if estimated_cost > self.max_single_call_cny:
            raise RuntimeError(f"Qwen single call exceeded safety guard: {estimated_cost:.4f} CNY")
        if previous + estimated_cost > self.budget_cny:
            raise RuntimeError("Qwen response would exceed configured budget")
        ledger["calls"].append({
            "model": MODEL, "purpose": "terminal_rgb_verification",
            "prompt_tokens": input_tokens, "completion_tokens": output_tokens,
            "estimated_cny": round(estimated_cost, 8),
        })
        ledger["total_estimated_cny"] = round(previous + estimated_cost, 8)
        atomic_json(self.ledger_path, ledger)
        result = normalize_verification(
            _extract_json(body["choices"][0]["message"]["content"]), task["verification_label"]
        )
        result.update({
            "model": body.get("model", MODEL), "prompt_version": PROMPT_VERSION,
            "usage": usage, "estimated_cny": round(estimated_cost, 8), "cache_hit": False,
        })
        atomic_json(cache_path, result)
        return result
