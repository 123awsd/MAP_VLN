"""Evaluate a natural-language inspection mission from ordered RGB evidence images."""

from __future__ import annotations

import base64
import json
import re
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
ENDPOINT = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
MODEL = "qwen3-vl-plus"
PROMPT_VERSION = "real-flight-mission-image-evaluator-v1"


SYSTEM_PROMPT = """你是室内无人机巡检任务的视觉验收员。
你会收到一条自然语言任务指令和若干张按主要巡检步骤顺序排列的任务时刻 RGB 图片。

你的职责：
1. 从原始指令中提取所有主要检查步骤、顺序和条件分支，不得把任务改写成另一项任务。
2. 只根据图片中可见证据回答每个检查问题；不得因为指令提到了某物，就假定它存在。
3. 区分“检查动作是否完成”和“检查结果”：
   - completion_status=completed：图片足以回答该检查问题，无论答案是存在还是不存在；
   - completion_status=not_completed：应有对应证据，但图片明显没有覆盖目标区域；
   - completion_status=uncertain：图片相关，但画面不足以可靠判断目标状态。
4. observation_result 使用 present、absent、on、off、not_empty、empty、uncertain 或简短的通用结果。
5. 对“如果没有则去别处寻找”之类条件分支：仅当条件由可见证据触发时才要求分支证据；未触发时标记 skipped，并说明原因。
6. 不得把检测框文字或文件名当作唯一证据；必须检查图像内容本身。
7. 对开关、液位、是否喝完等细粒度状态，如果画面无法支持，就输出 uncertain，不要猜测。
8. overall_reply 必须使用原指令的主要语言，像执行完巡检后向用户做简洁、自然、诚实的汇报。

只返回一个严格 JSON 对象，不要 Markdown，不要额外文字：
{
  "instruction_summary": "简短任务概述",
  "tasks": [
    {
      "task_index": 1,
      "request": "该检查步骤",
      "image_indices": [1],
      "completion_status": "completed|not_completed|uncertain|skipped",
      "observation_result": "present|absent|on|off|not_empty|empty|uncertain|其他简短结果",
      "confidence": 0.0,
      "evidence": "图片中的简短可见证据"
    }
  ],
  "overall_status": "completed|partially_completed|not_completed|uncertain",
  "overall_reply": "面向用户的自然语言汇报",
  "limitations": ["无法从图片确认的事项"]
}"""


def build_user_prompt(instruction: str, image_count: int) -> str:
    return (
        f"原始任务指令：\n{instruction.strip()}\n\n"
        f"证据图片数量：{image_count}。图片按原指令中的主要巡检步骤顺序排列，"
        "但路线优化可能使实际拍摄时间顺序不同；请依据此处的输入顺序建立对应关系。"
        "如果任务含条件分支，不要凭空要求未触发分支的图片。"
    )


def _extract_json(text: str) -> dict[str, Any]:
    text = text.strip()
    fenced = re.fullmatch(r"```(?:json)?\s*(.*?)\s*```", text, re.DOTALL | re.IGNORECASE)
    value = json.loads(fenced.group(1) if fenced else text)
    if not isinstance(value, dict):
        raise ValueError("Qwen mission evaluation must be a JSON object")
    return value


def normalize_mission_evaluation(value: dict[str, Any], image_count: int) -> dict[str, Any]:
    tasks = value.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        raise ValueError("mission evaluation tasks must be a non-empty array")
    valid_completion = {"completed", "not_completed", "uncertain", "skipped"}
    normalized_tasks = []
    for position, task in enumerate(tasks, 1):
        if not isinstance(task, dict):
            raise ValueError(f"task {position} must be an object")
        status = str(task.get("completion_status", "")).strip()
        if status not in valid_completion:
            raise ValueError(f"task {position} has invalid completion_status")
        indices = task.get("image_indices") or []
        if not isinstance(indices, list) or not all(isinstance(item, int) for item in indices):
            raise ValueError(f"task {position} image_indices must be integers")
        if any(item < 1 or item > image_count for item in indices):
            raise ValueError(f"task {position} references an unavailable image")
        confidence = float(task.get("confidence", -1.0))
        if not 0.0 <= confidence <= 1.0:
            raise ValueError(f"task {position} confidence must be in [0,1]")
        request = str(task.get("request", "")).strip()
        evidence = str(task.get("evidence", "")).strip()
        result = str(task.get("observation_result", "")).strip()
        if not request or not evidence or not result:
            raise ValueError(f"task {position} is missing request, evidence, or result")
        normalized_tasks.append({
            "task_index": int(task.get("task_index", position)),
            "request": request,
            "image_indices": indices,
            "completion_status": status,
            "observation_result": result,
            "confidence": confidence,
            "evidence": evidence,
        })
    overall_status = str(value.get("overall_status", "")).strip()
    if overall_status not in {"completed", "partially_completed", "not_completed", "uncertain"}:
        raise ValueError("invalid overall_status")
    limitations = value.get("limitations") or []
    if not isinstance(limitations, list) or not all(isinstance(item, str) for item in limitations):
        raise ValueError("limitations must be a string array")
    return {
        "instruction_summary": str(value.get("instruction_summary", "")).strip(),
        "tasks": normalized_tasks,
        "overall_status": overall_status,
        "overall_reply": str(value.get("overall_reply", "")).strip(),
        "limitations": [item.strip() for item in limitations if item.strip()],
    }


def evaluate_mission_images(
    instruction: str,
    image_paths: list[Path],
    api_key_path: Path | None = None,
    timeout_s: int = 180,
) -> dict[str, Any]:
    if not instruction.strip():
        raise ValueError("instruction must not be empty")
    if not image_paths or len(image_paths) > 12:
        raise ValueError("provide one to twelve ordered RGB evidence images")
    paths = [Path(path).resolve() for path in image_paths]
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(path)
    key_path = api_key_path or ROOT / ".secrets/dashscope_api_key"
    api_key = key_path.read_text(encoding="utf-8").strip()
    if not api_key:
        raise RuntimeError(f"empty DashScope API key: {key_path}")

    content: list[dict[str, Any]] = [{"type": "text", "text": build_user_prompt(instruction, len(paths))}]
    for index, path in enumerate(paths, 1):
        mime = "image/png" if path.suffix.lower() == ".png" else "image/jpeg"
        content.append({"type": "text", "text": f"证据图片 {index}："})
        content.append({
            "type": "image_url",
            "image_url": {"url": f"data:{mime};base64,{base64.b64encode(path.read_bytes()).decode('ascii')}"},
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
        "max_tokens": 1800,
    }
    request = urllib.request.Request(
        ENDPOINT,
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            body = json.load(response)
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", "replace")[:1500]
        raise RuntimeError(f"DashScope HTTP {error.code}: {detail}") from error
    raw_text = body["choices"][0]["message"]["content"]
    result = normalize_mission_evaluation(_extract_json(raw_text), len(paths))
    result.update({
        "format": "pre_map_vln.vlm_mission_evaluation.v1",
        "model": body.get("model", MODEL),
        "prompt_version": PROMPT_VERSION,
        "usage": body.get("usage") or {},
        "raw_response": raw_text,
    })
    return result
