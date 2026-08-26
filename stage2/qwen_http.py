"""Small bounded retry wrapper for idempotent, temperature-zero Qwen requests."""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from typing import Any, Callable


RETRIABLE_HTTP_STATUS = {408, 409, 425, 429, 500, 502, 503, 504}


def open_json_with_retry(
    request: urllib.request.Request,
    *,
    timeout: float,
    retry_delays: tuple[float, ...] = (2.0, 5.0),
    opener: Callable[..., Any] = urllib.request.urlopen,
    sleeper: Callable[[float], None] = time.sleep,
) -> tuple[dict[str, Any], int]:
    """Return a JSON response and attempt count after bounded transient retries.

    Authentication/validation errors are never retried. The caller remains
    responsible for cost accounting after a valid response is received.
    """

    maximum_attempts = len(retry_delays) + 1
    for attempt in range(1, maximum_attempts + 1):
        try:
            with opener(request, timeout=timeout) as response:
                value = json.load(response)
            if not isinstance(value, dict):
                raise ValueError("Qwen response must be a JSON object")
            return value, attempt
        except urllib.error.HTTPError as error:
            detail = error.read().decode("utf-8", "replace")[:1000]
            if error.code not in RETRIABLE_HTTP_STATUS or attempt == maximum_attempts:
                raise RuntimeError(f"DashScope HTTP {error.code}: {detail}") from error
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as error:
            if attempt == maximum_attempts:
                raise RuntimeError(
                    f"DashScope request failed after {maximum_attempts} attempts: {error}"
                ) from error
        sleeper(retry_delays[attempt - 1])
    raise AssertionError("unreachable")
