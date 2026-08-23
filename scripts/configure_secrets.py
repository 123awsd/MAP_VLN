#!/usr/bin/env python3
"""Store project API credentials outside Git with owner-only permissions."""

from __future__ import annotations

import argparse
import getpass
import os
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SECRET_DIR = ROOT / ".secrets"


def store(name: str, value: str) -> Path:
    value = value.strip()
    if not value:
        raise ValueError(f"empty value for {name}")
    SECRET_DIR.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(SECRET_DIR, 0o700)
    target = SECRET_DIR / name
    descriptor = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        stream.write(value + "\n")
    os.chmod(target, 0o600)
    return target


def matterport_from_document(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    match = re.search(r"matterport\s+apikey\s*[:：]\s*([^\s]+)", text, re.IGNORECASE)
    if not match:
        raise ValueError(f"Matterport API key not found in {path}")
    return match.group(1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--qwen-stdin", action="store_true")
    parser.add_argument("--matterport-document", type=Path)
    args = parser.parse_args()

    configured = []
    if args.qwen_stdin:
        configured.append(store("dashscope_api_key", getpass.getpass("DashScope API key: ")))
    if args.matterport_document:
        configured.append(
            store("matterport_api_key", matterport_from_document(args.matterport_document))
        )
    if not configured:
        parser.error("select at least one credential source")
    for path in configured:
        print(f"configured {path.relative_to(ROOT)} (mode {oct(path.stat().st_mode & 0o777)})")


if __name__ == "__main__":
    main()
