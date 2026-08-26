#!/usr/bin/env python3
"""Securely store the official HM3D download username/password for curl."""

from __future__ import annotations

import getpass
import json
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    username = input("Matterport HM3D download username: ").strip()
    password = getpass.getpass("Matterport HM3D download password: ")
    if not username or not password:
        raise SystemExit("username and password must both be non-empty")
    path = ROOT / ".secrets/hm3d_curl.conf"
    path.parent.mkdir(parents=True, exist_ok=True)
    # JSON string escaping is also valid for curl config quoted strings.
    path.write_text("user = {}\n".format(json.dumps(username + ":" + password)), encoding="utf-8")
    os.chmod(path, 0o600)
    print("saved encrypted-transport download credentials to {} (mode 0600)".format(path))


if __name__ == "__main__":
    main()
