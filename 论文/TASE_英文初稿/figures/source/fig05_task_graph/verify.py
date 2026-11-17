#!/usr/bin/env python3
"""Verify bundled master files and locally regenerated files with no external dependencies."""
from pathlib import Path
import hashlib, json, sys
ROOT = Path(__file__).resolve().parent
checks = json.loads((ROOT/'source'/'checksums.json').read_text(encoding='utf-8'))
paths = [
    (ROOT/'CPTG_TaskGraph.svg',checks['svg_sha256']),
    (ROOT/'CPTG_TaskGraph.png',checks['png_sha256']),
]
for name,sha in [('CPTG_TaskGraph.svg','svg_sha256'),('CPTG_TaskGraph.png','png_sha256')]:
    bp = ROOT/'build'/name
    if bp.exists(): paths.append((bp,checks[sha]))
failures = 0
for path, expected in paths:
    if not path.exists():
        print(f'MISSING {path}')
        failures += 1
        continue
    actual=hashlib.sha256(path.read_bytes()).hexdigest()
    print(f'{"OK" if actual==expected else "DIFFERENT"}: {path.relative_to(ROOT)}')
    if actual!=expected: failures += 1
if failures:
    print('Different PNG files can result from changes in OS/fonts/rendering libraries.')
sys.exit(1 if failures else 0)
