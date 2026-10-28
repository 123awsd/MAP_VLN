#!/usr/bin/env python3
"""
Regenerate the exact, self-contained SVG figure from the source template
and bundled PNG assets, completely offline.

Usage:
    python generate.py
    python generate.py --no-png
    python generate.py --output-dir /your/path

Dependencies: Python 3.9+; CairoSVG is needed only for PNG rendering.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / 'assets'
TEMPLATE = ROOT / 'source' / 'figure_template.svg'
EXPECTED = ROOT / 'source' / 'checksums.json'


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def build_svg() -> bytes:
    template = TEMPLATE.read_text(encoding='utf-8')
    placeholders = re.findall(r'\{\{BASE64:([^}]+)\}\}', template)
    if len(placeholders) != 7:
        raise RuntimeError(f'Expected 7 image placeholders, got {len(placeholders)}')
    for filename in placeholders:
        # Keep all paths within the bundled assets directory.
        if Path(filename).name != filename or not filename.endswith('.png'):
            raise ValueError(f'Invalid asset name in SVG template: {filename}')
        data = (ASSETS / filename).read_bytes()
        template = template.replace('{{BASE64:' + filename + '}}', base64.b64encode(data).decode('ascii'))
    if '{{BASE64:' in template:
        raise ValueError('Some embedded images were not replaced')
    return template.encode('utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description='Rebuild CPTG SVG and PNG from local assets')
    parser.add_argument('--no-png', action='store_true', help='Only generate SVG; no third-party modules needed')
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'build')
    args = parser.parse_args()
    outdir = args.output_dir.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    manifest = json.loads(EXPECTED.read_text(encoding='utf-8'))

    svg_bytes = build_svg()
    svg_path = outdir / 'CPTG_TaskGraph.svg'
    svg_path.write_bytes(svg_bytes)
    svg_hash = sha256(svg_bytes)
    print(f'SVG: {svg_path}')
    print(f'SVG SHA-256: {svg_hash}')
    if svg_hash != manifest['svg_sha256']:
        print('NOTE: SVG differs from the supplied reference (perhaps assets/template were edited).')
    else:
        print('SVG matches the supplied final reference byte for byte.')

    if not args.no_png:
        try:
            import cairosvg
        except ImportError:
            print('PNG skipped: install dependencies with `python -m pip install -r requirements.txt`.')
            print('The SVG is complete and can already be opened in browsers/Inkscape/Illustrator.')
            return 0
        png_path = outdir / 'CPTG_TaskGraph.png'
        cairosvg.svg2png(bytestring=svg_bytes, write_to=str(png_path), output_width=1640, output_height=772)
        png_hash = sha256(png_path.read_bytes())
        print(f'PNG: {png_path}')
        print(f'PNG SHA-256: {png_hash}')
        if png_hash == manifest['png_sha256']:
            print('PNG also matches the provided reference byte for byte.')
        else:
            print('NOTE: PNG may differ across OS / installed fonts / Cairo versions; SVG is the canonical source.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
