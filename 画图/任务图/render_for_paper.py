#!/usr/bin/python3
"""Build a code-aligned paper figure from the supplied CPTG package.

Run with /usr/bin/python3 (uses the installed PyGObject/Rsvg and pycairo).
The extracted reference package is left unchanged. Outputs go to paper/.
"""
from __future__ import annotations

import hashlib
import json
import runpy
from pathlib import Path

import cairo
import gi

gi.require_version("Rsvg", "2.0")
from gi.repository import Rsvg


ROOT = Path(__file__).resolve().parent
PACKAGE = ROOT / "CPTG_TaskGraph_Full_Reproducible"
OUTPUT = ROOT / "paper"
WIDTH, HEIGHT = 1640, 772


def replace_once(svg: str, old: str, new: str) -> str:
    if svg.count(old) != 1:
        raise ValueError(f"Expected exactly one occurrence of {old!r}")
    return svg.replace(old, new, 1)


def paper_svg() -> bytes:
    original = runpy.run_path(str(PACKAGE / "generate.py"))["build_svg"]()
    expected = json.loads((PACKAGE / "source/checksums.json").read_text())
    assert hashlib.sha256(original).hexdigest() == expected["svg_sha256"]
    svg = original.decode("utf-8")
    replacements = [
        # Match the top inset to the side inset; keep the lower edge and
        # instruction-to-parser arrow in their existing positions.
        (
            'x="23.0" y="10.0" width="1594.0" height="142.0"',
            'x="23.0" y="23.0" width="1594.0" height="129.0"',
        ),
        (
            "“First, check if the kettle is next to the stove in the kitchen. Then check if the water bottle on the bedroom desk is",
            "“First, check for the kettle next to the stove in the kitchen. Then check for a water bottle on the bedroom desk,",
        ),
        (
            "empty, if there is tissue next to the bathroom sink, and if the backpack is on the coffee table in the living room.",
            "tissue next to the bathroom sink, and a backpack on the coffee table in the living room. Also check for a book",
        ),
        (
            "Also check if the book is on the table in the living room. If it is not there, go to the bedroom and check the table.”",
            "on the table in the living room. If it is not there, go to the bedroom and check the table.”",
        ),
        ("Predicate: empty", "Check: presence"),
        ("Predicate: present", "Check: presence"),
        ("Predicate: on", "Relation: on"),
        ('x="1316.5" y="250.0" width="56.0"', 'x="1294.5" y="250.0" width="78.0"'),
        (
            'x="1344.5" y="274.0" class="task-tag" text-anchor="middle">T5',
            'x="1333.5" y="274.0" class="task-tag" text-anchor="middle">T5a/b',
        ),
        (
            'x="1317.5" y="506.0" class="two-lbl" text-anchor="middle">Anchor:',
            'x="1317.5" y="506.0" class="two-lbl" text-anchor="middle">T5a anchor:',
        ),
        (
            'x="1505.5" y="506.0" class="two-lbl" text-anchor="middle">Anchor:',
            'x="1505.5" y="506.0" class="two-lbl" text-anchor="middle">T5b anchor:',
        ),
        ('style="fill:#E9545C">T5</text>', 'style="fill:#E9545C">T5a</text>'),
    ]
    # Three graph occurrences refer to the initial book check. The fourth is
    # the explicit bedroom fallback, which must have a distinct task ID.
    for old, new in replacements[:-1]:
        svg = replace_once(svg, old, new)
    assert svg.count("Predicate: next to") == 2
    svg = svg.replace("Predicate: next to", "Relation: next to")
    old, new = replacements[-1]
    assert svg.count(old) == 4
    svg = svg.replace(old, new, 3)
    svg = replace_once(svg, old, 'style="fill:#E9545C">T5b</text>')
    svg = replace_once(svg, '>SKIP</text>', '>Skip T5b</text>')
    return svg.encode("utf-8")


def render(svg: bytes) -> None:
    OUTPUT.mkdir(exist_ok=True)
    (OUTPUT / "CPTG_TaskGraph.svg").write_bytes(svg)
    handle = Rsvg.Handle.new_from_data(svg)
    viewport = Rsvg.Rectangle()
    viewport.x, viewport.y = 0, 0
    viewport.width, viewport.height = WIDTH, HEIGHT

    # Native text and paths remain vectors; only the supplied scene assets
    # are raster images. Page dimensions match IEEE's full text width.
    scale = 516 / WIDTH
    surface = cairo.PDFSurface(str(OUTPUT / "CPTG_TaskGraph.pdf"), 516, HEIGHT * scale)
    surface.set_metadata(cairo.PDF_METADATA_TITLE, "Conditional Partial-Order Task Graph")
    context = cairo.Context(surface)
    context.scale(scale, scale)
    handle.render_document(context, viewport)
    surface.finish()

    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, WIDTH * 2, HEIGHT * 2)
    context = cairo.Context(surface)
    context.set_source_rgb(1, 1, 1)
    context.paint()
    context.scale(2, 2)
    handle.render_document(context, viewport)
    surface.write_to_png(str(OUTPUT / "CPTG_TaskGraph.png"))
    surface.finish()
    for path in sorted(OUTPUT.glob("CPTG_TaskGraph.*")):
        print(path)


if __name__ == "__main__":
    render(paper_svg())
