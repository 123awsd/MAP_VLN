#!/usr/bin/env python3
"""Redraw Fig. 8 as a vector mechanism diagram at the paper's two-column width.

All geometry is schematic. Mechanism checks use the existing production-code
fixture; no example geometry or sample counts are experimental measurements.
"""
from pathlib import Path
import json
import os
import sys

os.environ.setdefault("MPLCONFIGDIR", "/tmp/fig8_academic_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyArrowPatch
import numpy as np

OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT.parent))
import mechanism_audit

W, H = 184, 62
INK = "#30383D"
MUTED = "#68737A"
GRAY = "#A4ADB2"
LIGHT = "#DEE3E6"
PAPER = "#F7F8F9"
ACCENT = "#237F79"

plt.rcParams.update({
    "font.family": "Liberation Sans", "font.size": 7,
    "mathtext.fontset": "dejavusans", "pdf.fonttype": 42,
    "svg.fonttype": "none", "lines.solid_capstyle": "butt",
    "lines.solid_joinstyle": "miter",
})


def text(ax, x, y, s, size=7, color=INK, ha="center", weight="normal", **kw):
    return ax.text(x, y, s, fontsize=size, color=color, ha=ha,
                   va="center", weight=weight, zorder=20, **kw)


def line(ax, points, color=GRAY, lw=.6, **kw):
    return ax.plot(*zip(*points), color=color, lw=lw, zorder=3, **kw)


def arrow(ax, start, end, color=INK, lw=.65, rad=0):
    p = FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=4,
                        connectionstyle=f"arc3,rad={rad}", linewidth=lw,
                        color=color, shrinkA=1.5, shrinkB=1.5, zorder=7)
    ax.add_patch(p)


def dot(ax, xy, radius=.45, edge=INK, fill="white", lw=.55, z=5):
    ax.add_patch(Circle(xy, radius, edgecolor=edge, facecolor=fill,
                        linewidth=lw, zorder=z))


def region(ax, x, y, w, h, covered=(), small=False):
    ax.add_patch(Rectangle((x, y), w, h, facecolor=PAPER,
                           edgecolor=GRAY, linewidth=.55, zorder=1))
    positions = []
    for row in range(3):
        for col in range(4):
            pos = (x + w * (.16 + col * .225), y + h * (.2 + row * .3))
            positions.append(pos)
            done = row * 4 + col in covered
            dot(ax, pos, .33 if small else .44,
                edge=GRAY if done else MUTED,
                fill="white" if done else MUTED,
                lw=.45)
    return positions


def viewpoint(ax, origin, target, length=9, angle=19, dashed=False, color=GRAY):
    a = np.asarray(origin, dtype=float)
    d = np.asarray(target, dtype=float) - a
    d /= np.linalg.norm(d)
    n = np.array([-d[1], d[0]])
    p = a + length * d
    offset = length * np.tan(np.deg2rad(angle)) * n
    line(ax, [p + offset, a, p - offset], color=color, lw=.55,
         linestyle=(0, (2, 1.6)) if dashed else "-")
    line(ax, [p - offset, p + offset], color=LIGHT, lw=.45)
    dot(ax, a, .63, edge=color, lw=.65, z=8)


def detection(ax, x, y, w=10, h=8, shift=0):
    ax.add_patch(Rectangle((x, y), w, h, facecolor="white",
                           edgecolor=GRAY, linewidth=.55))
    ax.add_patch(Rectangle((x + .31*w + shift, y + .23*h), .38*w, .54*h,
                           facecolor="none", edgecolor=ACCENT, linewidth=.75))
    dot(ax, (x + .5*w + shift, y + .5*h), .38,
        edge=ACCENT, fill=ACCENT, lw=.4)


def cube(ax, x, y, s=4.3):
    a = [(x, y), (x+s, y), (x+s, y+s), (x, y+s), (x, y)]
    off = np.array([1.9, 1.5])
    line(ax, a, ACCENT, .65)
    line(ax, np.asarray(a[:4]) + off, GRAY, .45)
    for p in a[:4]:
        line(ax, [p, np.asarray(p) + off], ACCENT, .55)


def heading(ax, letter, title):
    text(ax, .5, 53.2, f"({letter})", size=8, ha="left", weight="bold")
    text(ax, 7.3, 53.2, title, size=8, ha="left", weight="bold")
    line(ax, [(.5, 49.8), (42.5, 49.8)], LIGHT, .6)


def local(ax):
    heading(ax, "a", "Local retries")
    text(ax, 21.5, 45.2, r"Current region $r_0$", size=7.3)
    region(ax, 9, 31, 25, 10)
    target = (21.5, 36)
    poses = [(4.8, 22), (21.5, 19.2), (38.2, 22)]
    for i, q in enumerate(poses):
        viewpoint(ax, q, target, length=11, dashed=i < 2,
                  color=GRAY if i < 2 else INK)
        text(ax, q[0], q[1]-3.3, rf"$q_{i+1}$", size=7, color=MUTED)
    arrow(ax, (8.1, 21.0), (18.2, 18.8), rad=.13, color=MUTED)
    arrow(ax, (24.8, 18.8), (34.9, 21.0), rad=.13, color=MUTED)
    text(ax, 21.5, 10.8, "Miss → next local view", size=7.2)
    text(ax, 21.5, 5.1, "Exhausted / local limit reached", size=6.8, weight="bold")


def recovery(ax):
    heading(ax, "b", "Region recovery")
    text(ax, 21.5, 44.4, "Instruction + missing target", size=7, color=MUTED)
    text(ax, 21.5, 40.2, "Failed region + scene inventory", size=6.8, color=MUTED)
    arrow(ax, (21.5, 37.9), (21.5, 33.5), color=MUTED)
    text(ax, 21.5, 31.1, "Infer and ground hypotheses", size=7.1)
    line(ax, [(21.5, 28.6), (21.5, 26.2), (10, 26.2)], MUTED, .6)
    line(ax, [(21.5, 26.2), (33.5, 26.2)], MUTED, .6)
    arrow(ax, (10, 26.2), (10, 23.1), color=MUTED)
    arrow(ax, (33.5, 26.2), (33.5, 23.1), color=MUTED)
    region(ax, 2.5, 14, 15, 8, small=True)
    region(ax, 26, 14, 15, 8, small=True)
    text(ax, 3, 24, r"$r_1$", size=7, ha="left")
    text(ax, 26.5, 24, r"$r_2$", size=7, ha="left")
    viewpoint(ax, (4.5, 9.3), (10, 18), length=4.5, angle=17)
    viewpoint(ax, (16, 9.3), (10, 18), length=4.5, angle=17)
    viewpoint(ax, (28, 9.3), (33.5, 18), length=4.5, angle=17)
    viewpoint(ax, (39.5, 9.3), (33.5, 18), length=4.5, angle=17)
    text(ax, 21.5, 5.1, "Feasible views → replan", size=7.2)


def evidence(ax):
    heading(ax, "c", "Search evidence")
    text(ax, 21.5, 45.2, "Negative observation", size=7.3)
    region(ax, 1.5, 33, 16, 8, covered=(0, 4, 8), small=True)
    region(ax, 25.5, 33, 16, 8, covered=(0, 1, 2, 4, 5, 6, 8, 9, 10), small=True)
    arrow(ax, (18.5, 37), (24.5, 37), color=MUTED)
    text(ax, 9.5, 29.8, r"$U_r$", size=7)
    text(ax, 33.5, 29.8, r"$U_r\cup V(q)$", size=7)
    dot(ax, (6, 25.5), .45, edge=GRAY)
    text(ax, 8, 25.5, "Covered", size=6.5, color=MUTED, ha="left")
    dot(ax, (24, 25.5), .45, edge=MUTED, fill=MUTED)
    text(ax, 26, 25.5, "Unseen", size=6.5, color=MUTED, ha="left")
    text(ax, 21.5, 20.8, "New coverage → lower belief", size=7)
    text(ax, 21.5, 16.8, "Repeat coverage: no further decay", size=6.5, color=MUTED)
    text(ax, 21.5, 12.8, "Anchor remains in the map", size=6.5, color=MUTED)
    detection(ax, 2.7, 1.8, w=7, h=7)
    text(ax, 26, 7.3, "Verified target", size=7.2, color=ACCENT, weight="bold")
    text(ax, 26, 3.2, "Resolve task online", size=7, color=ACCENT)


def memory(ax):
    heading(ax, "d", "Map update")
    text(ax, 21.5, 45.2, "During execution", size=7.3)
    detection(ax, 5, 33, shift=-.5)
    detection(ax, 28, 33, shift=.5)
    text(ax, 10, 30.6, r"$t$", size=6.5, color=MUTED)
    text(ax, 33, 30.6, r"$t+1$", size=6.5, color=MUTED)
    arrow(ax, (15.5, 33), (21, 28), color=MUTED)
    arrow(ax, (27.5, 33), (23.5, 28), color=MUTED)
    cube(ax, 18.5, 25, s=4.3)
    text(ax, 21.5, 22.4, "Cross-view association", size=7)
    line(ax, [(1, 19.8), (42, 19.8)], GRAY, .55, linestyle=(0, (2, 2)))
    text(ax, 21.5, 17, "After execution", size=7.3)
    root = (21.5, 10)
    for xy in [(8.3, 5), (21.5, 4.2), (35.2, 5)]:
        new = xy[0] > 30
        line(ax, [root, xy], ACCENT if new else GRAY, .7 if new else .55)
        dot(ax, xy, .95, edge=ACCENT if new else MUTED,
            fill=ACCENT if new else "white", lw=.65)
    dot(ax, root, 1.05, edge=MUTED, lw=.65)
    text(ax, 21.5, 12.7, "Room", size=6.5, color=MUTED)
    text(ax, 8.3, 1.5, "Existing", size=6.5, color=MUTED)
    text(ax, 35.2, 1.5, "New instance", size=6.5, color=ACCENT)


def render():
    fig = plt.figure(figsize=(W/25.4, H/25.4), facecolor="white")
    for x, draw in zip((1.5, 47.5, 93.5, 139.5), (local, recovery, evidence, memory)):
        ax = fig.add_axes([x/W, 5/H, 43/W, 55/H])
        ax.set(xlim=(0, 43), ylim=(0, 55), aspect="equal")
        ax.set_axis_off()
        draw(ax)
    # A shared line clarifies the guard and task identity without adding a card.
    ax = fig.add_axes([0, 0, 1, 1], zorder=-1)
    ax.set(xlim=(0, W), ylim=(0, H), aspect="equal")
    ax.set_axis_off()
    text(ax, 69.5, 2.4, "Recovery permitted · same task until verification", size=6.8, color=MUTED)
    fig.canvas.draw()
    boxes = [(t, t.get_window_extent()) for a in fig.axes for t in a.texts]
    for t, b in boxes:
        assert fig.bbox.contains(b.x0, b.y0) and fig.bbox.contains(b.x1, b.y1), t.get_text()
    for i, (t, b) in enumerate(boxes):
        for u, c in boxes[i+1:]:
            assert not b.overlaps(c), (t.get_text(), u.get_text())
    for ext in ("pdf", "svg", "png"):
        fig.savefig(OUT/f"fig8_academic_en.{ext}", dpi=600,
                    metadata={"Creator": "Fig. 8 vector redraw"} if ext == "pdf" else None)
    fig.savefig(OUT/"fig8_academic_en_preview.png", dpi=240)
    plt.close(fig)
    return {"size_mm": [W, H], "minimum_regular_text_pt": 6.5,
            "math_subscripts_use_standard_reduced_size": True,
            "text_bounds_and_overlaps_checked": True,
            "geometry_is_schematic": True, "data_or_runtime_measurements_generated": False,
            "accent_semantics": "positive verification and confirmed novel instance",
            "arrows_are_process_relations_not_flight_trajectories": True,
            "source_figures_and_paper_unchanged": True}


if __name__ == "__main__":
    # Only the new output directory receives the audit fixture's output.
    mechanism_audit.OUT = OUT
    checks = mechanism_audit.audit()
    checks["redraw"] = render()
    (OUT/"mechanism_evidence.json").write_text(json.dumps(checks, indent=2)+"\n")
    for p in sorted(OUT.glob("fig8_academic_en.*")):
        print(p)
