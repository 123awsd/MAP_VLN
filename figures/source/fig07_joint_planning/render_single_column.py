#!/usr/bin/env python3
"""Compact single-column Fig. 7, using the original planning and trajectory data."""
import os
import ast
import json
import types
import sys
from pathlib import Path
os.environ.setdefault("MPLCONFIGDIR", "/tmp/jtvp_figure_mpl")
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import numpy as np
import matplotlib.pyplot as plt
source = Path(__file__).with_name("render_figure.py")
tree = ast.parse(source.read_text())
tree.body = [node for node in tree.body if not (isinstance(node, ast.ImportFrom) and (node.module or "").startswith("stage2."))]
base = types.ModuleType("figure_drawing")
base.__file__ = str(source)
exec(compile(tree, str(source), "exec"), base.__dict__)

base.W, base.H = 89., 74.
e = json.loads((base.OUT / "mechanism_evidence.json").read_text())
fig = plt.figure(figsize=(base.W/25.4, base.H/25.4), facecolor=base.BACKGROUND)
ax = base.axes(fig, (0, 0, 89, 74), (0, 89), (0, 74), aspect='auto')
scene = base.axes(fig, (2, 7, 85, 64), (-.05, 12.05), (-.05, 9.05))
# Scale the symbols in the main scene while retaining their original coordinates.
_original_badge, _original_view = base.badge, base.view
def larger_badge(ax, x, y, label, color, fill=None, r=2.0, size=6.7):
    return _original_badge(ax, x, y, label, color, fill=fill, r=r*1.18, size=size*1.5)
def larger_view(ax, xy, yaw, color, fill, radius=.36, selected=False, lw=.55, apex_radius=None):
    return _original_view(ax, xy, yaw, color, fill, radius=radius*(1.15 if selected else 1.08), selected=selected, lw=lw, apex_radius=apex_radius)
base.badge, base.view = larger_badge, larger_view
base.draw_room(scene, labels=False, cycle=e['updated'])
_, previous, _ = base.merged_phases(e)
base.line(scene, np.asarray(previous['points'])[:, :2], base.PREVIOUS, .9, z=8, ls=(0, (3, 2.1)))
for phase in e['updated']['minco']:
    points = np.asarray(phase['points'])[:, :2]
    base.line(scene, points, base.TRAJECTORY, 1.25, z=9, ls=(0, (3, 1.8)))
    base.direction(scene, points, base.TRAJECTORY)
executed = np.asarray(e['initial']['minco'][0]['points'])[:, :2]
base.line(scene, executed, base.INK, 1.25, z=10)
base.direction(scene, executed, base.INK)
base.draw_views(scene, e['updated'])
base.drone(scene, e['initial']['start'])
sx, sy = e['initial']['start'][:2]
base.text(scene, sx-.3, sy, 'Start', 8.4, ha='right')
base.text(scene, 9.85, 6.85, 'Not found', 7.8, base.CHECKED, ha='center')

# Route legend sits in one row immediately beneath the main scene.
base.line(ax, [[3, 4.5], [8, 4.5]], base.INK, 1.15)
base.text(ax, 9.5, 4.5, 'Executed path', 7.5, base.INK)
base.line(ax, [[30, 4.5], [35, 4.5]], base.PREVIOUS, .9, ls=(0, (3, 2.1)))
base.text(ax, 36.5, 4.5, 'Initial path', 7.5, base.MUTED)
base.line(ax, [[55, 4.5], [60, 4.5]], base.TRAJECTORY, 1.15, ls=(0, (3, 1.8)))
base.text(ax, 61.5, 4.5, 'Replanned path', 7.5, base.TRAJECTORY)

for ext in ('pdf', 'svg', 'png'):
    fig.savefig(base.OUT / f'joint_task_viewpoint_planning_single_column.{ext}', dpi=500, facecolor=base.BACKGROUND)
plt.close(fig)
