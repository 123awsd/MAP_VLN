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

base.W, base.H = 135., 76.
e = json.loads((base.OUT / "mechanism_evidence.json").read_text())
base.PREVIOUS = '#A58B70'
base.CHECKED = '#8B91A0'
GUIDE_COLOR = '#C57939'
fig = plt.figure(figsize=(base.W/25.4, base.H/25.4), facecolor=base.BACKGROUND)
ax = base.axes(fig, (0, 0, 135, 76), (0, 135), (0, 76), aspect='auto')
scene = base.axes(fig, (2, 9, 85, 64), (-.05, 12.05), (-.05, 9.05))
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

# The dashed region and the matching detail refer to the same scene bounds.
scene.add_patch(base.Rectangle((8.5, 5.3), 3.4, 3.35, fill=False, ec=base.INK, lw=.8, linestyle=(0, (3, 2)), zorder=25))

# The matching local detail sits to the right of the main scene.
ax.add_patch(base.Rectangle((94, 41), 32, 32, fill=False, ec=base.INK, lw=.8, linestyle=(0, (3, 2))))
for y, target in [(8.65, (94, 73)), (5.3, (94, 41))]:
    fig.add_artist(base.ConnectionPatch(xyA=(11.9, y), xyB=target, coordsA='data', coordsB='data', axesA=scene, axesB=ax, arrowstyle='-', lw=.7, color=base.INK, linestyle=(0, (3, 2)), clip_on=False, zorder=30))
detail = base.axes(fig, (95, 42, 30, 30), (8.5, 11.9), (5.3, 8.65))
base.draw_room(detail, labels=False, anchors=False)
base.line(detail, base.schematic_guide(e['initial'])[:, :2], GUIDE_COLOR, 1., z=7.5)
base.route(detail, e['initial'], first_only=True, underlay=False)
pose = e['initial']['plan']['visits'][0]['pose']
base.view(detail, [pose['x'], pose['y']], pose['yaw'], base.TRAJECTORY, base.TRAJECTORY, radius=.44, selected=True, lw=.75)
detail.add_patch(base.Circle(e['initial']['start'][:2], .07, fc=base.INK, ec=base.BACKGROUND, lw=.4, zorder=18))
# Both path-state and optimization keys share the right-hand column.
base.line(ax, [[95, 34], [99, 34]], base.INK, 1.15)
base.text(ax, 101, 34, 'Executed path', 7.0, base.INK)
base.line(ax, [[95, 28.5], [99, 28.5]], base.PREVIOUS, .9, ls=(0, (3, 2.1)))
base.text(ax, 101, 28.5, 'Initial path', 7.0, base.MUTED)
base.line(ax, [[95, 23], [96.65, 23]], base.TRAJECTORY, 1.15)
base.line(ax, [[97.35, 23], [99, 23]], base.TRAJECTORY, 1.15)
base.text(ax, 101, 23, 'Replanned path', 7.0, base.TRAJECTORY)
base.line(ax, [[95, 16], [99, 16]], GUIDE_COLOR, 1.)
base.text(ax, 101, 16, 'A* guide', 7.0, GUIDE_COLOR)
base.line(ax, [[95, 10.5], [99, 10.5]], base.TRAJECTORY, 1.2)
base.text(ax, 101, 10.5, 'Optimized trajectory', 7.0, base.TRAJECTORY)
for ext in ('pdf', 'svg', 'png'):
    fig.savefig(base.OUT / f'joint_task_viewpoint_planning_single_column_minco_trial.{ext}', dpi=500, facecolor=base.BACKGROUND)
plt.close(fig)
