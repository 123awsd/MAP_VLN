#!/usr/bin/env python3
"""A full-floor figure of the measured wall-supported room partition.

Re-executes the relevant, unmodified source statements on saved measured
inputs, then requires exact agreement with the labels consumed by the room
semantic classifier. This script never writes into the project or paper.
"""
from pathlib import Path
import ast
import hashlib
import heapq
import json
import os
import textwrap

os.environ.setdefault('MPLCONFIGDIR', '/tmp/room_partition_figure_mpl')
import cv2
import numpy as np
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.path import Path as MplPath
from matplotlib.patches import PathPatch, Rectangle, FancyArrowPatch

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
REPO = ROOT / 'PRE_MAP_VLN'
BASE = REPO / 'outputs/coverage700_wall_room_preview'
SOURCE = REPO / 'scripts/run_wall_structural_rooms.py'
WATERSHED = REPO / 'scripts/segment_rooms_wall_guided.py'
FLOOR = 2
W, H = 88., 76.
INK = '#293D49'
MUTED = '#6C7D86'
WALL = '#5E7280'
DOMAIN = '#C9E4F3'
EXCLUDED = '#CC4B16'
UNTYPED = '#BEC7CE'
# Balanced lightness across region hues. Quiet input layers leave the seed
# and output colors as the visual focus; avoid neon yellow and hot pink.
PALETTE = ['#76B9DC', '#F0C66C', '#85CDB0', '#ADA5D6',
           '#EEAD8F', '#80CBCE']
plt.rcParams.update({
    'font.family': 'Liberation Sans', 'font.size': 7, 'font.weight': 'bold',
    'pdf.fonttype': 42, 'svg.fonttype': 'none',
    'hatch.linewidth': .25, 'lines.solid_capstyle': 'round',
})


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reproduce(floor):
    """Extract actual source code, with intermediate snapshots only."""
    path = BASE / 'wall_structural_rooms' / f'L{floor}_constrained.npz'
    data = dict(np.load(path))
    wall_source = BASE / 'wall_structural_evidence' / f'L{floor}_walls.npz'
    points = np.load(wall_source)['wall_points']
    shape = data['labels'].shape
    resolution = float(data['resolution'])
    ij = np.floor((points[:, :2] - data['origin_xy']) / resolution).astype(int)
    valid = ((ij[:, 0] >= 0) & (ij[:, 0] < shape[1]) &
             (ij[:, 1] >= 0) & (ij[:, 1] < shape[0]))
    wall = np.zeros(shape, np.uint8)
    wall[ij[valid, 1], ij[valid, 0]] = 1
    barrier = cv2.morphologyEx(wall, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)) > 0
    assert np.array_equal(wall, data['measured_wall'])
    assert np.array_equal(barrier, data['wall_barrier'])
    prior_path = BASE / 'wall_guided/measured' / f'L{floor}_rooms.npz'
    prior = np.load(prior_path)
    domain = (prior['domain'] | prior['wall_barrier']) & ~barrier
    direct = (data['surface_protected'] | data['free_protected'] |
              data['trajectory_protected'] | data['ray_protected'])
    assert np.array_equal(domain, data['original_domain'])
    assert np.array_equal(domain & direct, data['constrained_domain'])
    assert np.array_equal(domain & ~direct, data['side_excluded_candidates'])
    assert not data['fallback_seed'].any()

    node = next(n for n in ast.parse(WATERSHED.read_text()).body
                if isinstance(n, ast.FunctionDef) and n.name == 'watershed')
    namespace = {'np': np, 'heapq': heapq}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(WATERSHED), 'exec'), namespace)
    source = SOURCE.read_text()
    start = source.index(' dist=nd.distance_transform_edt(newdomain)')
    stop = source.index(' for rid in np.unique(old):', start)
    tail = textwrap.dedent(source[start:stop])
    marker = "labels=ns['watershed'](dist,seed,newdomain);merges=[]"
    assert tail.count(marker) == 1
    tail = tail.replace(marker, marker + ';expanded=labels.copy()')
    env = dict(np=np, cv2=cv2, nd=nd, ns=namespace,
               newdomain=data['constrained_domain'], shape=shape,
               res=resolution, bar=barrier, wall=wall, old=data['original_labels'])
    exec(compile(tail, str(SOURCE), 'exec'), env)
    assert np.array_equal(env['seed'], data['seeds'])
    assert np.array_equal(env['display'], data['labels'])
    assert not ((data['labels'] > 0) & barrier).any()
    assert not ((data['labels'] > 0) & ~data['constrained_domain']).any()
    data['distance_m'] = env['dist']
    data['expanded'] = env['expanded']
    record = {
        'floor': floor, 'saved_seeds_exact_match': True,
        'saved_labels_exact_match': True, 'reprojected_walls_exact_match': True,
        'observation_gate_exact_match': True, 'wall_overlap_cells': 0,
        'fallback_seed_cells': 0, 'merges_in_this_example': env['merges'],
        'hidden_fragments_in_this_example': env['hidden'],
        'unassigned_supported_cells': int((data['constrained_domain'] & (data['labels'] == 0)).sum()),
        'room_regions': int(len(np.unique(data['labels'][data['labels'] > 0]))),
        'input_files': {str(p.relative_to(ROOT)): digest(p)
                        for p in (path, wall_source, prior_path)},
    }
    return data, record


def rectangle_runs(mask):
    """Lossless cell-edge vectorization; no contour smoothing or wall edits."""
    polygons, active = [], {}
    for y, row in enumerate(mask):
        change = np.diff(np.pad(row.astype(np.int8), (1, 1)))
        runs = set(zip(np.flatnonzero(change == 1), np.flatnonzero(change == -1)))
        for (x0, x1), y0 in list(active.items()):
            if (x0, x1) not in runs:
                polygons.append([(x0, y0), (x1, y0), (x1, y), (x0, y)])
                del active[(x0, x1)]
        for run in runs:
            active.setdefault(run, y)
    for (x0, x1), y0 in active.items():
        polygons.append([(x0, y0), (x1, y0), (x1, len(mask)), (x0, len(mask))])
    return polygons


def fill(ax, mask, color, z=1):
    polygons = rectangle_runs(mask)
    if polygons:
        # One compound fill avoids PDF antialiasing seams between cell runs.
        rectangles = np.asarray(polygons, float)
        vertices = np.concatenate([rectangles, rectangles[:, :1]], axis=1).reshape(-1, 2)
        codes = np.tile([MplPath.MOVETO, MplPath.LINETO, MplPath.LINETO,
                         MplPath.LINETO, MplPath.CLOSEPOLY], len(rectangles))
        ax.add_patch(PathPatch(MplPath(vertices, codes), facecolor=color,
                               edgecolor='none', antialiased=False, zorder=z))


def room_colors(labels):
    """Reuse a small palette, assigning different colors to nearby regions."""
    ids = sorted(np.unique(labels[labels > 0]),
                 key=lambda rid: -int((labels == rid).sum()))
    colors, indices = {}, {}
    for rid in ids:
        near = nd.binary_dilation(labels == rid, iterations=9)
        neighbors = set(np.unique(labels[near])) - {0, rid}
        taken = {indices[n] for n in neighbors if n in indices}
        usage = {i: list(indices.values()).count(i) for i in range(len(PALETTE))}
        available = [i for i in usage if i not in taken] or list(usage)
        index = min(available, key=lambda i: (usage[i], i))
        indices[int(rid)] = index
        colors[int(rid)] = PALETTE[index]
    return colors


def axes_mm(fig, x, y, width, height):
    return fig.add_axes([x / W, y / H, width / W, height / H])


def make_figure(data):
    """Measured cloud and four complete-floor stages, without clipping."""
    fig = plt.figure(figsize=(W/25.4, H/25.4), facecolor='white')
    canvas = axes_mm(fig, 0, 0, W, H)
    canvas.set(xlim=(0, W), ylim=(0, H)); canvas.axis('off')
    masks = {k: np.fliplr(data[k]) for k in
             ['original_domain', 'constrained_domain', 'side_excluded_candidates',
              'seeds', 'labels', 'wall_barrier', 'distance_m']}
    geometry_path = REPO/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz'
    geometry = np.load(geometry_path)
    points, normals = geometry[f'p{FLOOR-1}'], geometry[f'n{FLOOR-1}']
    assert np.isfinite(points).all()
    assert len(points)==len(normals)
    res = float(data['resolution'])
    point_xy = np.column_stack([
        data['labels'].shape[1]-(points[:, 0]-data['origin_xy'][0])/res,
        (points[:, 1]-data['origin_xy'][1])/res])
    # Fit every occupied display cell and every original measured point. Remove
    # only empty exterior margins; no building content is cropped or masked.
    visible = (masks['original_domain'] | masks['wall_barrier'] |
               (masks['labels']>0) | (masks['seeds']>0))
    yy, xx = np.where(visible)
    bounds = (int(np.floor(min(xx.min(), point_xy[:, 0].min()))-6),
              int(np.ceil(max(xx.max()+1, point_xy[:, 0].max()))+6),
              int(np.floor(min(yy.min(), point_xy[:, 1].min()))-6),
              int(np.ceil(max(yy.max()+1, point_xy[:, 1].max()))+6))
    assert (xx>=bounds[0]).all() and (xx+1<=bounds[1]).all()
    assert (yy>=bounds[2]).all() and (yy+1<=bounds[3]).all()
    assert ((point_xy[:, 0]>=bounds[0]) & (point_xy[:, 0]<=bounds[1])).all()
    assert ((point_xy[:, 1]>=bounds[2]) & (point_xy[:, 1]<=bounds[3])).all()
    colors = room_colors(masks['labels'])
    axes = []
    titles = ['(a) Point cloud', '(b) Wall constraints', '(c) Supported space',
              '(d) Core seeds', '(e) Room regions']
    positions = [(1.,42.),(31.,42.),(61.,42.),(16.,8.),(46.,8.)]
    for i, ((x,y), title) in enumerate(zip(positions,titles)):
        canvas.text(x+13., y+29., title, fontsize=6.1, ha='center', va='center', color=INK)
        ax = axes_mm(fig, x, y, 26., 27.)
        ax.set(xlim=bounds[:2], ylim=bounds[2:]); ax.set_aspect('equal'); ax.axis('off')
        axes.append(ax)
        if i in (0,1,3):
            canvas.add_patch(FancyArrowPatch((x+26.6,y+13.5),(x+29.4,y+13.5),
                             arrowstyle='-|>',mutation_scale=4.5,linewidth=.65,
                             color='#708493',shrinkA=0,shrinkB=0))

    # (a) Every cached measured point is drawn, without height filtering,
    # point dropping, or wall overlays. Normal-dependent styling only makes
    # vertical structure legible through horizontal surfaces in the XY view.
    horizontal = np.abs(normals[:, 2])>=.9
    for select, color, alpha, size in [
        (horizontal, '#C3D7E0', .22, .025),
        (~horizontal, '#668FA8', .72, .035)]:
        p = point_xy[select]
        axes[0].scatter(p[:, 0], p[:, 1], s=size, c=color, alpha=alpha,
                         linewidths=0, rasterized=True, zorder=2)
    # (b) The actual wall-constrained candidate footprint.
    fill(axes[1], masks['original_domain'], '#F1F4F6')
    # (c) Observation support retains candidates; excluded cells stay visible.
    fill(axes[2], masks['constrained_domain'], DOMAIN)
    fill(axes[2], masks['side_excluded_candidates'], EXCLUDED)
    # (d) Seed regions are exact full-floor high-clearance cores, not centers.
    cmap = LinearSegmentedColormap.from_list('distance', ['#F2F6F8', '#E0EAF0', '#C3D7E1'])
    distance = masks['distance_m']
    levels = np.linspace(0, distance.max()+1e-6, 15)
    for lo, hi in zip(levels[:-1], levels[1:]):
        fill(axes[3], masks['constrained_domain'] & (distance>=lo) & (distance<hi),
             cmap((lo+hi)/(2*distance.max())))
    seed_to_room = {}
    for seed in np.unique(masks['seeds'][masks['seeds']>0]):
        core = masks['seeds']==seed
        values = masks['labels'][core]
        assert (values>0).all() and len(np.unique(values))==1
        rid = int(values[0]); seed_to_room[int(seed)] = rid
        fill(axes[3], core, colors[rid], z=3)
        axes[3].contour(core.astype(float), levels=[.5], colors=['white'],
                        linewidths=.32, origin='lower',
                        extent=(0, core.shape[1], 0, core.shape[0]), zorder=4)
    # (e) Full-run output includes the geometric merge checks. This selected
    # floor has no triggered merge; a separate before/after panel would mislead.
    fill(axes[4], masks['constrained_domain'], UNTYPED)
    for rid, color in colors.items():
        fill(axes[4], masks['labels']==rid, color, z=2)
    for ax in axes[1:]:
        fill(ax, masks['wall_barrier'], WALL, z=5)
    # Emphasize the actual wall mask in (b); stroke is display-only.
    wall_mask = masks['wall_barrier']
    fill(axes[1], wall_mask, '#164B69', z=6)
    axes[1].contour(wall_mask.astype(float), levels=[.5], colors=['#164B69'],
                    linewidths=.45, origin='lower',
                    extent=(0, wall_mask.shape[1], 0, wall_mask.shape[0]), zorder=7)

    # One small shared key instead of explanatory boxes or branching wires.
    for x, color, s in [(5., DOMAIN, 'Retained'), (31., EXCLUDED, 'Excluded'),
                        (57., UNTYPED, 'Unassigned')]:
        canvas.add_patch(Rectangle((x, 2.65), 2., 1.7, facecolor=color, edgecolor='none'))
        canvas.text(x+3.1, 3.5, s, fontsize=6.1, color=MUTED, va='center')
    for extension in ('pdf', 'svg', 'png'):
        fig.savefig(OUT/f'room_segmentation_en.{extension}', dpi=600, facecolor='white')
    fig.savefig(OUT/'room_segmentation_en_preview.png', dpi=400, facecolor='white')
    plt.close(fig)
    return {'floor': FLOOR, 'size_mm': [W, H],
            'layout': 'two-row single-column figure: a–c above, d–e below; equal map scale',
            'display_x_reflected': True, 'display_cell_bounds_after_reflection': list(bounds),
            'map_content_cropped': False, 'all_nonempty_cells_in_display_bounds': True,
            'visible_nonempty_cells': int(visible.sum()),
            'palette': {'regions': PALETTE, 'supported_domain': DOMAIN,
                        'excluded': EXCLUDED, 'unassigned': UNTYPED, 'walls': WALL},
            'point_cloud': {'input_file': str(geometry_path.relative_to(ROOT)),
                            'sha256': digest(geometry_path), 'point_count': int(len(points)),
                            'drawn_point_count': int(len(point_xy)), 'projection': 'XY, X reflected',
                            'height_filter_applied': False, 'points_subsampled': False,
                            'all_points_in_display_bounds': True,
                            'appearance': 'normal-dependent color and opacity; horizontal surfaces are pale',
                            'rasterized_dpi': 600},
            'label_colors': colors, 'seed_to_room': seed_to_room,
            'visual_reference': 'https://arxiv.org/pdf/2203.03519 (Fig. 1; presentation only, not the algorithm)'}


def main():
    records = []
    for floor in (1, 2, 3):
        data, record = reproduce(floor)
        records.append(record)
        if floor == FLOOR: selected = data
    visual = make_figure(selected)
    evidence = {
        'source_variant': 'wall_structural_rooms: offline measured checkpoint',
        'downstream_consumer': 'scripts/classify_coverage700_rooms_multiview.py:make_inputs',
        'not_default_online_segmentation_claim': True,
        'uses_ground_truth': False, 'uses_manual_boundary_correction': False,
        'paper_updated_by_renderer': False,
        'source_files': {str(p.relative_to(ROOT)): digest(p) for p in (
            SOURCE, WATERSHED, REPO/'scripts/run_wall_structural_trial.py',
            REPO/'scripts/classify_coverage700_rooms_multiview.py')},
        'checks': records, 'figure': visual,
        'interpretation': [
            'Panel a shows every point in the cached measured floor cloud used by wall extraction; no height or spatial filtering is applied for display.',
            'Panel b shows the extracted wall barriers and inherited candidate domain; upstream geometry reconstruction and wall extraction internals are not expanded.',
            'Supported space is the prior candidate domain intersected with horizontal surface OR multi-height FREE OR trajectory OR multi-frame ray support.',
            'Distance is computed to every boundary of supported space, not only to wall cells.',
            'Seed regions are connected distance-threshold cores passing the source area gate; no fallback point is added.',
            'Watershed is the source four-connected masked minimax flood. Merge rules are executed; zero merges occur in the illustrated floor.',
            'Display-fragment filtering and overlap-based ID association are also replayed; no fragments are hidden on this floor.',
            'Region colors are display identities, not semantic classes or room ground truth.',
            'Excluded means unsupported by the gate, not proven outside or occupied.',
            'Gray supported cells without assigned seeds remain unassigned; the map is not a collision-free navigation map.',
        ],
    }
    (OUT/'mechanism_evidence.json').write_text(json.dumps(evidence, indent=2) + '\n')
    print(json.dumps({'artifacts': str(OUT), 'checks': [
        {k:v for k,v in r.items() if k!='input_files'} for r in records]}, indent=2))


if __name__ == '__main__':
    main()
