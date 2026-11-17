#!/usr/bin/env python3
"""Render paper-ready 3-D occupancy voxel assets from the real Stage-1 map.

The whole-building panels show OCCUPIED voxels only because FREE and UNKNOWN
volumes would occlude the reconstructed structure.  A separate local cutaway
shows the actual OCCUPIED/FREE/UNKNOWN state semantics together.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ROOT = Path(__file__).resolve().parents[3]
EXPORT = ROOT / "outputs/stage1_3d/00337-CFVBbU9Rsyb/dormant_recovery_20260829/bag_export"
FLOORS_JSON = ROOT / "outputs/room_pipeline/00337_dormant_recovery_20260829_boxfloorfix/floors.json"
CANDIDATE_OUT = ROOT / "paper/system_overview/assets/candidates/voxel_occupancy"
SELECTED_OUT = ROOT / "paper/system_overview/assets/selected/voxel_occupancy"

FLOOR_PALETTES = {
    "ocean": ("#2F6B9A", "#29A38C", "#E28A4B"),
    "berry": ("#425D9B", "#8A63A8", "#D95E76"),
    "mineral": ("#31576B", "#789C79", "#C47B58"),
}

STATE_PALETTES = {
    "cyan": {"occupied": "#17324D", "free": "#45B8B0", "unknown": "#DDE3E8"},
    "warm": {"occupied": "#3C3549", "free": "#E3A13B", "unknown": "#E7E2EA"},
    "paper": {"occupied": "#263B59", "free": "#7BB7D1", "unknown": "#D9D9D9"},
}

HEIGHT_CMAP = mcolors.LinearSegmentedColormap.from_list(
    "voxel_height", ("#17365D", "#247B9B", "#35A98F", "#E4B64D", "#D2674A")
)


def pcd_header_lines(path: Path) -> int:
    count = 0
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            count += 1
            if line.strip().lower().startswith("data"):
                return count
    raise ValueError(f"PCD DATA header missing: {path}")


def load_pcd(path: Path) -> np.ndarray:
    return np.loadtxt(path, dtype=np.float32, skiprows=pcd_header_lines(path), usecols=(0, 1, 2))


def load_pcd_crop(path: Path, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Stream a crop so the 4.6M UNKNOWN voxels are never fully materialized."""
    rows: list[tuple[float, float, float]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip().lower().startswith("data"):
                break
        for line in handle:
            values = line.split()
            if len(values) < 3:
                continue
            x, y, z = map(float, values[:3])
            if lo[0] <= x < hi[0] and lo[1] <= y < hi[1] and lo[2] <= z < hi[2]:
                rows.append((x, y, z))
    return np.asarray(rows, dtype=np.float32).reshape(-1, 3)


def floor_data() -> tuple[list[dict], np.ndarray]:
    floors = json.loads(FLOORS_JSON.read_text(encoding="utf-8"))[:3]
    z = np.asarray([item["floor_z_m"] for item in floors], dtype=float)
    bounds = np.asarray([z[0] - 0.35, (z[0] + z[1]) / 2,
                         (z[1] + z[2]) / 2, z[2] + 2.85])
    return floors, bounds


def grid_from_points(points: np.ndarray, voxel: float, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    shape = np.ceil((hi - lo) / voxel).astype(int)
    grid = np.zeros(tuple(shape), dtype=bool)
    if len(points) == 0:
        return grid
    idx = np.floor((points - lo) / voxel).astype(int)
    valid = np.all((idx >= 0) & (idx < shape), axis=1)
    idx = idx[valid]
    grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    return grid


def rgba(hex_color: str, alpha: float = 1.0) -> np.ndarray:
    value = np.asarray(mcolors.to_rgba(hex_color), dtype=float)
    value[3] = alpha
    return value


def style_ax(ax, shape: tuple[int, int, int], elev=24, azim=-58, z_scale=0.86):
    ax.set_axis_off()
    ax.set_facecolor("white")
    ax.set_proj_type("ortho")
    ax.set_box_aspect((shape[0], shape[1], max(1, shape[2]) * z_scale))
    ax.view_init(elev=elev, azim=azim)


def voxel_plot(ax, filled: np.ndarray, facecolors: np.ndarray,
               edge=(1, 1, 1, 0.10), linewidth=0.05):
    """Draw only exposed faces, avoiding one artist per occupied voxel."""
    padded = np.pad(filled, 1, constant_values=False)
    directions = (
        ((-1, 0, 0), np.array([[0, 0, 0], [0, 1, 0], [0, 1, 1], [0, 0, 1]])),
        ((1, 0, 0), np.array([[1, 0, 0], [1, 0, 1], [1, 1, 1], [1, 1, 0]])),
        ((0, -1, 0), np.array([[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]])),
        ((0, 1, 0), np.array([[0, 1, 0], [1, 1, 0], [1, 1, 1], [0, 1, 1]])),
        ((0, 0, -1), np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]])),
        ((0, 0, 1), np.array([[0, 0, 1], [0, 1, 1], [1, 1, 1], [1, 0, 1]])),
    )
    all_faces = []
    all_colors = []
    core = padded[1:-1, 1:-1, 1:-1]
    for (dx, dy, dz), corners in directions:
        neighbor = padded[1 + dx:1 + dx + filled.shape[0],
                          1 + dy:1 + dy + filled.shape[1],
                          1 + dz:1 + dz + filled.shape[2]]
        exposed = core & ~neighbor
        idx = np.argwhere(exposed)
        if not len(idx):
            continue
        all_faces.append(idx[:, None, :] + corners[None, :, :])
        all_colors.append(facecolors[exposed])
    faces = np.concatenate(all_faces, axis=0)
    colors = np.concatenate(all_colors, axis=0)
    collection = Poly3DCollection(faces, facecolors=colors, edgecolors=edge,
                                  linewidths=linewidth, shade=True,
                                  lightsource=mcolors.LightSource(azdeg=315, altdeg=40))
    collection.set_rasterized(True)
    ax.add_collection3d(collection)
    ax.set_xlim(0, filled.shape[0])
    ax.set_ylim(0, filled.shape[1])
    ax.set_zlim(0, filled.shape[2])


def save(fig, stem: str):
    CANDIDATE_OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(CANDIDATE_OUT / f"{stem}.png", dpi=320, bbox_inches="tight",
                pad_inches=0.025, facecolor="white", transparent=False)
    fig.savefig(CANDIDATE_OUT / f"{stem}.pdf", dpi=320, bbox_inches="tight",
                pad_inches=0.025, facecolor="white", transparent=False)
    plt.close(fig)


def whole_building(points: np.ndarray, bounds: np.ndarray, palette_name: str):
    palette = FLOOR_PALETTES[palette_name]
    points = points[(points[:, 2] >= bounds[0]) & (points[:, 2] < bounds[-1])]
    # 0.30 m is a display aggregation; the source map remains at 0.10 m.
    # It keeps exposed cube faces legible and the paper asset lightweight.
    voxel = 0.30
    lo = np.floor(points.min(axis=0) / voxel) * voxel
    hi = np.ceil(points.max(axis=0) / voxel) * voxel + voxel
    filled = grid_from_points(points, voxel, lo, hi)
    colors = np.zeros(filled.shape + (4,), dtype=np.float32)
    zc = lo[2] + (np.arange(filled.shape[2]) + 0.5) * voxel
    for floor in range(3):
        mask_z = (zc >= bounds[floor]) & (zc < bounds[floor + 1])
        colors[:, :, mask_z, :] = rgba(palette[floor], 0.96)
    fig = plt.figure(figsize=(7.1, 7.1), facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    voxel_plot(ax, filled, colors)
    style_ax(ax, filled.shape, elev=25, azim=-57)
    save(fig, f"occupancy_3d_whole_{palette_name}")


def open_cutaway(points: np.ndarray, bounds: np.ndarray, palette_name="ocean"):
    """Remove top slabs and a front wedge to expose interior occupied surfaces."""
    palette = FLOOR_PALETTES[palette_name]
    points = points[(points[:, 2] >= bounds[0]) & (points[:, 2] < bounds[-1])]
    keep = np.ones(len(points), dtype=bool)
    for floor in range(3):
        floor_mask = (points[:, 2] >= bounds[floor]) & (points[:, 2] < bounds[floor + 1])
        local_top = points[:, 2] > bounds[floor + 1] - 0.48
        central = (points[:, 0] > -5.7) & (points[:, 0] < 7.8) & (points[:, 1] > 1.0)
        keep &= ~(floor_mask & local_top & central)
    # Open the near-right corner, retaining enough façade to read the building.
    keep &= ~((points[:, 0] > 5.8) & (points[:, 1] < 7.0))
    points = points[keep]
    voxel = 0.25
    lo = np.floor(points.min(axis=0) / voxel) * voxel
    hi = np.ceil(points.max(axis=0) / voxel) * voxel + voxel
    filled = grid_from_points(points, voxel, lo, hi)
    colors = np.zeros(filled.shape + (4,), dtype=np.float32)
    zc = lo[2] + (np.arange(filled.shape[2]) + 0.5) * voxel
    for floor in range(3):
        colors[:, :, (zc >= bounds[floor]) & (zc < bounds[floor + 1]), :] = rgba(palette[floor], 0.98)
    fig = plt.figure(figsize=(7.1, 7.1), facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    voxel_plot(ax, filled, colors)
    style_ax(ax, filled.shape, elev=29, azim=-58, z_scale=0.92)
    save(fig, f"occupancy_3d_open_cutaway_{palette_name}")


def floor_triptych(points: np.ndarray, bounds: np.ndarray, palette_name="ocean"):
    palette = FLOOR_PALETTES[palette_name]
    fig = plt.figure(figsize=(12.0, 4.0), facecolor="white")
    for floor in range(3):
        pts = points[(points[:, 2] >= bounds[floor]) & (points[:, 2] < bounds[floor + 1])]
        voxel = 0.25
        lo = np.floor(pts.min(axis=0) / voxel) * voxel
        hi = np.ceil(pts.max(axis=0) / voxel) * voxel + voxel
        filled = grid_from_points(pts, voxel, lo, hi)
        colors = np.zeros(filled.shape + (4,), dtype=np.float32)
        colors[filled] = rgba(palette[floor], 0.98)
        ax = fig.add_subplot(1, 3, floor + 1, projection="3d")
        voxel_plot(ax, filled, colors)
        style_ax(ax, filled.shape, elev=30, azim=-58, z_scale=1.15)
        ax.text2D(0.04, 0.91, f"L{floor + 1}", transform=ax.transAxes,
                  color=palette[floor], fontsize=15, fontweight="bold")
    fig.subplots_adjust(0, 0, 1, 1, wspace=-0.06)
    save(fig, f"occupancy_3d_floor_triptych_{palette_name}")


def remove_floor_and_roof_slabs(points: np.ndarray, bounds: np.ndarray) -> np.ndarray:
    """Remove floors, every ceiling, and the roof while preserving wall columns."""
    floors, _ = floor_data()
    slab = np.zeros(len(points), dtype=bool)
    xy = np.rint(points[:, :2] / 0.10).astype(np.int32)
    _, inverse = np.unique(xy, axis=0, return_inverse=True)
    column_count = inverse.max() + 1
    vertical_support = np.zeros(len(points), dtype=bool)
    for info in floors:
        center = float(info["floor_z_m"])
        current_slab = np.abs(points[:, 2] - center) <= 0.30
        slab |= current_slab
        below_band = (points[:, 2] >= center - 0.90) & (points[:, 2] <= center - 0.35)
        above_band = (points[:, 2] >= center + 0.35) & (points[:, 2] <= center + 0.90)
        below = np.bincount(inverse[below_band], minlength=column_count) >= 3
        above = np.bincount(inverse[above_band], minlength=column_count) >= 3
        vertical_support |= current_slab & (below & above)[inverse]

    # Explicit ceiling bands sit slightly below the next floor slab. Estimate
    # them from the repeated 2.8 m storey spacing observed in the online map.
    # Preserve a candidate only when its XY column has several occupied samples
    # immediately below it, which identifies a terminating wall/column edge.
    ceiling = np.zeros(len(points), dtype=bool)
    ceiling_support = np.zeros(len(points), dtype=bool)
    for index, info in enumerate(floors):
        if index + 1 < len(floors):
            center = float(floors[index + 1]["floor_z_m"]) - 0.35
        else:
            center = float(info["floor_z_m"]) + 2.55
        current_ceiling = np.abs(points[:, 2] - center) <= 0.35
        ceiling |= current_ceiling
        below_band = (points[:, 2] >= center - 0.95) & (points[:, 2] <= center - 0.40)
        below = np.bincount(inverse[below_band], minlength=column_count) >= 3
        ceiling_support |= current_ceiling & below[inverse]

    return points[(~slab | vertical_support) & (~ceiling | ceiling_support)]


def notched_building(points: np.ndarray, bounds: np.ndarray, color_mode: str,
                     cut_mode: str = "corner_notch", strip_slabs: bool = False):
    """Boolean-cut either a small upper notch or one vertical XY quadrant."""
    points = points[(points[:, 2] >= bounds[0]) & (points[:, 2] < bounds[-1])]
    if cut_mode == "corner_notch":
        # A smaller cuboid opening toward the current camera.
        removed = (points[:, 0] > 3.0) & (points[:, 1] < 8.0) & (points[:, 2] > 2.15)
    elif cut_mode == "quarter_cut":
        # Remove one complete vertical quadrant: half the X span times half
        # the Y span, through the full L1-L3 height = 1/4 bounding-box volume.
        midpoint = 0.5 * (points.min(axis=0) + points.max(axis=0))
        removed = (points[:, 0] > midpoint[0]) & (points[:, 1] < midpoint[1])
    else:
        raise ValueError(cut_mode)
    points = points[~removed]
    if strip_slabs:
        points = remove_floor_and_roof_slabs(points, bounds)
    voxel = 0.22
    lo = np.floor(points.min(axis=0) / voxel) * voxel
    hi = np.ceil(points.max(axis=0) / voxel) * voxel + voxel
    filled = grid_from_points(points, voxel, lo, hi)
    colors = np.zeros(filled.shape + (4,), dtype=np.float32)
    zc = lo[2] + (np.arange(filled.shape[2]) + 0.5) * voxel

    if color_mode == "height":
        normalized = np.clip((zc - bounds[0]) / (bounds[-1] - bounds[0]), 0.0, 1.0)
        for iz, value in enumerate(normalized):
            colors[:, :, iz, :] = HEIGHT_CMAP(value)
    elif color_mode == "floor_gradient":
        bases = FLOOR_PALETTES["ocean"]
        for floor in range(3):
            mask_z = (zc >= bounds[floor]) & (zc < bounds[floor + 1])
            local = np.clip((zc[mask_z] - bounds[floor]) /
                            (bounds[floor + 1] - bounds[floor]), 0.0, 1.0)
            base = np.asarray(mcolors.to_rgb(bases[floor]))
            dark = base * 0.68
            light = 1.0 - (1.0 - base) * 0.72
            for iz, value in zip(np.where(mask_z)[0], local):
                colors[:, :, iz, :3] = dark * (1.0 - value) + light * value
                colors[:, :, iz, 3] = 1.0
    else:
        raise ValueError(color_mode)

    fig = plt.figure(figsize=(7.1, 7.1), facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    voxel_plot(ax, filled, colors, edge=(1, 1, 1, 0.08), linewidth=0.045)
    style_ax(ax, filled.shape, elev=27, azim=-57, z_scale=0.90)
    suffix = "_no_slabs" if strip_slabs else ""
    save(fig, f"occupancy_3d_{cut_mode}{suffix}_{color_mode}")


def local_state_cutaway(palette_name: str):
    """Render a bounded L2 volume where all three occupancy states remain legible."""
    # A representative observed L2 room/corridor junction, selected in map coordinates.
    lo = np.asarray([-1.8, 7.0, 2.35], dtype=float)
    hi = np.asarray([4.8, 13.2, 5.25], dtype=float)
    voxel = 0.20
    state_points = {
        name: load_pcd_crop(EXPORT / f"map_{name}.pcd", lo, hi)
        for name in ("unknown", "free", "occupied")
    }
    grids = {name: grid_from_points(value, voxel, lo, hi) for name, value in state_points.items()}
    # Planner semantics resolve collisions conservatively: OCCUPIED > FREE > UNKNOWN.
    grids["free"] &= ~grids["occupied"]
    grids["unknown"] &= ~(grids["occupied"] | grids["free"])

    # Cut away the front and top portions of UNKNOWN so it provides context, not a solid box.
    shape = grids["occupied"].shape
    xx, yy, zz = np.indices(shape)
    unknown_shell = grids["unknown"] & ((yy >= shape[1] * 0.50) | (xx <= shape[0] * 0.25))
    unknown_shell &= zz < shape[2] * 0.86
    shown = grids["occupied"] | grids["free"] | unknown_shell
    colors = np.zeros(shape + (4,), dtype=np.float32)
    pal = STATE_PALETTES[palette_name]
    colors[unknown_shell] = rgba(pal["unknown"], 0.16)
    colors[grids["free"]] = rgba(pal["free"], 0.33)
    colors[grids["occupied"]] = rgba(pal["occupied"], 1.0)

    fig = plt.figure(figsize=(7.0, 6.2), facecolor="white")
    ax = fig.add_subplot(111, projection="3d")
    voxel_plot(ax, shown, colors, edge=(1, 1, 1, 0.07), linewidth=0.035)
    style_ax(ax, shape, elev=27, azim=-61, z_scale=1.05)
    save(fig, f"occupancy_3d_local_states_{palette_name}")


def curate():
    selected = (
        "occupancy_3d_whole_ocean",
        "occupancy_3d_whole_berry",
        "occupancy_3d_open_cutaway_ocean",
        "occupancy_3d_floor_triptych_ocean",
        "occupancy_3d_corner_notch_height",
        "occupancy_3d_corner_notch_floor_gradient",
        "occupancy_3d_quarter_cut_height",
        "occupancy_3d_quarter_cut_floor_gradient",
        "occupancy_3d_quarter_cut_no_slabs_height",
        "occupancy_3d_quarter_cut_no_slabs_floor_gradient",
        "occupancy_3d_local_states_cyan",
        "occupancy_3d_local_states_warm",
    )
    SELECTED_OUT.mkdir(parents=True, exist_ok=True)
    for old in SELECTED_OUT.glob("*"):
        if old.is_file():
            old.unlink()
    for stem in selected:
        for suffix in (".png", ".pdf"):
            shutil.copy2(CANDIDATE_OUT / f"{stem}{suffix}", SELECTED_OUT / f"{stem}{suffix}")
    (SELECTED_OUT / "README.md").write_text(
        "# Stage-1 3D voxel occupancy assets\n\n"
        "These panels use the real FALCON voxel map for floors L1-L3. Whole-building "
        "panels render OCCUPIED voxels because full FREE/UNKNOWN volumes occlude the "
        "building. Local-state panels explicitly show OCCUPIED, FREE, and UNKNOWN.\n\n"
        "- `occupancy_3d_whole_ocean`: primary blue/green/orange floor palette.\n"
        "- `occupancy_3d_whole_berry`: cooler alternative floor palette.\n"
        "- `occupancy_3d_open_cutaway_ocean`: roof/front cutaway exposing interiors.\n"
        "- `occupancy_3d_floor_triptych_ocean`: isolated L1-L3 comparison.\n"
        "- `occupancy_3d_corner_notch_height`: cuboid corner cut with continuous z color.\n"
        "- `occupancy_3d_corner_notch_floor_gradient`: same cut with per-floor gradients.\n"
        "- `occupancy_3d_quarter_cut_height`: one full vertical quadrant removed, z gradient.\n"
        "- `occupancy_3d_quarter_cut_floor_gradient`: quarter cut with per-floor gradients.\n"
        "- `occupancy_3d_quarter_cut_no_slabs_height`: quarter cut without floors/ceilings/roof.\n"
        "- `occupancy_3d_quarter_cut_no_slabs_floor_gradient`: open per-floor-gradient variant.\n"
        "- `occupancy_3d_local_states_cyan`: dark OCCUPIED, cyan FREE, gray UNKNOWN.\n"
        "- `occupancy_3d_local_states_warm`: dark OCCUPIED, amber FREE, gray UNKNOWN.\n\n"
        "The source grid resolution is 0.10 m. Whole-building rendering aggregates "
        "voxels to 0.30 m, and detailed/cutaway panels to 0.20-0.25 m, solely for "
        "legibility and manageable paper file size. No z coordinate is flattened.\n",
        encoding="utf-8",
    )


def main():
    occupied = load_pcd(EXPORT / "map_occupied.pcd")
    _, bounds = floor_data()
    for palette_name in FLOOR_PALETTES:
        whole_building(occupied, bounds, palette_name)
    open_cutaway(occupied, bounds)
    floor_triptych(occupied, bounds)
    notched_building(occupied, bounds, "height")
    notched_building(occupied, bounds, "floor_gradient")
    notched_building(occupied, bounds, "height", "quarter_cut")
    notched_building(occupied, bounds, "floor_gradient", "quarter_cut")
    notched_building(occupied, bounds, "height", "quarter_cut", strip_slabs=True)
    notched_building(occupied, bounds, "floor_gradient", "quarter_cut", strip_slabs=True)
    for palette_name in STATE_PALETTES:
        local_state_cutaway(palette_name)
    curate()
    print(f"generated candidates: {CANDIDATE_OUT}")
    print(f"curated assets:      {SELECTED_OUT}")


if __name__ == "__main__":
    main()
