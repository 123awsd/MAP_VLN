"""Build the editable paper figure from the prepared, portable assets.

Usage: /usr/bin/python3 build.py
Requires the system Pillow, NumPy, PyGObject/Rsvg and Cairo packages.
"""
from pathlib import Path
import base64
import html
import json
import math

import cairo
import gi
gi.require_version("Rsvg", "2.0")
from gi.repository import Rsvg
import numpy as np
from PIL import Image, ImageFont


HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"
OUT = HERE / "no_room_labels"
OUT.mkdir(exist_ok=True)
W, H = 3600, 2320
PHYSICAL_WIDTH_MM = 180
MM_PER_PIXEL = PHYSICAL_WIDTH_MM / W
NAVY = "#19384F"
BLUE = "#2678B5"
ORANGE = "#D57527"
GREEN = "#258168"
MUTED = "#5E6F7B"
FONT = "Liberation Sans"
parts = [f'''<svg xmlns="http://www.w3.org/2000/svg"
    xmlns:xlink="http://www.w3.org/1999/xlink"
    width="{PHYSICAL_WIDTH_MM}mm" height="{H * MM_PER_PIXEL}mm"
    viewBox="0 0 {W} {H}">
    <title>LogicFly: real-world multi-room execution and conditional search</title>
    <desc>Measured structural map with recorded onboard EKF flight trajectory,
    four third-person photographs, and a three-step laptop recovery sequence.</desc>
    <rect width="3600" height="2320" fill="white"/>
    <defs><clipPath id="map_clip"><rect x="70" y="486" width="2140" height="955"/></clipPath></defs>''']
text_bounds = []


def text(x, y, content, size=50, color=NAVY, weight="normal", anchor="start"):
    """Coordinates use SVG baselines; record bounds to catch overflow."""
    face = "/usr/share/fonts/truetype/liberation/LiberationSans-"
    face += "Bold.ttf" if weight == "bold" else "Regular.ttf"
    font = ImageFont.truetype(face, size)
    advance = font.getlength(content)
    left = x - (advance / 2 if anchor == "middle" else advance if anchor == "end" else 0)
    top = y - size
    assert left >= 0 and left + advance <= W + 1, (content, left, advance)
    assert top >= 0 and y <= H, (content, y)
    text_bounds.append({"text": content, "x": left, "y": top, "width": advance, "height": size})
    parts.append(f'<text x="{x:.3f}" y="{y:.3f}" font-family="{FONT}" font-size="{size}" '
                 f'font-weight="{weight}" fill="{color}" text-anchor="{anchor}">{html.escape(content)}</text>')


def rect(x, y, w, h, fill="none", stroke="none", stroke_width=1, rx=0):
    parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" '
                 f'fill="{fill}" stroke="{stroke}" stroke-width="{stroke_width}"/>')


def line(points, color, width=6, dash=None):
    coords = " ".join(f"{x:.3f},{y:.3f}" for x, y in points)
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    parts.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}" '
                 f'stroke-linecap="round" stroke-linejoin="round"{extra}/>')


def circle(x, y, radius, fill, stroke="white", stroke_width=4):
    parts.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{radius}" fill="{fill}" '
                 f'stroke="{stroke}" stroke-width="{stroke_width}"/>')


def badge(x, y, label, color=BLUE, radius=32):
    circle(x, y, radius, color)
    text(x, y + 17, label, 48, "white", "bold", "middle")


def image(path, x, y, w, h, preserve="none"):
    suffix = "jpeg" if path.suffix.lower() in (".jpg", ".jpeg") else "png"
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    parts.append(f'<image x="{x}" y="{y}" width="{w}" height="{h}" '
                 f'preserveAspectRatio="{preserve}" xlink:href="data:image/{suffix};base64,{encoded}"/>')


def arrow_tip(point, direction, color, size=22):
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    perpendicular = np.array([-direction[1], direction[0]])
    point = np.asarray(point)
    polygon = np.array([point + size * direction,
                        point - size * .70 * direction + size * .60 * perpendicular,
                        point - size * .70 * direction - size * .60 * perpendicular])
    coords = " ".join(f"{x:.3f},{y:.3f}" for x, y in polygon)
    parts.append(f'<polygon points="{coords}" fill="{color}" stroke="white" stroke-width="3" stroke-linejoin="round"/>')


# Compact instruction: no large block of prose competing with the experiment.
text(70, 104, "LogicFly", 78, NAVY, "bold")
line([(407, 48), (407, 110)], "#B6C2CC", 3)
text(445, 101, "Language-guided multi-room flight", 66, NAVY)
rect(70, 151, 3460, 158, "#F3F6F9", rx=5)
text(102, 202, "INSTRUCTION (ABRIDGED)", 36, MUTED, "bold")
text(102, 266, "Check the laptop, toolbox, microwave, and cup. If the laptop is absent, find it.", 56, NAVY)

text(70, 403, "(a) Recorded flight", 54, NAVY, "bold")
text(2290, 403, "(b) Real-world execution", 54, NAVY, "bold")

# Main spatial view, using measured structural walls and approved boundaries.
geometry = json.loads((ASSETS / "map_geometry.json").read_text())
events = json.loads((ASSETS / "flight_events.json").read_text())
route = np.loadtxt(ASSETS / "recorded_flight.csv", delimiter=",", skiprows=1)
xmin, xmax, ymin, ymax = geometry["extent_xyxy_m"]
plot_x, plot_y, plot_w, plot_h = 70, 486, 2140, 955
scale = min(plot_w / (xmax - xmin), plot_h / (ymax - ymin))
map_w, map_h = (xmax - xmin) * scale, (ymax - ymin) * scale
map_x, map_y = plot_x + (plot_w - map_w) / 2, plot_y + (plot_h - map_h) / 2


def project(xy):
    xy = np.asarray(xy)
    return np.stack([map_x + (xy[..., 0] - xmin) * scale,
                     map_y + (ymax - xy[..., 1]) * scale], axis=-1)


line([(80, 453), (142, 453)], BLUE, 9)
text(162, 468, "Flight", 44, MUTED)
line([(352, 453), (414, 453)], ORANGE, 9)
text(434, 468, "Recovery", 44, MUTED)
circle(730, 453, 13, GREEN, "white", 3)
text(756, 468, "Start", 44, MUTED)

parts.append('<g clip-path="url(#map_clip)">')
# Keep only measured structural walls as spatial context. No room segmentation,
# names, label boxes or semantic region colors are displayed in this revision.
image(ASSETS / "measured_walls.png", map_x, map_y, map_w, map_h)
path = project(route[:, 1:3])
line(path, "white", 18)
line(path, BLUE, 10)
recovery_start, recovery_end = events["recovery_bag_seconds"]
recovery_mask = (route[:, 0] >= recovery_start) & (route[:, 0] <= recovery_end)
line(path[recovery_mask], "white", 18)
line(path[recovery_mask], ORANGE, 10)
for seconds, color in [(60, BLUE), (82, ORANGE), (130, BLUE), (182, BLUE)]:
    index = np.argmin(abs(route[:, 0] - seconds))
    before, after = max(0, index - 8), min(len(path) - 1, index + 8)
    arrow_tip(path[index], path[after] - path[before], color, 19)
parts.append("</g>")

for marker in events["markers"]:
    x, y = project(marker["xyz_m"][:2])
    if marker["id"] == "start":
        circle(x, y, 19, GREEN, "white", 5)
    else:
        badge(x, y, marker["id"], ORANGE if marker["id"] == "A" else BLUE, radius=29)

# A clear metric reference, below the map and away from the flight path.
sx, sy = 100, 1478
line([(sx, sy), (sx + 5 * scale, sy)], NAVY, 5)
line([(sx, sy - 10), (sx, sy + 10)], NAVY, 4)
line([(sx + 5 * scale, sy - 10), (sx + 5 * scale, sy + 10)], NAVY, 4)
text(sx + 5 * scale / 2, sy - 23, "5 m", 42, MUTED, anchor="middle")

# Four photographs: preserve the measured flight record, enlarge the actual UAV.
photo_positions = [(2290, 451), (2930, 451), (2290, 1000), (2930, 1000)]
photo_specs = [
    ("task_1_laptop.jpg", "1", "Laptop"),
    ("task_2_toolbox.jpg", "2", "Toolbox"),
    ("task_3_microwave.jpg", "3", "Microwave"),
    ("task_4_cup.jpg", "4", "Cup"),
]
for (x, y), (name, number, label) in zip(photo_positions, photo_specs):
    image(ASSETS / name, x, y, 600, 420)
    rect(x, y, 600, 420, stroke="#C3CCD3", stroke_width=2)
    badge(x + 42, y + 44, number, BLUE, 30)
    text(x, y + 478, label, 51, NAVY, "bold")

line([(70, 1572), (3530, 1572)], "#C9D4DD", 3)
text(70, 1660, "(c) Conditional search and recovery", 54, NAVY, "bold")

# Recovery detail: independent third-person photos, then a real onboard target view.
recovery_panels = [
    (70, "recovery_absent.jpg", "A", "Laptop absent", "Lounge / third-person view", ORANGE),
    (1260, "recovery_search.jpg", "A → 1", "Search another room", "Reading room / third-person view", ORANGE),
    (2450, "recovery_found.jpg", "1", "Laptop found", "Reading room / onboard view", GREEN),
]
for x, name, key, title, subtitle, color in recovery_panels:
    y, w, h = 1707, 1080, 530
    image(ASSETS / name, x, y, w, h)
    rect(x, y, w, h, stroke="#C3CCD3", stroke_width=2)
    tag_width = 93 if len(key) == 1 else 185
    rect(x + 16, y + 16, tag_width, 69, color, rx=2)
    text(x + 16 + tag_width / 2, y + 68, key, 49, "white", "bold", "middle")
    if name == "recovery_found.jpg":
        rect(x + w - 323, y + 16, 307, 65, "white", rx=2)
        text(x + w - 170, y + 63, "Onboard view", 42, NAVY, "bold", "middle")
    # Titles occupy a clean white caption strip rather than covering image content.
    text(x, 2292, title, 49, color, "bold")

for x in [1150, 2340]:
    line([(x + 14, 1968), (x + 92, 1968)], ORANGE, 7)
    arrow_tip([x + 80, 1968], [1, 0], ORANGE, 18)

parts.append("</svg>")
svg_path = OUT / "main_figure.svg"
svg_path.write_text("\n".join(parts))

# Render the same SVG for every deliverable, so preview, PNG and PDF agree.
handle = Rsvg.Handle.new_from_file(str(svg_path))
viewport = Rsvg.Rectangle()
viewport.x, viewport.y, viewport.width, viewport.height = 0, 0, W, H
surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, W, H)
context = cairo.Context(surface)
assert handle.render_document(context, viewport)
surface.write_to_png(str(OUT / "main_figure.png"))
surface.finish()
png = Image.open(OUT / "main_figure.png").convert("RGB")
dpi = 25.4 / MM_PER_PIXEL
png.save(OUT / "main_figure.png", dpi=(dpi, dpi))
png.resize((1800, 1160), Image.Resampling.LANCZOS).save(OUT / "preview.jpg", quality=96)

point_per_pixel = MM_PER_PIXEL * 72 / 25.4
pdf = cairo.PDFSurface(str(OUT / "main_figure.pdf"), W * point_per_pixel, H * point_per_pixel)
context = cairo.Context(pdf)
context.scale(point_per_pixel, point_per_pixel)
assert handle.render_document(context, viewport)
context.show_page()
pdf.finish()

(OUT / "layout_audit.json").write_text(json.dumps({
    "canvas_pixels": [W, H], "physical_width_mm": PHYSICAL_WIDTH_MM,
    "map_scale_pixels_per_m": scale,
    "trajectory_samples_displayed": len(route),
    "minimum_text_size_px": min(v["height"] for v in text_bounds),
    "minimum_text_size_pt": min(v["height"] for v in text_bounds) * MM_PER_PIXEL * 72 / 25.4,
    "text_bounds": text_bounds,
}, indent=2))
print("Built SVG, full-resolution PNG, vector PDF, and preview from one layout.")
