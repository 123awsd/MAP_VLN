"""Prepare measured geometry and real photographs for this figure draft.

Run with /usr/bin/python3. Original recordings remain outside the paper project.
The layout builder uses only the derived assets saved next to this script.
"""
from pathlib import Path
import hashlib
import json

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[4]
ASSETS = HERE / "assets"
ASSETS.mkdir(exist_ok=True)
UPSTREAM = WORKSPACE / "画图/主图/local_vggt"
RUN = WORKSPACE / "真机实验素材/成功场次/实验室4任务加主动搜索"
ROOM_DIR = RUN / "02_地图与场景/room_review_pcd"
sources = []


def source_record(path):
    return {
        "path": str(path.relative_to(WORKSPACE)),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


photos = [
    ("task_1_laptop", 53, (350, 60, 3350, 2160)),
    ("task_2_toolbox", 69, (850, 0, 3350, 1750)),
    ("task_3_microwave", 129, (950, 450, 2950, 1850)),
    ("task_4_cup", 163, (400, 144, 3280, 2160)),
    ("recovery_absent", 31, (200, 650, 2950, 2000)),
    ("recovery_search", 47, (900, 650, 3600, 1975)),
]
for name, seconds, crop in photos:
    path = UPSTREAM / "third_person_assets" / f"rgb_{seconds:03}.jpg"
    original = Image.open(path).convert("RGB")
    assert 0 <= crop[0] < crop[2] <= original.width
    assert 0 <= crop[1] < crop[3] <= original.height
    original.crop(crop).save(ASSETS / f"{name}.jpg", quality=98)
    sources.append({
        "asset": f"assets/{name}.jpg",
        **source_record(path),
        "view": "third-person handheld follow camera",
        "encoded_video_seconds": seconds,
        "crop_xyxy_pixels": crop,
        "original_size": original.size,
    })

video = RUN / "01_飞行记录/flight_20261001_184127/flight_20261001_184127_rgb.mp4"
cap = cv2.VideoCapture(str(video))
cap.set(cv2.CAP_PROP_POS_MSEC, 46000)
ok, frame = cap.read()
assert ok, "Cannot read the onboard laptop observation."
cap.release()
crop = (150, 170, 1250, 710)
image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
image.crop(crop).save(ASSETS / "recovery_found.jpg", quality=98)
sources.append({
    "asset": "assets/recovery_found.jpg",
    "path": str(video.relative_to(WORKSPACE)),
    "view": "onboard RGB camera",
    "encoded_video_seconds": 46.0,
    "crop_xyxy_pixels": crop,
    "original_size": image.size,
})

# Measured structural occupancy, with exactly the same world coordinates as odom.
metadata = json.loads((ROOM_DIR / "structure_grid.json").read_text())
grid = np.load(ROOM_DIR / "structure_grid.npy")
assert set(np.unique(grid)) == {-1, 0, 100}
origin = np.array(metadata["origin_xy_m"])
resolution = metadata["resolution_m"]
extent = [-44.0, 21.0, -11.0, 18.0]  # xmin, xmax, ymin, ymax; display crop only
ix0, iy0 = np.rint((np.array([extent[0], extent[2]]) - origin) / resolution).astype(int)
ix1, iy1 = np.rint((np.array([extent[1], extent[3]]) - origin) / resolution).astype(int)
selected = grid[iy0:iy1, ix0:ix1]
rgba = np.zeros((*selected.shape, 4), dtype=np.uint8)
rgba[selected == 100] = (91, 105, 117, 215)
Image.fromarray(rgba[::-1]).save(ASSETS / "measured_walls.png")
rooms = json.loads((ROOM_DIR / "rooms_approved.json").read_text())["rooms"]
rooms = [room for room in rooms if room["id"] != "L1_R06"]
(ASSETS / "map_geometry.json").write_text(json.dumps({
    "extent_xyxy_m": extent,
    "coordinate_frame": metadata["coordinate_frame"],
    "wall_resolution_m": resolution,
    "rooms": [{"id": r["id"], "name": r["name"], "polygon_xy_m": r["polygon_xy_m"]} for r in rooms],
}, indent=2))
sources.append({
    "asset": "assets/measured_walls.png",
    **source_record(ROOM_DIR / "structure_grid.npy"),
    "description": "Measured structural occupied cells; cropped in world XY and flipped for top-down display.",
    "metadata": str((ROOM_DIR / "structure_grid.json").relative_to(WORKSPACE)),
})
sources.append({"asset": "assets/map_geometry.json", **source_record(ROOM_DIR / "rooms_approved.json")})

# Use the recorded UAV body positions directly. No planned path or video-camera
# reconstruction is used as a substitute for the recorded flight trajectory.
odom_path = UPSTREAM / "odom_correction/odom.npy"
odom = np.load(odom_path)
events = json.loads((UPSTREAM / "odom_correction/events.json").read_text())
t0 = events[0]["bag_sec"]
t1 = events[-2]["bag_sec"]
selected = odom[(odom[:, 0] >= t0) & (odom[:, 0] <= t1)]
draw_path = selected[::8, :4]
draw_path = np.vstack([draw_path, selected[-1, :4]])
np.savetxt(ASSETS / "recorded_flight.csv", draw_path, delimiter=",", fmt="%.8f",
           header="bag_seconds,x_m,y_m,z_m", comments="")
markers = []
for key, event_index, label in [
    ("start", 0, "Start"), ("A", 2, "Laptop absent"),
    ("1", 4, "Laptop / reading room"), ("2", 5, "Toolbox / storage room"),
    ("3", 6, "Microwave / tea room"), ("4", 7, "Cup / recreation room"),
]:
    event = events[event_index]
    row = odom[np.argmin(np.abs(odom[:, 0] - event["bag_sec"]))]
    markers.append({"id": key, "label": label, "bag_seconds": event["bag_sec"], "xyz_m": row[1:4].tolist()})
(ASSETS / "flight_events.json").write_text(json.dumps({
    "markers": markers,
    "recovery_bag_seconds": [events[2]["bag_sec"], events[4]["bag_sec"]],
    "interpretation": "Phase anchors from the flight log, associated with visual inspection/recovery events.",
}, indent=2))
sources.append({"asset": "assets/recorded_flight.csv", **source_record(odom_path),
                "topic": "/ekf_quat/ekf_odom", "position": "UAV body in recorded world frame",
                "display_sampling": "Every eighth recorded sample, plus the last sample; no smoothing or geometry changes.",
                "bag_seconds": [t0, t1]})
sources.append({"asset": "assets/flight_events.json", **source_record(UPSTREAM / "odom_correction/events.json")})

(HERE / "manifest.json").write_text(json.dumps({
    "status": "Independent layout draft; not installed into figures/final.",
    "flight": "flight_20261001_184127",
    "geometry": "Archived measured LiDAR structural grid and author-approved room boundaries.",
    "trajectory": "Recorded onboard EKF body-position estimate, not external ground truth.",
    "photo_alignment": "Semantic event matching. Third-person and onboard video timestamps are separate clocks; correspondence is approximate.",
    "instruction": "English abridgment of the archived Chinese four-task instruction; microwave clause is inspection only.",
    "image_processing": "Rectangular crop, resize and JPEG encoding only. No generative image editing.",
    "sources": sources,
}, indent=2, ensure_ascii=False))

sheet = Image.new("RGB", (1600, 1160), "white")
draw = ImageDraw.Draw(sheet)
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
for i, (name, _, _) in enumerate(photos[:4]):
    image = Image.open(ASSETS / f"{name}.jpg")
    image.thumbnail((800, 520))
    x, y = (i % 2) * 800, (i // 2) * 580
    sheet.paste(image, (x, y + 45))
    draw.text((x + 8, y + 6), name, fill="black", font=font)
sheet.save(HERE / "photo_crops.jpg", quality=95)
print(f"Prepared {len(sources)} source records and {len(draw_path)} recorded path samples.")
