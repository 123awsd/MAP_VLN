"""Render measured RGB-D geometry; keep actual recorded flight positions."""
from pathlib import Path
import json
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
UPSTREAM = ROOT / '画图/主图/local_vggt'
ASSETS = HERE / 'assets'
ASSETS.mkdir(exist_ok=True)
cloud_path = UPSTREAM / 'mapping_rgbd/mapping_rgbd.ply'
with cloud_path.open('rb') as stream:
    header = []
    while True:
        row = stream.readline()
        header.append(row)
        if row.strip() == b'end_header':
            break
    offset = stream.tell()
header = b''.join(header).decode()
count = int(next(s for s in header.splitlines() if s.startswith('element vertex')).split()[-1])
dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                  ('r', 'u1'), ('g', 'u1'), ('b', 'u1'), ('a', 'u1')])
raw = np.memmap(cloud_path, dtype=dtype, mode='r', offset=offset, shape=(count,))
xyz = np.column_stack([raw['x'], raw['y'], raw['z']])
rgb = np.column_stack([raw['r'], raw['g'], raw['b']])
# The stored viewer frame is (world x, world z, -world y).
# Remove the ceiling by a world-height display cut, without moving any points.
keep = ((xyz[:, 0] > -43) & (xyz[:, 0] < 21) &
        (xyz[:, 1] > -.25) & (xyz[:, 1] < 1.65) &
        (xyz[:, 2] > -18) & (xyz[:, 2] < 10))
xyz, rgb = xyz[keep], rgb[keep]
odom = np.load(UPSTREAM / 'odom_correction/odom.npy')
events = json.loads((UPSTREAM / 'odom_correction/events.json').read_text())
route = odom[(odom[:, 0] >= events[0]['bag_sec']) & (odom[:, 0] <= events[7]['bag_sec'])][::8]
centers = route[:, [1, 3, 2]] * [1, 1, -1]
marker_rows = [odom[np.argmin(abs(odom[:, 0] - e['bag_sec']))] for e in [events[2], events[4], events[5], events[6], events[7]]]
marker_centers = np.array([r[[1, 3, 2]] * [1, 1, -1] for r in marker_rows])


def render(azimuth, elevation, destination):
    width, height = 3000, 1380
    az, el = np.deg2rad([azimuth, elevation])
    right = np.array([np.cos(az), 0, np.sin(az)])
    up = np.array([-np.sin(az) * np.sin(el), np.cos(el), np.cos(az) * np.sin(el)])
    basis = np.stack([right, -up], axis=1)
    projected = xyz @ basis
    lo, hi = np.quantile(projected, [.001, .999], axis=0)
    scale = min((width - 130) / (hi[0] - lo[0]), (height - 100) / (hi[1] - lo[1]))
    midpoint = (lo + hi) / 2
    def project(points):
        return (points @ basis - midpoint) * scale + [width / 2, height / 2]
    uv = np.rint(project(xyz)).astype(int)
    valid = (uv[:, 0] >= 2) & (uv[:, 0] < width - 3) & (uv[:, 1] >= 2) & (uv[:, 1] < height - 3)
    pixels, colors = uv[valid], rgb[valid]
    colors = np.clip((colors.astype(float) - 128) * 1.08 + 128, 0, 255).astype('uint8')
    depth = xyz[valid] @ np.cross(right, up)
    order = np.argsort(depth)
    pixels, colors = pixels[order], colors[order]
    buffer = np.full((height, width, 3), 255, dtype='uint8')
    alpha = np.zeros((height, width), dtype='uint8')
    # Ordered splats, with the same display radius for every point.
    for dx, dy in [(0, 0), (-1, 0), (1, 0), (0, -1), (0, 1), (1, 1), (-1, -1)]:
        buffer[pixels[:, 1] + dy, pixels[:, 0] + dx] = colors
        alpha[pixels[:, 1] + dy, pixels[:, 0] + dx] = 255
    image = Image.fromarray(buffer)
    image.save(destination)
    Image.fromarray(np.dstack([buffer, alpha]), mode='RGBA').save(destination.with_name(destination.stem + '_transparent.png'))
    metadata = {
        'size': [width, height], 'azimuth_degrees': azimuth, 'elevation_degrees': elevation,
        'scale_pixels_per_meter': float(scale),
        'route_pixels': project(centers).tolist(), 'route_bag_seconds': route[:, 0].tolist(),
        'markers': [{'id': label, 'pixel': xy.tolist()} for label, xy in zip(['A', '1', '2', '3', '4'], project(marker_centers))],
        'recovery_bag_seconds': [events[2]['bag_sec'], events[4]['bag_sec']],
        'geometry_source': str(cloud_path.relative_to(ROOT)),
        'geometry_display_crop': {'world_x': [-43, 21], 'world_y': [-10, 18], 'world_z': [-.25, 1.65]},
        'trajectory_source': 'Recorded /ekf_quat/ekf_odom; body positions; same map frame; not planned path or external ground truth.',
        'geometry_method': 'Measured RGB-D unprojection with archived optical camera poses; not VGGT.',
    }
    destination.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    preview = image.copy()
    draw = ImageDraw.Draw(preview)
    line = [tuple(p) for p in project(centers)]
    draw.line(line, fill='white', width=15)
    draw.line(line, fill='#d52f59', width=8)
    for marker in metadata['markers']:
        x, y = marker['pixel']
        draw.ellipse((x-18, y-18, x+18, y+18), fill='#d52f59', outline='white', width=3)
    preview.thumbnail((1000, 460))
    return preview


sheet = Image.new('RGB', (2000, 1500), '#eeeeee')
draw = ImageDraw.Draw(sheet)
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 24)
for index, (az, el) in enumerate([(0, 55), (-15, 55), (15, 55), (0, 75), (-15, 75), (15, 75)]):
    preview = render(az, el, ASSETS / f'scene_az{az}_el{el}.png')
    x, y = index % 2 * 1000, index // 2 * 500
    sheet.paste(preview, (x, y + 40))
    draw.text((x + 20, y + 5), f'azimuth {az}, elevation {el}', font=font, fill='black')
sheet.save(HERE / 'scene_angles.jpg', quality=95)
print('Rendered six measured-scene views:', len(xyz), 'points retained.')
