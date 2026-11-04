"""Photograph-led teaser, informed by OnFly, AirHunt and SPF figure studies."""
from pathlib import Path
import base64
import html
import io
import json
import shutil

import cairo
import gi
gi.require_version('Rsvg', '2.0')
from gi.repository import Rsvg
import numpy as np
from PIL import Image, ImageFont

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
ASSETS = HERE / 'assets'
PREVIOUS = HERE.parent / 'trajectory_photo_draft/assets'
W, H = 3600, 2140
PHYSICAL_WIDTH_MM = 180
PINK = '#CC315C'
ORANGE = '#DD761D'
INK = '#202427'
GREY = '#68717A'
scene_name = 'scene_az-15_el55'
scene = json.loads((ASSETS / f'{scene_name}.json').read_text())

# These are real recordings. Crops preserve the photographic scene and its UAV.
hero_source = ROOT / '画图/主图/paper_figure/assets/rgb_033.jpg'
hero_asset = ASSETS / 'hero_real_flight_033.jpg'
if not hero_asset.exists():
    Image.open(hero_source).crop((0, 500, 3840, 1940)).save(hero_asset, quality=97)
for filename in ['recovery_found.jpg', 'task_2_toolbox.jpg', 'task_3_microwave.jpg', 'task_4_cup.jpg']:
    if not (ASSETS / filename).exists():
        shutil.copyfile(PREVIOUS / filename, ASSETS / filename)
text_boxes = []
parts = [f'''<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink"
    width="180mm" height="{H * .05}mm" viewBox="0 0 {W} {H}">
    <title>Real-world multi-task inspection with conditional recovery</title>
    <desc>A real third-person flight photograph, a measured RGB-D reconstruction
    with recorded EKF trajectory, and four target observations. No room labels.</desc>
    <defs>
      <linearGradient id="map_fade" x1="0" x2="1">
        <stop offset="0" stop-color="white" stop-opacity="0"/>
        <stop offset=".12" stop-color="white" stop-opacity=".93"/>
        <stop offset="1" stop-color="white" stop-opacity=".98"/>
      </linearGradient>
      <linearGradient id="hero_bottom" x1="0" y1="0" x2="0" y2="1">
        <stop offset="0" stop-color="white" stop-opacity="0"/>
        <stop offset="1" stop-color="white" stop-opacity=".93"/>
      </linearGradient>
      <linearGradient id="map_vertical_fade" x1="0" y1="0" x2="0" y2="1">
        <stop offset="0" stop-color="black"/>
        <stop offset=".13" stop-color="white"/>
        <stop offset=".82" stop-color="white"/>
        <stop offset="1" stop-color="black"/>
      </linearGradient>
      <mask id="map_background_mask" maskUnits="userSpaceOnUse" x="1610" y="90" width="1990" height="1150">
        <rect x="1610" y="90" width="1990" height="1150" fill="url(#map_vertical_fade)"/>
      </mask>
    </defs>
    <rect width="3600" height="2140" fill="white"/>''']


def text(x, baseline, value, size=48, color=INK, bold=False, serif=False, italic=False, anchor='start'):
    family = 'Liberation Serif' if serif else 'Liberation Sans'
    style = 'Bold' if bold else 'Italic' if italic else 'Regular'
    font_name = 'LiberationSerif' if serif else 'LiberationSans'
    font = ImageFont.truetype(f'/usr/share/fonts/truetype/liberation/{font_name}-{style}.ttf', size)
    width = font.getlength(value)
    left = x - (width / 2 if anchor == 'middle' else width if anchor == 'end' else 0)
    assert 0 <= left and left + width <= W + 1, (value, left, width)
    assert size <= baseline <= H, value
    text_boxes.append({'text': value, 'x': left, 'y': baseline-size, 'width': width, 'height': size})
    parts.append(f'<text x="{x}" y="{baseline}" fill="{color}" font-family="{family}" font-size="{size}" '
                 f'font-weight="{"bold" if bold else "normal"}" font-style="{"italic" if italic else "normal"}" '
                 f'text-anchor="{anchor}">{html.escape(value)}</text>')


def rect(x, y, width, height, fill='none', stroke='none', sw=1, radius=0, opacity=1):
    parts.append(f'<rect x="{x}" y="{y}" width="{width}" height="{height}" rx="{radius}" '
                 f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}" opacity="{opacity}"/>')


def line(points, color, width=6, dash=None):
    coordinates = ' '.join(f'{x:.2f},{y:.2f}' for x,y in points)
    extra = f' stroke-dasharray="{dash}"' if dash else ''
    parts.append(f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="{width}" '
                 f'stroke-linecap="round" stroke-linejoin="round"{extra}/>')


def circle(x, y, radius, fill, stroke='white', sw=3):
    parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')


def badge(x, y, value, color=PINK, radius=29):
    circle(x, y, radius, color, 'white', 4)
    text(x, y+16, value, 44, 'white', True, anchor='middle')


def image(path, x, y, width, height, fit='xMidYMid slice'):
    # Embed at print resolution, while leaving the source crops untouched.
    source = Image.open(path)
    source.thumbnail((round(width*1.5), round(height*1.5)), Image.Resampling.LANCZOS)
    stream = io.BytesIO()
    fmt = 'PNG' if source.mode == 'RGBA' or path.suffix.lower()=='.png' else 'JPEG'
    source.save(stream, format=fmt, **({'quality':96} if fmt == 'JPEG' else {}))
    encoded = base64.b64encode(stream.getvalue()).decode()
    parts.append(f'<image x="{x}" y="{y}" width="{width}" height="{height}" preserveAspectRatio="{fit}" '
                 f'xlink:href="data:image/{fmt.lower()};base64,{encoded}"/>')


# One actual scene anchors the composition. No cross-frame drone overlays.
image(hero_asset, 0, 0, W, 1350)
rect(0, 1120, W, 230, 'url(#hero_bottom)')

# The abridged instruction stays inside the scene; no separate logo/title block.
rect(62, 52, 1680, 207, 'white', radius=5, opacity=.94)
text(98, 127, 'Check the laptop, toolbox, microwave, and cup.', 58, INK, serif=True)
text(98, 207, 'If the laptop is absent, find it.', 65, INK, bold=True, serif=True)

# Integrate the reconstruction as a supporting inset, with a soft white edge.
map_x, map_y, map_w, map_h = 1770, 215, 1800, 828
parts.append('<rect x="1610" y="90" width="1990" height="1150" fill="url(#map_fade)" mask="url(#map_background_mask)"/>')
text(1950, 194, 'Recorded multi-room flight', 46, INK)
image(ASSETS / f'{scene_name}_transparent.png', map_x, map_y, map_w, map_h, fit='none')
ratio = map_w / scene['size'][0]
project = lambda uv: np.asarray(uv) * ratio + [map_x, map_y]
route = project(scene['route_pixels'])
line(route, 'white', 16)
line(route, PINK, 8)
times = np.array(scene['route_bag_seconds'])
begin, end = scene['recovery_bag_seconds']
mask = (times >= begin) & (times <= end)
line(route[mask], 'white', 16)
line(route[mask], ORANGE, 8)
for marker in scene['markers']:
    x, y = project(marker['pixel'])
    if marker['id'] == 'A':
        circle(x,y,12,ORANGE,'white',3)
        line([(x-5,y-14),(x-30,y-80)], ORANGE, 3)
        badge(x-34,y-100,'A',ORANGE,26)
    else:
        badge(x,y,marker['id'])
circle(*route[0], 12, '#21896D', 'white', 3)
legend_y = 1095
line([(2630,legend_y-14),(2692,legend_y-14)], PINK, 7)
text(2710,legend_y,'Flight',38,GREY)
line([(2910,legend_y-14),(2972,legend_y-14)], ORANGE, 7)
text(2990,legend_y,'Recovery',38,GREY)
bar_width=scene['scale_pixels_per_meter']*ratio*5
line([(3370-bar_width,legend_y-14),(3370,legend_y-14)], GREY, 3)
text(3394,legend_y,'5 m',36,GREY)

# The original table and real flying UAV stay visible in the main photograph.
rect(62,1169,985,90,'white',radius=3,opacity=.95)
badge(110,1214,'A',ORANGE,27)
text(160,1233,'Laptop absent at the initial check',51,INK)

# A single aligned observation strip completes the multi-task story.
photo_specs = [
    ('recovery_found.jpg','1','Laptop found','Onboard view'),
    ('task_2_toolbox.jpg','2','Toolbox',''),
    ('task_3_microwave.jpg','3','Microwave',''),
    ('task_4_cup.jpg','4','Cup',''),
]
for index,(name,number,label,view) in enumerate(photo_specs):
    x,y,width,height = 45+index*890,1396,840,588
    image(ASSETS / name,x,y,width,height)
    badge(x+44,y+47,number,PINK,29)
    if view:
        rect(x+width-299,y+16,283,62,'white',radius=2,opacity=.96)
        text(x+width-158,y+61,view,39,INK,bold=True,anchor='middle')
    line([(x,2047),(x+64,2047)],PINK,6)
    text(x+83,2067,label,56,INK,bold=True)

parts.append('</svg>')
svg_path=HERE/'main_figure.svg'
svg_path.write_text('\n'.join(parts))
handle=Rsvg.Handle.new_from_file(str(svg_path))
viewport=Rsvg.Rectangle();viewport.x=0;viewport.y=0;viewport.width=W;viewport.height=H
surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H)
assert handle.render_document(cairo.Context(surface),viewport)
surface.write_to_png(str(HERE/'main_figure.png'));surface.finish()
png=Image.open(HERE/'main_figure.png').convert('RGB')
png.save(HERE/'main_figure.png',dpi=(508,508))
png.resize((1800,1070),Image.Resampling.LANCZOS).save(HERE/'preview.jpg',quality=96)
point_per_pixel=.05*72/25.4
surface=cairo.PDFSurface(str(HERE/'main_figure.pdf'),W*point_per_pixel,H*point_per_pixel)
context=cairo.Context(surface);context.scale(point_per_pixel,point_per_pixel)
assert handle.render_document(context,viewport)
context.show_page();surface.finish()
(HERE/'layout_audit.json').write_text(json.dumps({'size':[W,H],'text':text_boxes},indent=2))
print('Built photograph-led teaser: SVG / PNG / PDF / preview.')
