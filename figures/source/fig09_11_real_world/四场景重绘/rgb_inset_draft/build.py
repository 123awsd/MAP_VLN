"""Three-row Fig. 9 draft; original recordings remain unmodified.

RGB panels show the same mission's reviewed target observations, not a
pixel magnification of the third-person photo or precisely synchronized frames.
"""
import ast
import json
import subprocess
import sys
from pathlib import Path

import fitz
from PIL import Image

BASE = Path(__file__).resolve().parent
SOURCE = BASE.parent
ROOT = next(p for p in BASE.parents if (p / '项目素材').is_dir())
FF = '/home/uav/.local/lib/python3.13/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
ASSETS = BASE / 'assets'
ASSETS.mkdir(exist_ok=True)
ENHANCED = '--enhanced' in sys.argv
SUFFIX = '_enhanced' if ENHANCED else ''
LABELS = {(0, 1): 'Electric steamer', (0, 2): 'Cup', (0, 5): 'Bottled water',
          (1, 1): 'Cup', (1, 4): 'Basketball', (1, 5): 'Trash can',
          (2, 5): 'Backpack'}
# IEEEtran journal text width is 43 pc = 516 TeX pt. At the manuscript's
# 0.94\textwidth placement, this gives the same final 8 pt size as the caption.
INSTRUCTION_SIZE = 8 * 780 / (0.94 * 516)
INSTRUCTION_LEADING = INSTRUCTION_SIZE * 1.18
# Reviewed target bounds in the full original inset, normalized to its dimensions.
DETAIL_BOXES = {(0, 1): (.21, .24, .77, .91),
                (0, 2): (.22, .15, .43, .42),
                (0, 5): (.22, .24, .74, .94),
                (1, 1): (.27, .30, .46, .82),
                (1, 4): (.28, .42, .59, .85),
                (1, 5): (.14, .25, .61, .94),
                (2, 5): (.30, .05, .81, .84)}
DISPLAY_CROPS = {(0, 2): (.08, .04, .58, .54)}

def display_crop(key, photo):
    if key not in DISPLAY_CROPS:
        return photo
    im = Image.open(photo)
    a, b, c, d = DISPLAY_CROPS[key]
    out = ASSETS / f'display_{key[0]}_{key[1]}{SUFFIX}.png'
    im.crop((round(a*im.width), round(b*im.height), round(c*im.width), round(d*im.height))).save(out)
    return out

# Read the established layout without executing or overwriting its renderer.
settings = {}
for node in ast.parse((SOURCE / 'create_fig8_style.py').read_text()).body:
    if isinstance(node, ast.Assign):
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in {'rows', 'lefts', 'instructions'}:
                settings[target.id] = ast.literal_eval(node.value)

# User correction: the pool-table mission targets the backpack, not a ball.
settings['instructions'][0] = [
    '"First check whether the **electric steamer** in the kitchen is on, then check whether there is any **bottled water** left under the bedroom table.',
    'Check whether the **cup** is on the living-room refrigerator; if not found, look for it on the bedroom table."',
]
settings['instructions'][1] = [
    '"Please see whether my **cup** is still on the lounge table and my **basketball** is still under the chair in the activity room;',
    'also see whether the **trash can** next to the fan in the storage room is full."',
]
settings['instructions'][2] = ['"Find the **backpack** on the pool table in the recreation room."']
# Give both tiny cup regions breathing room in the third-person photos.
settings['rows'][0][3][2][2][:] = [(213, 121, 242, 151)]
settings['rows'][1][3][1][2][:] = [(102, 92, 136, 142)]

# Crop coordinates are in the original 1280 x 720 RGB recordings.
# These are photographic crops, with no synthetic pixels or detection overlays.
specs = [
    (0, 1, '夹山漾宿舍4任务', 27.1, (560, 60, 1000, 390), 'left', 'electric steamer in kitchen'),
    (0, 2, '夹山漾宿舍4任务', 37.7, (400, 180, 880, 540), 'right', 'cup on refrigerator'),
    (0, 5, '夹山漾宿舍4任务', 65.9, (500, 365, 940, 695), 'left', 'bottled water below table'),
    (1, 1, '实验室3任务版本', 44.0, (450, 20, 850, 320), 'right', 'cup on table'),
    (1, 4, '实验室3任务版本', 63.767, (170, 150, 930, 720), 'right', 'basketball below chair'),
    (1, 5, '实验室3任务版本', 69.1, (590, 210, 1150, 630), 'bottom_left', 'trash can beside fan'),
]
manifest = []
insets = {}
for row, col, mission, second, crop, side, target in specs:
    video = next((ROOT / '真机实验素材/成功场次' / mission / '01_飞行记录').glob('*/*_rgb.mp4'))
    raw = ASSETS / f'rgb_{row}_{col}_{second}.jpg'
    if not raw.exists():
        subprocess.run([FF, '-hide_banner', '-loglevel', 'error', '-ss', str(second),
                        '-i', str(video), '-frames:v', '1', '-q:v', '2', '-y', str(raw)], check=True)
    photo = ASSETS / f'inset_{row}_{col}.jpg'
    assert (crop[2]-crop[0])*3 == (crop[3]-crop[1])*4, 'Keep RGB crop at 4:3 without distortion'
    Image.open(raw).crop(crop).save(photo, quality=97, subsampling=0)
    enhanced = ASSETS / f'enhanced_{row}_{col}.png'
    if ENHANCED and enhanced.exists():
        photo = enhanced
    photo = display_crop((row, col), photo)
    brightened = ASSETS / f'brightened_{row}_{col}.png'
    if ENHANCED and brightened.exists():
        photo = brightened
    local_bright = ASSETS / f'local_bright_{row}_{col}.png'
    if ENHANCED and local_bright.exists():
        photo = local_bright
    insets[row, col] = (photo, side)
    manifest.append(dict(row=row+1, column=col+1, target=target,
                         rgb_video=str(video.relative_to(ROOT)), rgb_time_s=second,
                         crop_xyxy=crop, corner=side,
                         correspondence='Same mission and target; independently selected observation, not exact time synchronization.'))
    manifest[-1]['display_label'] = LABELS[row, col]
    manifest[-1]['ai_enhanced'] = bool(ENHANCED and (enhanced.exists() or brightened.exists()))
    manifest[-1]['brightness_balanced'] = bool(ENHANCED and brightened.exists())
    manifest[-1]['object_only_brightness'] = bool(ENHANCED and local_bright.exists())
    manifest[-1]['display_asset'] = str(photo.relative_to(BASE))
    manifest[-1]['additional_display_crop_normalized'] = DISPLAY_CROPS.get((row, col))
    if manifest[-1]['ai_enhanced']:
        manifest[-1]['processing'] = 'Built-in imagegen restoration preview. Source crop retained. Generated fine detail is not validated experimental evidence.'
    if manifest[-1]['brightness_balanced']:
        manifest[-1]['brightness_processing'] = 'Built-in imagegen exposure-only edit request, approximately +0.5 to +0.7 EV with highlight rolloff. Original crop and prior edit retained.'
    if manifest[-1]['object_only_brightness']:
        manifest[-1]['brightness_processing'] = 'Gamma lift on source RGB through imagegen segmentation alpha only; generated cutout RGB discarded. Background outside mask unchanged. See local_brightness_manifest.json.'

# User-supplied original photograph, cropped without perspective synthesis.
reference = ASSETS / 'backpack_reference_original.jpg'
reference_crop = (280, 170, 1480, 1070)
backpack = ASSETS / 'backpack_reference_inset.jpg'
Image.open(reference).crop(reference_crop).save(backpack, quality=97, subsampling=0)
insets[2, 5] = (backpack, 'left')
if ENHANCED and (ASSETS / 'local_bright_2_5.png').exists():
    insets[2, 5] = (ASSETS / 'local_bright_2_5.png', 'left')
# Manually reviewed backpack region in the existing 480 x 270 reference frame.
settings['rows'][2][3][5][2].append((235, 91, 266, 127))
manifest.append(dict(row=3, column=6, target='backpack on pool table',
                     source_type='user-supplied reference photograph, not recorded onboard RGB',
                     original_source='/home/uav/桌面/7e972e6f8d63fe03f470ce949d76030c.jpg',
                     local_source='assets/backpack_reference_original.jpg',
                     crop_xyxy=reference_crop, corner='left',
                     processing='Crop only; no image generation, perspective reconstruction, or object modification.',
                     third_person_roi_480x270=(235, 91, 266, 127)))
manifest[-1]['display_label'] = 'Backpack'
manifest[-1]['ai_enhanced'] = False
manifest[-1]['object_only_brightness'] = bool(ENHANCED and (ASSETS / 'local_bright_2_5.png').exists())
if manifest[-1]['object_only_brightness']:
    manifest[-1]['processing'] = 'Original reference-photo crop with object-only gamma lift through generated alpha mask; no generated RGB pixels used. Background outside mask unchanged.'

doc = fitz.open()
page = doc.new_page(width=780, height=400)
y = 10
import re

def instruction(line, baseline):
    parts = re.split(r'(\*\*.*?\*\*)', line)
    runs = [(p[2:-2], 'tibo') if p.startswith('**') else (p, 'tiro') for p in parts if p]
    size = INSTRUCTION_SIZE
    x = (780 - sum(fitz.get_text_length(t, fontname=f, fontsize=size) for t, f in runs)) / 2
    for text, font in runs:
        page.insert_text((x, baseline), text, fontname=font, fontsize=size)
        x += fitz.get_text_length(text, fontname=font, fontsize=size)

for ri, (_, _, _, panels) in enumerate(settings['rows'][:3]):
    for line in settings['instructions'][ri]:
        instruction(line, y + INSTRUCTION_SIZE)
        y += INSTRUCTION_LEADING
    y += 4
    for j, (t, (cx, cy, r), boxes) in enumerate(panels):
        x, w, h = 12 + j * 127, 120, 90
        left = settings['lefts'][ri][j]
        scale = w / 360
        photo = SOURCE / 'frames' / f'compact_layout_{ri}_{j}_{t}_{left}.jpg'
        assert photo.exists(), photo
        page.insert_image(fitz.Rect(x, y, x+w, y+h), filename=str(photo))
        shape = page.new_shape()
        shape.draw_circle(fitz.Point(x+(cx-left)*scale, y+cy*scale), r*scale)
        shape.finish(color=(1, .94, 0), width=1.4, dashes='[4 2] 0')
        shape.commit()
        for a, b, c, d in boxes:
            roi = fitz.Rect(x+(a-left)*scale, y+b*scale, x+(c-left)*scale, y+d*scale)
            page.draw_rect(roi, color=(1, 1, 1), width=.8, dashes='[2.5 1.8] 0')
        if (ri, j) in insets:
            photo, side = insets[ri, j]
            iw, ih = (57, 42.75) if LABELS[ri, j] == 'Cup' else (52, 39)
            # Keep the complete long names at 6 pt on one line inside the view.
            if LABELS[ri, j] == 'Electric steamer':
                iw, ih = 80, 60
            elif LABELS[ri, j] == 'Bottled water':
                iw, ih = 66, 49.5
            ix = x+3 if side in ('left', 'bottom_left') else x+w-iw-3
            iy = y+h-ih-3 if side == 'bottom_left' else y+3
            inset = fitz.Rect(ix, iy, ix+iw, iy+ih)
            # Two matched dashed links make the cross-view detail relation clear.
            a, b, c, d = boxes[0]
            roi = fitz.Rect(x+(a-left)*scale, y+b*scale, x+(c-left)*scale, y+d*scale)
            if inset.y0 >= roi.y1:
                links = [(inset.tl, roi.bl), (inset.tr, roi.br)]
            elif inset.y1 <= roi.y0:
                links = [(inset.bl, roi.tl), (inset.br, roi.tr)]
            elif (inset.x0+inset.x1)/2 > (roi.x0+roi.x1)/2:
                links = [(inset.tl, roi.tr), (inset.bl, roi.br)]
            else:
                links = [(inset.tr, roi.tl), (inset.br, roi.bl)]
            for anchor, target in links:
                page.draw_line(anchor, target, color=(1, 1, 1), width=.65, dashes='[2.5 1.8] 0')
            page.insert_image(inset, filename=str(photo), keep_proportion=False)
            outline = ASSETS / f'outline_{ri}_{j}.png'
            if ENHANCED and outline.exists():
                page.insert_image(inset, filename=str(outline), keep_proportion=False)
            # Put full target names inside the inset, without a text plate.
            label = LABELS[ri, j]
            a, b, c, d = DETAIL_BOXES[ri, j]
            ca, cb, cc, cd = DISPLAY_CROPS.get((ri, j), (0, 0, 1, 1))
            detail = fitz.Rect(ix+(a-ca)/(cc-ca)*iw, inset.y0+(b-cb)/(cd-cb)*ih,
                               ix+(c-ca)/(cc-ca)*iw, inset.y0+(d-cb)/(cd-cb)*ih)
            label_size = INSTRUCTION_SIZE * 6 / 8  # 6 TeX pt at manuscript placement
            label_width = fitz.get_text_length(label, fontname='hebo', fontsize=label_size)
            assert label_width+3 <= iw, f'Label does not fit inside inset: {label}'
            label_x = ix+1.5
            label_y = inset.y0+label_size+1.5
            label_at = (label_x, label_y)
            page.insert_text(label_at, label, fontname='hebo', fontsize=label_size,
                             color=(0, 0, 0), fill=(1, .94, .30), render_mode=2, border_width=.10)
            page.insert_text(label_at, label, fontname='hebo', fontsize=label_size,
                             color=(1, .94, .30))
            page.draw_rect(inset, color=(1, 1, 1), width=.8, dashes='[2.5 1.8] 0')
    y += 94
page.set_cropbox(fitz.Rect(0, 0, 780, y))
for entry in manifest:
    key = f"{entry['row']-1}_{entry['column']-1}"
    entry['silhouette_outline'] = bool(ENHANCED and (ASSETS / f'outline_{key}.png').exists())
    entry['silhouette_outline_color'] = '#80F0FF' if entry['silhouette_outline'] else None
doc.save(BASE / f'fig9_three_rows_rgb_insets{SUFFIX}.pdf', deflate=True)
page.get_pixmap(matrix=fitz.Matrix(3, 3), alpha=False).save(BASE / f'fig9_three_rows_rgb_insets{SUFFIX}.png')
(BASE / f'source_manifest{SUFFIX}.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
print(BASE / f'fig9_three_rows_rgb_insets{SUFFIX}.pdf')
