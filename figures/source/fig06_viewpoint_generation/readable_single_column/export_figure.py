#!/usr/bin/env python3
"""Keep the installed scene and replace its tiny legend with vector artwork."""
from pathlib import Path
import hashlib
import json
import subprocess

import fitz
from PIL import Image, ImageChops


OUT = Path(__file__).resolve().parent
PT = 72 / 25.4
WIDTH_MM = 88
FONT_SIZE = 8.5
COLORS = [(102 / 255, 120 / 255, 136 / 255),
          (0, 125 / 255, 145 / 255),
          (217 / 255, 173 / 255, 69 / 255),
          (213 / 255, 69 / 255, 77 / 255)]
LABELS = ['Sampled viewpoints', 'Retained candidates',
          'Occluded viewpoints', 'Collision rejected']


def main():
    source = OUT / 'base_figure.pdf'
    for script in ('draw_global.py', 'compose_scene.py'):
        subprocess.run(['/usr/bin/env', 'python', str(OUT / script)], check=True)
    scene_path = OUT / 'scene_large_icons.png'
    image = Image.open(scene_path).convert('RGB')
    difference = ImageChops.difference(image, Image.new('RGB', image.size, 'white'))
    bounds = difference.convert('L').point(lambda x: 255 if x > 8 else 0).getbbox()
    assert bounds is not None
    clip = fitz.Rect(max(0, bounds[0] - 20), max(0, bounds[1] - 20),
                     min(image.width, bounds[2] + 20),
                     min(image.height, bounds[3] + 20))
    scene_image = image.crop(tuple(int(v) for v in clip))
    import io
    stream = io.BytesIO()
    scene_image.save(stream, format='PNG')
    width = WIDTH_MM * PT
    scene_height = clip.height * width / clip.width
    height = scene_height + 37
    result = fitz.open()
    target = result.new_page(width=width, height=height)
    target.insert_image(fitz.Rect(0, 0, width, scene_height), stream=stream.getvalue())
    font_path = subprocess.check_output(
        ['fc-match', '-f', '%{file}', 'Liberation Sans'], text=True).strip()
    font = fitz.Font(fontfile=font_path)
    target.insert_font(fontname='Legend', fontfile=font_path)
    symbol_boxes = []
    for index, (label, color) in enumerate(zip(LABELS, COLORS)):
        row, column = divmod(index, 2)
        left = column * width / 2
        center = fitz.Point(left + 9, scene_height + 11 + row * 16)
        box = fitz.Rect(center.x - 6, center.y - 5, center.x + 6, center.y + 5)
        symbol_boxes.append(list(box))
        if index == 0:
            target.draw_circle(center, 3.5, color=color, fill=color)
        elif index == 1:
            tip = fitz.Point(center.x - 6, center.y)
            plane = [fitz.Point(center.x + 2, center.y - 4),
                     fitz.Point(center.x + 6, center.y - 2.5),
                     fitz.Point(center.x + 6, center.y + 4),
                     fitz.Point(center.x + 2, center.y + 2.5)]
            for point in plane:
                target.draw_line(tip, point, color=color, width=.9)
            target.draw_polyline(plane, color=color, width=.9, closePath=True)
        elif index == 2:
            target.draw_rect(fitz.Rect(center.x - 4.5, center.y - 4.5,
                                       center.x + 4.5, center.y + 4.5),
                             color=color, width=1.4)
        else:
            for sign in (-1, 1):
                target.draw_line(fitz.Point(center.x - 4.5, center.y - sign * 4.5),
                                 fitz.Point(center.x + 4.5, center.y + sign * 4.5),
                                 color=color, width=1.7)
        text_left = left + 20
        assert text_left + font.text_length(label, fontsize=FONT_SIZE) < left + width / 2 - 2
        target.insert_text((text_left, center.y + 2.8), label, fontname='Legend',
                           fontsize=FONT_SIZE, color=(36 / 255, 52 / 255, 67 / 255))
    result.save(OUT / 'viewpoint_generation_en.pdf', garbage=4, deflate=True)
    for dpi, name in [(600, 'viewpoint_generation_en.png'), (400, 'preview.png')]:
        target.get_pixmap(dpi=dpi, alpha=False).save(OUT / name)
    (OUT / 'viewpoint_generation_en.svg').write_text(target.get_svg_image(text_as_path=False))
    (OUT / 'layout_evidence.json').write_text(json.dumps({
        'base_pdf_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'size_mm': [WIDTH_MM, height / PT],
        'source_scene_clip_pt': list(clip),
        'scene_geometry_changed': False,
        'global_icon_scale': {'dots': 1.33, 'occluded': 1.33, 'collision': 1.35, 'frustum': 1.12},
        'candidate_positions_changed': False,
        'legend_layout': 'two rows, two columns',
        'legend_font_pt': FONT_SIZE,
        'legend_symbol_boxes_pt': symbol_boxes,
        'legend_labels': LABELS,
    }, indent=2) + '\n')
    print(f'Exported {WIDTH_MM} x {height / PT:.2f} mm with {FONT_SIZE} pt vector legend.')


if __name__ == '__main__':
    main()
