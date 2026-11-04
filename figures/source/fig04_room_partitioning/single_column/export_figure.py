#!/usr/bin/env python3
"""Lay out the existing measured vector panels at readable single-column sizes."""
from pathlib import Path
import copy
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET

import fitz

OUT = Path(__file__).resolve().parent
BASE = OUT.parent / 'single_row'
PT = 72 / 25.4
WIDTH_MM, HEIGHT_MM = 88, 86
TITLE_SIZE = 8.5
TITLES = ['(a) Input point cloud', '(b) Wall extraction',
          '(c) Room core extraction', '(d) Room partitioning']
LEGEND = {'Supported space', 'Excluded', 'Interior cores', 'Unassigned'}
NS = 'http://www.w3.org/2000/svg'
INK = '#293d49'


def main():
    source_pdf = BASE / 'room_segmentation_en.pdf'
    source_svg = BASE / 'room_segmentation_en.svg'
    original = ET.parse(source_svg).getroot()
    namespace = {'s': NS}
    clips = original.findall('./s:defs/s:clipPath/s:rect', namespace)
    map_clip = next(rect for rect in clips if float(rect.get('y')) > 0)
    x, y, w, h = [float(map_clip.get(key)) for key in ('x', 'y', 'width', 'height')]
    step = float(original.get('viewBox').split()[2]) / 4
    sources = [fitz.Rect(x + (i - 1) * step, y, x + (i - 1) * step + w, y + h)
               for i in range(4)]
    document = fitz.open(source_pdf)
    page = document[0]
    for image in page.get_images():
        assert all(sources[0].contains(box) for box in page.get_image_rects(image[0]))
    spans = [span for block in page.get_text('dict')['blocks']
             for line in block.get('lines', []) for span in line['spans']]
    assert {span['text'] for span in spans} == set(TITLES) | LEGEND
    for span in spans:
        page.add_redact_annot(fitz.Rect(span['bbox']), fill=False)
    page.apply_redactions(images=0, graphics=0)
    assert not page.get_text().strip()

    font_path = subprocess.check_output(
        ['fc-match', '-f', '%{file}', 'Liberation Sans:style=Bold'], text=True).strip()
    font = fitz.Font(fontfile=font_path)
    result = fitz.open()
    target = result.new_page(width=WIDTH_MM * PT, height=HEIGHT_MM * PT)
    target.insert_font(fontname='PanelHeading', fontfile=font_path)
    destinations = []
    svg = ET.Element('{' + NS + '}svg', {
        'width': f'{WIDTH_MM * PT:.9f}pt', 'height': f'{HEIGHT_MM * PT:.9f}pt',
        'viewBox': f'0 0 {WIDTH_MM * PT:.9f} {HEIGHT_MM * PT:.9f}', 'version': '1.1'})
    for definitions in original.findall('./s:defs', namespace):
        svg.append(copy.deepcopy(definitions))
    definitions = ET.SubElement(svg, '{' + NS + '}defs')
    for i, (title, clip) in enumerate(zip(TITLES, sources)):
        row, column = [(0, 0), (0, 1), (1, 1), (1, 0)][i]
        cell_left, row_top = (2 + column * 44) * PT, (2 + row * 45) * PT
        map_height = 32 * PT
        scale = map_height / clip.height
        map_width = clip.width * scale
        left = cell_left + (40 * PT - map_width) / 2
        top = row_top + 5 * PT
        destination = fitz.Rect(left, top, left + map_width, top + map_height)
        destinations.append(destination)
        target.show_pdf_page(destination, document, 0, clip=clip)
        title_width = font.text_length(title, fontsize=TITLE_SIZE)
        assert title_width < 40 * PT
        baseline = row_top + font.ascender * TITLE_SIZE
        target.insert_text((cell_left + 20 * PT - title_width / 2, baseline), title,
                           fontname='PanelHeading', fontsize=TITLE_SIZE,
                           color=(41 / 255, 61 / 255, 73 / 255))
        heading = ET.SubElement(svg, '{' + NS + '}text', {
            'x': str(cell_left + 20 * PT), 'y': str(baseline),
            'font-family': 'Liberation Sans', 'font-size': str(TITLE_SIZE),
            'font-weight': '700', 'text-anchor': 'middle', 'fill': INK})
        heading.text = title
        clip_id = f'single_column_panel_{i}'
        clip_path = ET.SubElement(definitions, '{' + NS + '}clipPath', {'id': clip_id})
        ET.SubElement(clip_path, '{' + NS + '}rect', {
            'x': str(clip.x0), 'y': str(clip.y0),
            'width': str(clip.width), 'height': str(clip.height)})
        group = ET.SubElement(svg, '{' + NS + '}g', {
            'transform': f'translate({left} {top}) scale({scale}) translate({-clip.x0} {-clip.y0})',
            'clip-path': f'url(#{clip_id})'})
        panel = original.find(f'.//s:g[@id="axes_{i + 2}"]', namespace)
        assert panel is not None
        group.append(copy.deepcopy(panel))
    arrows = [((40, 23), (48, 23)), ((66, 40), (66, 46)),
              ((48, 68), (40, 68))]
    for start_mm, end_mm in arrows:
        start = fitz.Point(start_mm[0] * PT, start_mm[1] * PT)
        end = fitz.Point(end_mm[0] * PT, end_mm[1] * PT)
        delta = end - start
        direction = delta / abs(delta)
        base = end - direction * (1.5 * PT)
        perpendicular = fitz.Point(-direction.y, direction.x) * (.7 * PT)
        color = (112 / 255, 132 / 255, 147 / 255)
        target.draw_line(start, base, color=color, width=.7)
        triangle = [end, base + perpendicular, base - perpendicular]
        target.draw_polyline(triangle, color=color, fill=color, closePath=True)
        ET.SubElement(svg, '{' + NS + '}path', {
            'd': f'M {start.x} {start.y} L {base.x} {base.y}',
            'fill': 'none', 'stroke': '#708493', 'stroke-width': '.7'})
        ET.SubElement(svg, '{' + NS + '}path', {
            'd': f'M {triangle[0].x} {triangle[0].y} L {triangle[1].x} {triangle[1].y} L {triangle[2].x} {triangle[2].y} Z',
            'fill': '#708493'})
    assert not any(label in target.get_text() for label in LEGEND)
    result.save(OUT / 'room_segmentation_en.pdf', garbage=4, deflate=True)
    for dpi, name in [(600, 'room_segmentation_en.png'),
                      (400, 'room_segmentation_en_preview.png')]:
        target.get_pixmap(dpi=dpi, alpha=False).save(OUT / name)
    ET.register_namespace('', NS)
    ET.register_namespace('xlink', 'http://www.w3.org/1999/xlink')
    ET.ElementTree(svg).write(OUT / 'room_segmentation_en.svg',
                              encoding='utf-8', xml_declaration=True)
    (OUT / 'layout_evidence.json').write_text(json.dumps({
        'base_variant': 'single_row',
        'base_pdf_sha256': hashlib.sha256(source_pdf.read_bytes()).hexdigest(),
        'base_svg_sha256': hashlib.sha256(source_svg.read_bytes()).hexdigest(),
        'size_mm': [WIDTH_MM, HEIGHT_MM], 'title_font_pt': TITLE_SIZE,
        'source_panel_rects_pt': [list(rect) for rect in sources],
        'destination_panel_rects_pt': [list(rect) for rect in destinations],
        'removed_footer_legend': sorted(LEGEND),
        'layout': 'two rows, two columns; single-column width; snake order a-b-c-d',
        'panel_positions': ['top left', 'top right', 'bottom right', 'bottom left'],
        'arrows_mm': arrows,
        'measured_geometry_regenerated': False, 'map_content_cropped': False,
    }, indent=2) + '\n')
    print(f'Exported {WIDTH_MM} x {HEIGHT_MM} mm, 2 x 2 panels, {TITLE_SIZE} pt headings.')


if __name__ == '__main__':
    main()
