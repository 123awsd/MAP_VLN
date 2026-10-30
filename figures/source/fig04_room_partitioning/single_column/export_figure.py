#!/usr/bin/env python3
"""Derive the single-column figure from the existing measured vector artwork."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import fitz


OUT = Path(__file__).resolve().parent
BASE = OUT.parent / 'single_row'
PT_PER_MM = 72 / 25.4
WIDTH_MM = 88
CROP_HEIGHT_MM = 50
LEGEND = {'Supported space', 'Excluded', 'Interior cores', 'Unassigned'}


def main():
    source_pdf = BASE / 'room_segmentation_en.pdf'
    source_svg = BASE / 'room_segmentation_en.svg'
    document = fitz.open(source_pdf)
    page = document[0]
    crop = fitz.Rect(0, 0, page.rect.width, CROP_HEIGHT_MM * PT_PER_MM)
    spans = [span for block in page.get_text('dict')['blocks']
             for line in block.get('lines', []) for span in line['spans']]
    assert {span['text'] for span in spans if span['text'] in LEGEND} == LEGEND
    for span in spans:
        box = fitz.Rect(span['bbox'])
        assert box.y0 > crop.y1 if span['text'] in LEGEND else box.y1 < crop.y1
    for image in page.get_images():
        assert all(box.y1 < crop.y1 for box in page.get_image_rects(image[0]))
    for drawing in page.get_drawings():
        box = drawing['rect']
        if box.y0 < crop.y1 < box.y1:
            assert box.contains(page.rect), 'Crop would intersect figure geometry'

    page.add_redact_annot(fitz.Rect(0, crop.y1, page.rect.width, page.rect.height),
                          fill=False)
    page.apply_redactions(images=0, graphics=0)
    width = WIDTH_MM * PT_PER_MM
    height = crop.height * width / crop.width
    result = fitz.open()
    target = result.new_page(width=width, height=height)
    target.show_pdf_page(target.rect, document, 0, clip=crop)
    result.save(OUT / 'room_segmentation_en.pdf', garbage=4, deflate=True)
    assert not any(label in target.get_text() for label in LEGEND)
    for dpi, name in [(600, 'room_segmentation_en.png'),
                      (400, 'room_segmentation_en_preview.png')]:
        target.get_pixmap(dpi=dpi, alpha=False).save(OUT / name)

    ET.register_namespace('', 'http://www.w3.org/2000/svg')
    ET.register_namespace('xlink', 'http://www.w3.org/1999/xlink')
    tree = ET.parse(source_svg)
    svg = tree.getroot()
    namespace = {'s': 'http://www.w3.org/2000/svg'}
    canvas = svg.find('.//s:g[@id="axes_1"]', namespace)
    for number in range(5, 9):
        for prefix in ('patch', 'text'):
            group = canvas.find(f's:g[@id="{prefix}_{number}"]', namespace)
            assert group is not None
            if prefix == 'text':
                assert ''.join(group.itertext()).strip() in LEGEND
            canvas.remove(group)
    svg.set('width', f'{width:.9f}pt')
    svg.set('height', f'{height:.9f}pt')
    svg.set('viewBox', f'0 0 {crop.width:.9f} {crop.height:.9f}')
    tree.write(OUT / 'room_segmentation_en.svg', encoding='utf-8', xml_declaration=True)
    (OUT / 'layout_evidence.json').write_text(json.dumps({
        'base_variant': 'single_row',
        'base_pdf_sha256': hashlib.sha256(source_pdf.read_bytes()).hexdigest(),
        'base_svg_sha256': hashlib.sha256(source_svg.read_bytes()).hexdigest(),
        'size_mm': [WIDTH_MM, height / PT_PER_MM],
        'source_clip_pt': list(crop),
        'removed_footer_legend': sorted(LEGEND),
        'layout': 'single row, four panels; single-column width',
        'measured_geometry_regenerated': False,
        'map_content_cropped': False,
    }, indent=2) + '\n')
    print(f'Exported {WIDTH_MM} x {height / PT_PER_MM:.2f} mm without footer legend.')


if __name__ == '__main__':
    main()
