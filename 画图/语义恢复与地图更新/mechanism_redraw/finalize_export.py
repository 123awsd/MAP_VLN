"""Compress PDF raster streams, retain vectors/alpha, and inspect final export.

Run with the workspace Python (PyMuPDF and Pillow). Native raster dimensions
are preserved. JPEG quality 95 is used only for embedded photographic streams;
transparency masks and all SVG/PDF geometry remain lossless.
"""
from pathlib import Path
from io import BytesIO
import hashlib
import json
import re
import xml.etree.ElementTree as ET
import fitz
from PIL import Image

OUT=Path(__file__).resolve().parent
PDF=OUT/'fig8_mechanism_en.pdf'
doc=fitz.open(PDF)
for record in doc[0].get_images(full=True):
    xref=record[0]
    if 'DCTDecode' in doc.xref_get_key(xref,'Filter')[1]:
        continue
    pix=fitz.Pixmap(doc,xref)
    if pix.n != 3:
        continue
    im=Image.frombytes('RGB',(pix.width,pix.height),pix.samples)
    stream=BytesIO()
    im.save(stream,format='JPEG',quality=95,subsampling=0,optimize=True)
    # Updating the RGB stream preserves the existing /SMask reference.
    doc.update_stream(xref,stream.getvalue(),compress=False)
    doc.xref_set_key(xref,'Filter','/DCTDecode')
    doc.xref_set_key(xref,'DecodeParms','null')
    doc.xref_set_key(xref,'ColorSpace','/DeviceRGB')
    doc.xref_set_key(xref,'BitsPerComponent','8')
tmp=OUT/'export_tmp.pdf'
doc.save(tmp,garbage=4,deflate=True,clean=True)
doc.close()
tmp.replace(PDF)

doc=fitz.open(PDF)
p=doc[0]
spans=[s for b in p.get_text('dict')['blocks'] if 'lines' in b for l in b['lines'] for s in l['spans']]
layout=json.loads((OUT/'layout.json').read_text())
collisions=[]
for i,a in enumerate(layout['text']):
    for b in layout['text'][i+1:]:
        if min(a['right'],b['right'])>max(a['left'],b['left']) and min(a['bottom'],b['bottom'])>max(a['top'],b['top']):
            collisions.append([a['text'],b['text']])
svg=ET.parse(OUT/'fig8_mechanism_en.svg')
external=[v for e in svg.iter() for k,v in e.attrib.items()
          if k.endswith('href') and not v.startswith(('data:','#'))]
p.get_pixmap(matrix=fitz.Matrix(3,3),alpha=False).save(OUT/'fig8_mechanism_en_pdf_check.png')
p.get_pixmap(matrix=fitz.Matrix(1.8,1.8),alpha=False).save(OUT/'fig8_mechanism_en_readability.png')
check=dict(page_count=len(doc),physical_size_mm=[p.rect.width/72*25.4,p.rect.height/72*25.4],
           minimum_font_pt=min(s['size'] for s in spans),text_bounds_overlaps=collisions,
           all_labels_english=not bool(re.search(r'[\u4e00-\u9fff]',p.get_text())),
           native_pdf_text_spans=len(spans),svg_external_dependencies=external,
           pdf_reopened_and_rendered=True,pdf_photograph_compression='JPEG quality 95; native dimensions; lossless alpha masks',
           pdf_size_mb=round(PDF.stat().st_size/1e6,3))
assert not collisions and not external
assert check['all_labels_english'] and check['page_count']==1
G=json.loads((OUT/'assets/projection.json').read_text())
old=json.loads((OUT.parent/'multiroom_redraw/provenance.json').read_text())
frames=OUT.parents[2]/'画图/Habitat长指令执行/keyframes/00166_cushion_recovery_20261024'
files={'miss':'obs_000000.png','earlier':'frame_000420.png','verified':'obs_000003.png'}
provenance=dict(kind='Spatial mechanism illustration in a real HM3D apartment',date='2026-10-24',
    source_scene=G['source_scene'],scene='Same full apartment mesh; near-overhead camera; original texture UV correction retained',
    route=G['route_kind'],coverage=G['coverage_kind'],
    coverage_limits='Author-designed planar partitions; not measured FoV/visibility/confidence; no quantitative belief values',
    new_instance=old['new_instance'],source_logs_modified=False,manuscript_modified=False,
    route_geometry_check=old['geometry_check'],layout_check=check,
    source_observations=[dict(path=str(frames/files[c['asset']]),
        sha256=hashlib.sha256((frames/files[c['asset']]).read_bytes()).hexdigest(),
        crop_xywh=c['crop_xywh']) for c in layout['crops']],
    outputs_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in OUT.iterdir()
                    if f.suffix in ['.pdf','.svg','.py','.tex','.blend'] and f.name!='fig8_mechanism_en_compact.pdf'})
(OUT/'provenance.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(check,indent=2))
