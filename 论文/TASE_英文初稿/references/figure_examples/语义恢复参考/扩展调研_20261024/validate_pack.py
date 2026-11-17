"""Verify delivered images, PDFs, source hashes and local index links."""
from pathlib import Path
import collections
import hashlib
import json
import re
import fitz
from PIL import Image

ROOT=Path(__file__).resolve().parent
data=json.loads((ROOT/'sources.json').read_text())
errors=[]
for entry in data['figures']:
    for key in ['png','pdf','source_pdf']:
        assert (ROOT/entry[key]).is_file(),(entry['id'],key)
    Image.open(ROOT/entry['png']).verify()
    assert len(fitz.open(ROOT/entry['pdf']))==1
for name,paper in data['papers'].items():
    assert hashlib.sha256((ROOT/paper['local_pdf']).read_bytes()).hexdigest()==paper['sha256'],name
pdfs={}
for name,count in [('Fig8_完整参考图集_57项.pdf',57),('Fig8_优先12项详解.pdf',12),('Fig8_分类速览.pdf',8)]:
    doc=fitz.open(ROOT/name)
    assert len(doc)==count,(name,len(doc))
    if '速览' not in name:
        for i,page in enumerate(doc):
            text=page.get_text()
            assert '可借鉴' in text and '区别' in text and page.get_links(),(name,i)
        doc[0].get_pixmap(matrix=fitz.Matrix(1.2,1.2),alpha=False).save(ROOT/'review_pages'/('check_'+name.replace('.pdf','.png')))
    pdfs[name]=dict(pages=len(doc),megabytes=round((ROOT/name).stat().st_size/1e6,2))
for name in ['README.md','index.html']:
    text=(ROOT/name).read_text()
    links=re.findall(r'(?:href|src)="([^"]+)"',text) if name.endswith('html') else re.findall(r'\]\(([^)]+)\)',text)
    for link in links:
        if not link.startswith(('http','https','#')) and not (ROOT/link.split('#')[0]).exists():
            errors.append(name+' broken '+link)
log=json.loads((ROOT/'download_log.json').read_text())
for row in log:
    if row['id'] in data['papers']:
        good=data['papers'][row['id']]
        row.update(status='ok',pages=good['pages'],sha256=good['sha256'])
        row.pop('error',None)
(ROOT/'download_log.json').write_text(json.dumps(log,ensure_ascii=False,indent=2))
result=dict(papers=len(data['papers']),figures=len(data['figures']),groups=dict(collections.Counter(e['group'] for e in data['figures'])),pdfs=pdfs,errors=errors,verification='All image files decode, source hashes match, PDFs open, local links resolve; figure pages contain notes and source links.')
(ROOT/'validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps(result,ensure_ascii=False))
assert not errors,errors
