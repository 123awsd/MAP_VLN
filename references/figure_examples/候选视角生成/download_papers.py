"""Download public research PDFs listed in papers.json; never overwrite valid PDFs."""
from pathlib import Path
import concurrent.futures
import hashlib
import json
import requests
import fitz

ROOT = Path(__file__).resolve().parent

def download(row):
    dst = ROOT / 'papers' / (row['id'] + '.pdf')
    try:
        if not dst.exists() or not dst.read_bytes().startswith(b'%PDF'):
            response = requests.get(row['url'], timeout=55)
            response.raise_for_status()
            if not response.content.startswith(b'%PDF'):
                raise ValueError('Response is not a PDF')
            dst.write_bytes(response.content)
        doc = fitz.open(dst)
        row.update(pages=len(doc), sha256=hashlib.sha256(dst.read_bytes()).hexdigest(), status='ok')
        (ROOT / 'texts').mkdir(exist_ok=True)
        (ROOT / 'texts' / (row['id'] + '.txt')).write_text('\n'.join(f'\n--- PDF PAGE {i+1} ---\n'+p.get_text() for i,p in enumerate(doc)))
        print(f"OK {row['id']}: {len(doc)} pages", flush=True)
    except Exception as exc:
        row.update(status='failed',error=str(exc))
        print(f"FAILED {row['id']}: {exc}", flush=True)
    return row

if __name__ == '__main__':
    rows=json.loads((ROOT/'papers.json').read_text())
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        rows=list(pool.map(download, rows))
    (ROOT/'download_log.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
