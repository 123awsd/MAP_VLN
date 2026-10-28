import requests,time,hashlib
from pathlib import Path
p=Path(__file__).resolve().parent/'weights/model.pt';tmp=p.with_suffix('.download');n=5026874952
start=tmp.stat().st_size if tmp.exists() else 0
u='https://huggingface.co/facebook/VGGT-1B/resolve/main/model.pt?download=true&ts='+str(int(time.time()))
with requests.get(u,headers={'Range':f'bytes={start}-'} if start else {},stream=True,timeout=(30,60)) as r:
 r.raise_for_status()
 if start:assert r.status_code==206 and r.headers.get('Content-Range','').startswith(f'bytes {start}-')
 last=time.time()
 with tmp.open('ab' if start else 'wb') as f:
  for c in r.iter_content(1024*1024):
   f.write(c);start+=len(c)
   if time.time()-last>20:print(f'{start/1e9:.2f} / {n/1e9:.2f} GB',flush=True);last=time.time()
assert tmp.stat().st_size==n
h=hashlib.file_digest(tmp.open('rb'),'sha256').hexdigest();print('sha256',h,flush=True)
assert h=='d15bf50a8615c8225ed48b51ea5cac673d82442ec0309036df555a053253afe0'
tmp.rename(p);print('VERIFIED',p,flush=True)
