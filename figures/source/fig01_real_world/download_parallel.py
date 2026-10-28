import requests,time,concurrent.futures,hashlib,os
from pathlib import Path
r=Path(__file__).resolve().parent/'weights';d=r/'parts';d.mkdir(exist_ok=True);n=5026874952;chunk=8*1024*1024
u='https://hf-mirror.com/facebook/VGGT-1B/resolve/main/model.pt'
def get(start):
 end=min(n-1,start+chunk-1);p=d/str(start)
 if p.exists() and p.stat().st_size==end-start+1:return p.stat().st_size
 for i in range(5):
  try:
   with requests.get(u+'?part='+str(start)+'&t='+str(int(time.time())),headers={'Range':f'bytes={start}-{end}'},stream=True,timeout=(30,45)) as q:
    q.raise_for_status();assert q.status_code==206 and q.headers.get('Content-Range')==f'bytes {start}-{end}/{n}'
    with p.with_suffix('.tmp').open('wb') as f:
     for c in q.iter_content(262144):f.write(c)
   assert p.with_suffix('.tmp').stat().st_size==end-start+1
   p.with_suffix('.tmp').rename(p);return end-start+1
  except Exception:
   if i==4:raise
   time.sleep(2)
count=0;last=0
with concurrent.futures.ThreadPoolExecutor(max_workers=32) as ex:
 for f in concurrent.futures.as_completed([ex.submit(get,s) for s in range(0,n,chunk)]):
  count+=f.result()
  if time.time()-last>15:print(f'{count/1e9:.2f}/{n/1e9:.2f} GB verified chunks',flush=True);last=time.time()
h=hashlib.sha256()
with (r/'model.assembling').open('wb') as out:
 for start in range(0,n,chunk):
  b=(d/str(start)).read_bytes();h.update(b);out.write(b)
assert h.hexdigest()=='d15bf50a8615c8225ed48b51ea5cac673d82442ec0309036df555a053253afe0'
(r/'model.assembling').rename(r/'model.pt');print('SHA256 VERIFIED',flush=True)
