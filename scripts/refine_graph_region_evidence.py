"""Merge graph partitions lacking measured shared architectural support."""
from pathlib import Path
import numpy as np,json
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/room_graph_partition';VIS=ROOT.parent/'视频制作/整体房间图划分';font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',22);records=[]
for f in [1,2,3]:
 d=np.load(OUT/f'L{f}_0.npz');labels=d['labels'].copy();ev=d['wall_evidence'];parent=np.arange(labels.max()+1)
 for iteration in range(20):
  pairs={}
  for a,b,e in [(labels[:,1:],labels[:,:-1],np.maximum(ev[:,1:],ev[:,:-1])),(labels[1:],labels[:-1],np.maximum(ev[1:],ev[:-1]))]:
   y,x=np.where((a!=b)&(a>0)&(b>0))
   for yy,xx in zip(y,x):pairs.setdefault(tuple(sorted((a[yy,xx],b[yy,xx]))),[]).append(e[yy,xx])
  weak=[(np.mean(v),k)for k,v in pairs.items()if np.mean(v)<.35]
  if not weak:break
  _,(a,b)=min(weak);labels[labels==b]=a
 boundary=np.zeros(labels.shape,bool);boundary[:,1:]|=(labels[:,1:]!=labels[:,:-1])&(labels[:,1:]>0);boundary[1:]|=(labels[1:]!=labels[:-1])&(labels[1:]>0)
 ids=np.unique(labels[labels>0]);np.savez_compressed(OUT/f'L{f}_merged.npz',labels=labels,boundary=boundary,wall_evidence=ev,origin=d['origin'],resolution=d['resolution'])
 im=Image.open(VIS/f'{f}层_0.png');# Render at identical original comparison coordinates.
 rgb=np.full((*labels.shape,3),(18,22,28),np.uint8);rgb[labels>0]=(45,48,52);rgb[boundary]=(245,245,245);pic=Image.fromarray(np.flipud(rgb).copy());sc=min(830/pic.width,1210/pic.height);pic=pic.resize((round(pic.width*sc),round(pic.height*sc)),Image.Resampling.NEAREST);panel=Image.new('RGB',(900,1500),(18,22,28));left=(900-pic.width)//2;panel.paste(pic,(left,170));dr=ImageDraw.Draw(panel);dr.text((24,20),f'{f}层 · 共享边界证据合并 · {len(ids)}区域',font=font,fill='white')
 for i,k in enumerate(ids):
  yy,xx=np.where(labels==k);c=np.argmin((yy-yy.mean())**2+(xx-xx.mean())**2);dr.text((left+xx[c]*sc,170+(labels.shape[0]-1-yy[c])*sc),str(i+1),font=font,fill='white',stroke_width=2,stroke_fill=(18,22,28))
 panel.save(VIS/f'{f}层_弱分界合并.png');records.append(dict(floor=f,regions=len(ids),merges=iteration))
(OUT/'merged_metrics.json').write_text(json.dumps(records,indent=2));print(records)
