"""Small-core recovery and broad-interface merge without floor/room ID exceptions."""
from pathlib import Path
import sys,json
sys.path.insert(0,'/tmp/rose2_deps')
import numpy as np
from scipy.ndimage import label,distance_transform_edt
from skimage.segmentation import watershed
root=Path(__file__).resolve().parents[2]/'outputs/room_wall_guided_trial';rows=[]
for f in [1,2,3]:
 d=np.load(root/'r0.45'/f'L{f}'/'segmentation.npz');grid=d['grid'];wall=d['wall_barrier'];res=float(d['resolution']);domain=(grid!=-1)&~wall;distance=distance_transform_edt(domain)*res
 marker,num=label(distance>=.45);seeds=np.zeros(domain.shape,np.int32);k=0
 for i in range(1,num+1):
  m=marker==i
  # Existing 1m2 cutoff discards tiny interiors even when surrounded by real walls.
  if m.sum()*res**2<.12:continue
  k+=1;seeds[m]=k
 components,num=label(domain)
 for i in range(1,num+1):
  m=components==i
  if np.any(seeds[m]) or m.sum()*res**2<.5:continue
  yy,xx=np.where(m);j=np.argmax(distance[yy,xx]);k+=1;seeds[yy[j],xx[j]]=k
 result=watershed(-distance,seeds,mask=domain).astype(np.int32);parent=list(range(k+1));merges=[]
 def find(i):
  while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
  return i
 edges={}
 for axis in [0,1]:
  a=result[:-1,:] if axis==0 else result[:,:-1];b=result[1:,:] if axis==0 else result[:,1:];dist=distance[:-1,:] if axis==0 else distance[:,:-1];dist2=distance[1:,:] if axis==0 else distance[:,1:];m=(a>0)&(b>0)&(a!=b)
  for x,y,v in zip(a[m],b[m],np.minimum(dist[m],dist2[m])):
   pair=tuple(sorted([int(x),int(y)]));edges.setdefault(pair,[]).append(float(v))
 for (a,b),values in sorted(edges.items(),key=lambda x:-len(x[1])):
  # Broad passage has high clearance on both sides; door constrictions remain separated.
  interface_length=len(values)*res;clearance=float(np.quantile(values,.8));peaks=[float(distance[result==i].max()) for i in [a,b]]
  if (interface_length>=1.2 and interface_length/max(clearance,.05)>=6.0) or (.85<=interface_length<=1.1 and clearance<=.22 and interface_length/max(clearance,.05)>=4.5):
   pa,pb=find(a),find(b)
   if pa!=pb:parent[pb]=pa;merges.append({'a':a,'b':b,'interface_m':interface_length,'clearance_m':clearance})
 final=np.full(domain.shape,-1,np.int32);ids={}
 for i in range(1,k+1):
  p=find(i)
  if p not in ids:ids[p]=len(ids)
  final[result==i]=ids[p]
 out=root/'adaptive_merge'/f'L{f}';out.mkdir(parents=True,exist_ok=True);np.savez_compressed(out/'segmentation.npz',grid=grid,labels=final,origin=d['origin'],resolution=res,wall_barrier=wall);rows.append({'floor':f,'initial_cores':k,'regions':len(ids),'automatic_merges':merges,'unknown_colored':int(np.sum((final>=0)&(grid==-1)))});print(rows[-1])
(root/'adaptive_merge/metrics.json').write_text(json.dumps(rows,indent=2))
