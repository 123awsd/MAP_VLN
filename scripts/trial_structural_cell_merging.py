"""Architectural plane arrangement with signed shared-boundary region merging.
Coordinates are inferred from observed wall evidence; no manual room seeds/counts.
"""
from pathlib import Path
import json,numpy as np,cv2,heapq
from scipy.signal import find_peaks
from scipy.ndimage import binary_fill_holes,gaussian_filter1d
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'outputs/room_quality_ceiling';OUT=ROOT/'outputs/structural_cell_merging';OUT.mkdir(exist_ok=True);VIS=ROOT.parent/'视频制作/建筑区域整体合并';VIS.mkdir(exist_ok=True);g=np.load(BASE/'geometry.npz');BG=(18,22,28);font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',27);small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',21);records=[]
for f,ground in enumerate([-.155,2.595,5.595]):
 p,n=g[f'p{f}'],g[f'n{f}'];res=.05;org=np.floor((p[:,:2].min(0)-.25)/res)*res;wh=np.ceil((p[:,:2].max(0)+.25-org)/res).astype(int)+1;shape=tuple(wh[::-1]);uv=np.floor((p[:,:2]-org)/res).astype(int);count=np.zeros(shape,np.int32);np.add.at(count,(uv[:,1],uv[:,0]),1);domain=binary_fill_holes(cv2.morphologyEx((count>0).astype('uint8'),cv2.MORPH_CLOSE,np.ones((5,5),np.uint8)))
 rel=p[:,2]-ground;bins=np.zeros((14,*shape),np.uint8)
 for k in range(14):
  use=(abs(n[:,2])<.35)&(rel>=k*.2)&(rel<(k+1)*.2)&(rel>.3);xy=uv[use];bins[k,xy[:,1],xy[:,0]]=1
 votes=sum(cv2.dilate(b,np.ones((3,3),np.uint8))for b in bins);upper=sum(cv2.dilate(b,np.ones((3,3),np.uint8))for b in bins[7:]);wall=np.clip(votes/7,0,1)*np.clip(upper/2,0,1)
 # Continuous candidate coordinates from peaks in wall support, shared by all rooms.
 xs=find_peaks(gaussian_filter1d(wall.sum(0),1),distance=5,prominence=5)[0];ys=find_peaks(gaussian_filter1d(wall.sum(1),1),distance=5,prominence=5)[0];xs=np.r_[0,xs,shape[1]];ys=np.r_[0,ys,shape[0]];nx=len(xs)-1;ny=len(ys)-1
 initial=np.zeros(shape,np.int32)
 for y in range(ny):
  for x in range(nx):initial[ys[y]:ys[y+1],xs[x]:xs[x+1]]=y*nx+x+1
 initial[~domain]=0;areas=np.bincount(initial.ravel(),minlength=nx*ny+1)*res**2
 # Candidate cells occupied mainly by structural wall evidence represent wall volume, not rooms.
 wallmass=np.bincount(initial.ravel(),weights=wall.ravel(),minlength=nx*ny+1)*res**2
 solid=(wallmass/np.maximum(areas,1e-9)>.65);solid[0]=False;initial[solid[initial]]=0
 areas=np.bincount(initial.ravel(),minlength=nx*ny+1)*res**2
 for threshold in [.3,.5,.7]:
  parent=np.arange(nx*ny+1);neighbors={i:{}for i in range(1,len(parent))if areas[i]>0};heap=[]
  for y in range(ny):
   for x in range(nx):
    a=y*nx+x+1
    if a not in neighbors:continue
    for yy,xx in [(y,x+1),(y+1,x)]:
     if yy>=ny or xx>=nx:continue
     b=yy*nx+xx+1
     if b not in neighbors:continue
     if yy==y:
      edge=xs[xx];vals=wall[ys[y]:ys[y+1],max(0,edge-2):min(shape[1],edge+3)].max(1);valid=domain[ys[y]:ys[y+1],edge-1]&domain[ys[y]:ys[y+1],edge]
     else:
      edge=ys[yy];vals=wall[max(0,edge-2):min(shape[0],edge+3),xs[x]:xs[x+1]].max(0);valid=domain[edge-1,xs[x]:xs[x+1]]&domain[edge,xs[x]:xs[x+1]]
     if not valid.any():continue
     cost=float((vals[valid]-threshold).sum()*res);neighbors[a][b]=cost;neighbors[b][a]=cost;heapq.heappush(heap,(cost,a,b))
  while heap:
   cost,a,b=heapq.heappop(heap)
   if a not in neighbors or b not in neighbors or neighbors[a].get(b)!=cost:continue
   if cost>=0:break
   # Greedy decrease of the total signed inter-region boundary objective.
   parent[b]=a
   for c,v in list(neighbors[b].items()):
    neighbors[c].pop(b,None)
    if c==a:continue
    total=neighbors[a].get(c,0)+v;neighbors[a][c]=total;neighbors[c][a]=total;heapq.heappush(heap,(total,min(a,c),max(a,c)))
   neighbors.pop(b);neighbors[a].pop(b,None)
  for i in range(1,len(parent)):
   j=i
   while parent[j]!=j:j=parent[j]
   parent[i]=j
  merged=parent[initial];unique=np.unique(merged[merged>0]);lut=np.zeros(len(parent),int);lut[unique]=np.arange(1,len(unique)+1);labels=lut[merged]
  boundary=np.zeros(shape,np.uint8);boundary[:,1:]|=((labels[:,1:]!=labels[:,:-1])&(labels[:,1:]>0)&(labels[:,:-1]>0)).astype('uint8');boundary[1:]|=((labels[1:]!=labels[:-1])&(labels[1:]>0)&(labels[:-1]>0)).astype('uint8')
  np.savez_compressed(OUT/f'L{f+1}_t{threshold}.npz',labels=labels,wall_evidence=wall,boundary=boundary,origin=org,resolution=res,x_planes=org[0]+xs*res,y_planes=org[1]+ys*res)
  rgb=np.full((*shape,3),BG,np.uint8);shade=(35+70*np.clip(np.log1p(count)/np.log(60),0,1)).astype('uint8');rgb[count>0]=shade[count>0,None];rgb[boundary>0]=250;pic=Image.fromarray(np.flipud(rgb).copy());scale=min(850/pic.width,1240/pic.height);pic=pic.resize((round(pic.width*scale),round(pic.height*scale)),Image.Resampling.NEAREST);panel=Image.new('RGB',(900,1500),BG);left=(900-pic.width)//2;panel.paste(pic,(left,160));d=ImageDraw.Draw(panel);d.text((22,20),f'{f+1}层 · 建筑区域整体合并 {threshold}',font=font,fill='white');d.text((22,75),f'自动产生 {len(unique)} 个区域 · 灰：原始点云',font=small,fill=(190,202,216))
  for i in range(1,len(unique)+1):
   yy,xx=np.where(labels==i)
   if len(yy)*res**2<.8:continue
   k=np.argmin((yy-yy.mean())**2+(xx-xx.mean())**2);d.text((left+xx[k]*scale,160+(shape[0]-yy[k])*scale),str(i),font=small,fill='white',stroke_width=2,stroke_fill=BG)
  panel.save(VIS/f'L{f+1}_t{threshold}.png');records.append(dict(floor=f+1,threshold=threshold,regions=len(unique),small_regions=int(sum((labels==i).sum()*res**2<1 for i in range(1,len(unique)+1)))));print(records[-1],flush=True)
(OUT/'metrics.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
