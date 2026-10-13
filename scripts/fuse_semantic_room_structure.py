"""Fuse synchronized semantic RGB-D, keep semantic wall evidence and derive region hypotheses."""
from pathlib import Path
import numpy as np,cv2,json
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/semantic_structure_trial';BASE=ROOT/'outputs/room_quality_ceiling';VIS=ROOT.parent/'视频制作/语义几何房间结构';VIS.mkdir(exist_ok=True)
g=np.load(BASE/'geometry.npz');font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',27);small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',19);BG=(18,22,28);metrics=[]
for f,ground in enumerate([-.155,2.595,5.595]):
 b=np.load(BASE/f'D_墙面方向统一_L{f+1}.npz');org=b['origin'];res=float(b['resolution']);shape=b['wall'].shape;p=g[f'p{f}'];uv=np.floor((p[:,:2]-org)/res).astype(int);counts=np.zeros(shape,np.int32);np.add.at(counts,(uv[:,1],uv[:,0]),1);votes=np.zeros((3,*shape),np.uint16);high=np.zeros(shape,np.uint16);tree=cKDTree(p);normals=g[f'n{f}']
 for path in sorted((OUT/'rgbd').glob('*.npz')):
  s=np.load(OUT/'semantics'/path.name);d=np.load(path);dep=d['depth'];h,w=dep.shape;v,u=np.mgrid[0:h:4,0:w:4];v=v.ravel();u=u.ravel();z=dep[v,u];lab=s['labels'][v,u];conf=s['confidence'][v,u];valid=(z>.1)&(z<4.95)&(conf>=.45);v,u,z,lab=v[valid],u[valid],z[valid],lab[valid];xyz=np.c_[(u-w/2)*z/(w/2),(v-h/2)*z/(w/2),z]@d['rotation'].T+d['position'];xy=np.floor((xyz[:,:2]-org)/res).astype(int);rel=xyz[:,2]-ground;ok=(rel>.15)&(rel<2.7)&(xy[:,0]>=0)&(xy[:,0]<shape[1])&(xy[:,1]>=0)&(xy[:,1]<shape[0]);near,ii=tree.query(xyz[ok]);vertical=(near<.12)&(abs(normals[ii,2])<.35)
  xy,lab,rel=xy[ok],lab[ok],rel[ok]
  for k,good in enumerate([(lab==0)&vertical,lab==14,~np.isin(lab,[0,3,5,8,14,42])]):
   frame=np.zeros(shape,bool);frame[xy[good,1],xy[good,0]]=1;votes[k]+=cv2.dilate(frame.astype("uint8"),np.ones((3,3),np.uint8))
  frame=np.zeros(shape,bool);good=(lab==0)&vertical&(rel>1.4);frame[xy[good,1],xy[good,0]]=1;high+=cv2.dilate(frame.astype("uint8"),np.ones((3,3),np.uint8))
 observed=(votes[0]>=2)&((high>=1)|(votes[0]>=4));wall=cv2.morphologyEx(observed.astype('uint8'),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))>0
 # Derive furniture-independent interior from observed building footprint, then split with wall evidence.
 roof=np.load(BASE/f'E_天花板边界辅助_L{f+1}.npz');domain=roof['envelope'];barrier=cv2.dilate(wall.astype('uint8'),np.ones((3,3),np.uint8))>0;inside=domain&~barrier
 dist=distance_transform_edt(inside)*res;core=(dist>.6).astype('uint8');num,labels,stats,_=cv2.connectedComponentsWithStats(core,8);markers=np.zeros(shape,np.int32);idx=1
 for k in range(1,num):
  if stats[k,4]*res*res>=.8:markers[labels==k]=idx;idx+=1
 from collections import deque
 queue=deque(map(tuple,np.argwhere(markers>0)))
 while queue:
  y,x=queue.popleft()
  for yy,xx in [(y-1,x),(y+1,x),(y,x-1),(y,x+1)]:
   if 0<=yy<shape[0] and 0<=xx<shape[1] and inside[yy,xx] and markers[yy,xx]==0:markers[yy,xx]=markers[y,x];queue.append((yy,xx))
 boundary=np.zeros(shape,bool);boundary[:,1:]|=(markers[:,1:]!=markers[:,:-1])&(markers[:,1:]>0)&(markers[:,:-1]>0);boundary[1:]|=(markers[1:]!=markers[:-1])&(markers[1:]>0)&(markers[:-1]>0)
 np.savez_compressed(OUT/f'L{f+1}_fusion.npz',wall_votes=votes[0],door_votes=votes[1],furniture_votes=votes[2],upper_wall_votes=high,wall=wall,room_labels=markers,room_boundary=boundary,origin=org,resolution=res)
 shade=(35+65*np.clip(np.log1p(counts)/np.log(60),0,1)).astype('uint8');cloud=np.full((*shape,3),BG,np.uint8);cloud[counts>0]=shade[counts>0,None];yy,xx=np.where(counts>0);ys=slice(max(0,yy.min()-6),min(shape[0],yy.max()+7));xs=slice(max(0,xx.min()-6),min(shape[1],xx.max()+7));board=Image.new('RGB',(2700,1500),BG)
 for j,(name,mask)in enumerate([('原始点云',np.zeros(shape,bool)),('多视角语义墙体',wall),('房间区域假设',wall|boundary)]):
  im=cloud.copy();im[mask]=250;pic=Image.fromarray(np.flipud(im[ys,xs]).copy());sc=min(830/pic.width,1220/pic.height);pic=pic.resize((round(pic.width*sc),round(pic.height*sc)),Image.Resampling.NEAREST);panel=Image.new('RGB',(900,1500),BG);panel.paste(pic,((900-pic.width)//2,155));dr=ImageDraw.Draw(panel);dr.text((24,20),f'{f+1}层 · {name}',font=font,fill='white');dr.text((24,70),'灰：原始点云；白：结构候选',font=small,fill=(190,202,216));panel.save(VIS/f'{f+1}层_{name}.png');board.paste(panel,(j*900,0))
 board.save(VIS/f'{f+1}层_对比.png');metrics.append(dict(floor=f+1,room_hypotheses=idx-1,observed_wall_cells=int(observed.sum()),unassigned_interior_cells=int((inside&(markers==0)).sum())));print(metrics[-1],flush=True)
(OUT/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2))
