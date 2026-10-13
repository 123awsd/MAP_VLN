"""Test ceiling-connected room interiors with local sloped-roof height evidence."""
from pathlib import Path
import numpy as np,cv2,json
from scipy.ndimage import maximum_filter,median_filter,distance_transform_edt
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'outputs/room_quality_ceiling';OUT=ROOT/'outputs/ceiling_room_support';OUT.mkdir(exist_ok=True);VIS=ROOT.parent/'视频制作/局部顶面房间支持';VIS.mkdir(exist_ok=True);g=np.load(BASE/'geometry.npz');BG=(18,22,28);font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',27);small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',20);metrics=[]
for f,ground in enumerate([-.155,2.595,5.595]):
 b=np.load(BASE/f'D_墙面方向统一_L{f+1}.npz');org=b['origin'];res=float(b['resolution']);shape=b['wall'].shape;p,n=g[f'p{f}'],g[f'n{f}'];uv=np.floor((p[:,:2]-org)/res).astype(int);counts=np.zeros(shape,int);np.add.at(counts,(uv[:,1],uv[:,0]),1);top=np.full(shape,ground,np.float32);np.maximum.at(top,(uv[:,1],uv[:,0]),p[:,2]);near_top=maximum_filter(top,size=3)
 for mode in ['局部顶面','局部顶面连通']:
  selected=(p[:,2]>ground+1.4)&(p[:,2]>near_top[uv[:,1],uv[:,0]]-.15)&(abs(n[:,2])>.45);xy=uv[selected];roof=np.zeros(shape,np.uint8);roof[xy[:,1],xy[:,0]]=1
  # Close sampling holes at 5cm; no long wall bridging.
  roof=cv2.morphologyEx(roof,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8));dist=distance_transform_edt(roof)*res;core=(dist>(.1 if mode=='局部顶面' else .25)).astype('uint8');num,lab,stats,_=cv2.connectedComponentsWithStats(core,8);markers=np.zeros(shape,np.int32);j=0
  for k in range(1,num):
   if stats[k,4]*res*res>=.8:j+=1;markers[lab==k]=j
  from collections import deque
  queue=deque(map(tuple,np.argwhere(markers>0)))
  while queue:
   y,x=queue.popleft()
   for yy,xx in [(y-1,x),(y+1,x),(y,x-1),(y,x+1)]:
    if 0<=yy<shape[0] and 0<=xx<shape[1] and roof[yy,xx] and not markers[yy,xx]:markers[yy,xx]=markers[y,x];queue.append((yy,xx))
  boundary=np.zeros(shape,np.uint8)
  for k in range(1,j+1):
   mask=(markers==k).astype('uint8');boundary|=mask-cv2.erode(mask,np.ones((3,3),np.uint8))
  np.savez_compressed(OUT/f'L{f+1}_{mode}.npz',ceiling_labels=markers,ceiling_support=roof,origin=org,resolution=res)
  yy,xx=np.where(counts>0);ys=slice(max(0,yy.min()-6),min(shape[0],yy.max()+7));xs=slice(max(0,xx.min()-6),min(shape[1],xx.max()+7));rgb=np.full((*shape,3),BG,np.uint8);shade=(35+65*np.clip(np.log1p(counts)/np.log(60),0,1)).astype('uint8');rgb[counts>0]=shade[counts>0,None];rgb[boundary>0]=250;pic=Image.fromarray(np.flipud(rgb[ys,xs]).copy());scale=min(830/pic.width,1220/pic.height);pic=pic.resize((round(pic.width*scale),round(pic.height*scale)),Image.Resampling.NEAREST);panel=Image.new('RGB',(900,1500),BG);left=(900-pic.width)//2;panel.paste(pic,(left,160));d=ImageDraw.Draw(panel);d.text((24,20),f'{f+1}层 · {mode} · {j}块',font=font,fill='white');d.text((24,75),'白：顶面支持区域边缘（尚非墙线）',font=small,fill=(190,202,216));panel.save(VIS/f'L{f+1}_{mode}.png');metrics.append(dict(floor=f+1,mode=mode,regions=j));print(metrics[-1],flush=True)
(OUT/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2))
