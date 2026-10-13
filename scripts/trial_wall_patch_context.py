"""Measured wall patches with local upper-surface context, without synthesized lines."""
from pathlib import Path
import numpy as np,cv2,json
from scipy.spatial import cKDTree
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/wall_patch_context';OUT.mkdir(exist_ok=True);VIS=ROOT.parent/'视频制作/墙面片段结构验证';VIS.mkdir(exist_ok=True);g=np.load(ROOT/'outputs/room_quality_ceiling/geometry.npz');font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',27);BG=(18,22,28);metrics=[]
for f,ground in enumerate([-.155,2.595,5.595]):
 p,n=g[f'p{f}'],g[f'n{f}'];q=p[(abs(n[:,2])<.3)&(p[:,2]>ground+.3)];nn=n[(abs(n[:,2])<.3)&(p[:,2]>ground+.3)];roof=p[(abs(n[:,2])>.55)&(p[:,2]>ground+1.7)];tree=cKDTree(roof);near,_=tree.query(q,k=1);rooftree=cKDTree(roof[:,:2]);_,ri=rooftree.query(q[:,:2],k=8);localtop=np.quantile(roof[ri,2],.75,axis=1);# spatial proximity includes height; not XY envelope proximity.
 angle=np.angle(np.mean(np.exp(4j*np.arctan2(nn[:,1],nn[:,0]))))/4;patches=[]
 for theta in [angle,angle+np.pi/2]:
  normal=np.array([np.cos(theta),np.sin(theta)]);tan=np.array([-normal[1],normal[0]]);align=abs(nn[:,:2]@normal)>.95;keys=np.round(q[:,:2]@normal/.04).astype(int);unique,cnt=np.unique(keys[align],return_counts=True);used=[]
  for key in unique[np.argsort(cnt)[::-1]]:
   off=key*.04
   if any(abs(off-u)<.12 for u in used):continue
   ids=np.where(align&(abs(q[:,:2]@normal-off)<.055))[0]
   if len(ids)<100:continue
   t=q[ids,:2]@tan;z=q[ids,2];lo=t.min();ti=np.floor((t-lo)/.1).astype(int);nz=int(ti.max())+1;hit=np.zeros((14,nz),bool);zb=np.clip(((z-ground)/.2).astype(int),0,13);hit[zb,ti]=1;nearcol=np.zeros(nz,bool);nearcol[ti[near[ids]<.3]]=1
   support=(hit.sum(0)>=3);diff=np.diff(np.r_[False,support,False].astype(int));accepted=False
   for a,b in zip(np.where(diff==1)[0],np.where(diff==-1)[0]):
    if (b-a)*.1<.5:continue
    select=ids[(ti>=a)&(ti<b)];height=np.ptp(q[select,2]);contact=nearcol[a:b].mean();relative=height/max(float(np.median(localtop[select]))-ground-.3,.5);patches.append((select,height,contact,relative));accepted=True
   if accepted:used.append(off)
 variants=[('局部高度保守',.65,.2,.6),('局部高度宽松',.5,.1,.45),('顶面接近支持',.6,.3,.25)];origin=p[:,:2].min(0)-.25;res=.05;wh=np.ceil((p[:,:2].max(0)+.25-origin)/res).astype(int)+1;shape=tuple(wh[::-1]);uv=np.floor((p[:,:2]-origin)/res).astype(int);count=np.zeros(shape,int);np.add.at(count,(uv[:,1],uv[:,0]),1)
 for name,minheight,mincontact,minrelative in variants:
  ids=np.unique(np.concatenate([i for i,h,c,rr in patches if h>=minheight and c>=mincontact and rr>=minrelative])) if any(h>=minheight and c>=mincontact and rr>=minrelative for i,h,c,rr in patches) else np.array([],int);walls=q[ids];np.savez_compressed(OUT/f'L{f+1}_{name}.npz',wall_points=walls,origin=origin,resolution=res);im=np.full((*shape,3),BG,np.uint8);shade=(35+65*np.clip(np.log1p(count)/np.log(60),0,1)).astype('uint8');im[count>0]=shade[count>0,None];xy=np.floor((walls[:,:2]-origin)/res).astype(int);im[xy[:,1],xy[:,0]]=250;pic=Image.fromarray(np.flipud(im).copy());sc=min(850/pic.width,1230/pic.height);pic=pic.resize((round(pic.width*sc),round(pic.height*sc)),Image.Resampling.NEAREST);canvas=Image.new('RGB',(900,1500),BG);canvas.paste(pic,((900-pic.width)//2,150));dr=ImageDraw.Draw(canvas);dr.text((22,20),f'{f+1}层 · {name}',font=font,fill='white');dr.text((22,75),'白：保留的实测墙面点；灰：完整点云',font=font,fill='white');canvas.save(VIS/f'L{f+1}_{name}.png');metrics.append(dict(floor=f+1,mode=name,points=len(walls)));print(metrics[-1],flush=True)
(OUT/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2))
