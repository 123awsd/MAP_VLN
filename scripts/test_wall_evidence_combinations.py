"""Compare measured wall-plane aggregation, temporal support, openings and roof constraints.
All outputs are diagnostic candidates, not a replacement collision map.
"""
from pathlib import Path
import json
import numpy as np,cv2
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'outputs/room_quality_ceiling';OUT=ROOT/'outputs/wall_evidence_combinations';OUT.mkdir(exist_ok=True);VIS=ROOT.parent/'视频制作/墙体证据组合测试';VIS.mkdir(exist_ok=True)
geo=np.load(BASE/'geometry.npz');cache=ROOT/'outputs/exploration_recovery_local_trial/round6_extended';inc=np.load(cache/'preview_trimmed/increments.npz');rays=np.load(cache/'measured_free_rays.npz');fontpath='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc';font=ImageFont.truetype(fontpath,29);small=ImageFont.truetype(fontpath,20);BG=(18,22,28);metrics=[]
names=['A_墙面聚合','B_多帧支持','C_门口证据','D_天花板辅助','E_组合证据']
for f,ground in enumerate([-.155,2.595,5.595]):
 base=np.load(BASE/f'D_墙面方向统一_L{f+1}.npz');org=base['origin'];res=float(base['resolution']);shape=base['wall'].shape;h,w=shape;p,n=geo[f'p{f}'],geo[f'n{f}'];v=(abs(n[:,2])<.3)&(p[:,2]>ground+.3);q=p[v];nn=n[v,:2];nn=nn/np.maximum(np.linalg.norm(nn,axis=1,keepdims=True),1e-8);angle=np.angle(np.mean(np.exp(4j*np.arctan2(nn[:,1],nn[:,0]))))/4;planes=[]
 for theta in [angle,angle+np.pi/2]:
  normal=np.array([np.cos(theta),np.sin(theta)]);tangent=np.array([-normal[1],normal[0]]);qq=q[abs(nn@normal)>.97];offs=qq[:,:2]@normal;keys,cnt=np.unique(np.round(offs/.05).astype(int),return_counts=True);accepted=[]
  for key in keys[np.argsort(cnt)[::-1]]:
   off=key*.05
   if any(abs(off-x)<.18 for x in accepted):continue
   pts=qq[abs(offs-off)<.07];z=pts[:,2]-ground
   if len(pts)<100 or np.ptp(z)<1.2 or z.max()<1.6:continue
   t=pts[:,:2]@tangent;lo=np.floor(t.min()/.1)*.1;nt=int(np.ceil((t.max()-lo)/.1))+1
   if nt<7:continue
   hit=np.zeros((14,nt),bool);ti=np.clip(np.floor((t-lo)/.1).astype(int),0,nt-1);zi=np.clip(np.floor(z/.2).astype(int),0,13);hit[zi,ti]=True
   if np.sum(hit.any(0))<7:continue
   accepted.append(off);planes.append(dict(normal=normal,tangent=tangent,offset=off,lo=lo,hit=hit,frames=np.zeros_like(hit,dtype=np.uint16),free=np.zeros_like(hit,dtype=np.uint16)))
 # Temporal support counts exported time bins; each bin counted once per wall-face cell.
 for key in inc.files:
  if not key.startswith('new_'):continue
  pts=inc[key];pts=pts[(pts[:,2]>ground+.25)&(pts[:,2]<ground+2.65)][::4]
  for plane in planes:
   good=abs(pts[:,:2]@plane['normal']-plane['offset'])<.07;qq=pts[good]
   if not len(qq):continue
   ti=np.floor((qq[:,:2]@plane['tangent']-plane['lo'])/.1).astype(int);zi=np.floor((qq[:,2]-ground)/.2).astype(int);ok=(ti>=0)&(ti<plane['hit'].shape[1])&(zi>=0)&(zi<14);frame=np.zeros_like(plane['hit']);frame[zi[ok],ti[ok]]=True;plane['frames']+=frame
 # Free rays are tested against the wall plane in 3D; never flatten their height.
 for ri,pose in enumerate(rays['poses']):
  if not ground+.35<pose[3]<ground+2.3:continue
  ends=rays[f'r{ri:04d}'];start=pose[1:4];delta=ends-start
  for plane in planes:
   denom=delta[:,:2]@plane['normal'];u=np.divide(plane['offset']-start[:2]@plane['normal'],denom,out=np.full(len(denom),np.nan),where=abs(denom)>1e-6);dist=np.linalg.norm(delta,axis=1);ok=(u>0)&(u<1-.15/np.maximum(dist,.15));xyz=start+u[ok,None]*delta[ok];ti=np.floor((xyz[:,:2]@plane['tangent']-plane['lo'])/.1).astype(int);zi=np.floor((xyz[:,2]-ground)/.2).astype(int);ok=(ti>=0)&(ti<plane['hit'].shape[1])&(zi>=0)&(zi<14);frame=np.zeros_like(plane['hit']);frame[zi[ok],ti[ok]]=True;plane['free']+=frame
 roof=np.load(BASE/f'E_天花板边界辅助_L{f+1}.npz');masks=[np.zeros(shape,np.uint8)for _ in names];records=[]
 for i,plane in enumerate(planes):
  hit=plane['hit'];temporal=(plane['frames']>=3)&hit;strong=(hit.sum(0)>=4)&hit[7:].any(0);stable=strong&(temporal.sum(0)>=3)&temporal[7:].any(0)
  # Only a vertically extended, observed opening can break the room-boundary hypothesis.
  free=plane['free']>=4;door=(free[2:9].sum(0)>=3)&(hit[2:9].sum(0)<=1)
  runs=np.diff(np.r_[False,door,False].astype(int));validdoor=np.zeros_like(door)
  for a,b in zip(np.where(runs==1)[0],np.where(runs==-1)[0]):
   if .5<=(b-a)*.1<=1.5:validdoor[a:b]=True
  options=[strong,stable,strong&~validdoor,strong,stable&~validdoor]
  for j,option in enumerate(options):
   if j in [0,3]:option=cv2.morphologyEx(option.astype('uint8')[None,:],cv2.MORPH_CLOSE,np.ones((1,3),np.uint8))[0]>0
   changes=np.diff(np.r_[False,option,False].astype(int));supported=np.zeros_like(option)
   for aa,bb in zip(np.where(changes==1)[0],np.where(changes==-1)[0]):
    if (bb-aa)*.1>=.5:supported[aa:bb]=True
   for k in np.where(supported)[0]:
    endpoints=np.array([plane['normal']*plane['offset']+plane['tangent']*(plane['lo']+k*.1),plane['normal']*plane['offset']+plane['tangent']*(plane['lo']+(k+1)*.1)]);xy=np.floor((endpoints-org)/res).astype(int);cv2.line(masks[j],tuple(xy[0]),tuple(xy[1]),1,1)
  # Wall-face diagnostic: white hit, cyan repeated hits, red measured free crossings.
  face=np.full((*hit.shape,3),BG,np.uint8);face[hit]=(232,239,247);face[temporal]=(110,213,221);face[free&~hit]=(235,99,99);face[free&hit]=(230,186,88);face=Image.fromarray(np.flipud(face).copy());face=face.resize((min(1800,max(500,hit.shape[1]*12)),350),Image.Resampling.NEAREST);canvas=Image.new('RGB',(face.width,450),BG);canvas.paste(face,(0,90));dr=ImageDraw.Draw(canvas);dr.text((12,10),f'{f+1}层 · 墙面候选{i+1} · 横轴沿墙/纵轴高度',font=small,fill='white');dr.text((12,45),'白：表面点；青：多帧支持；红：射线穿过；黄：证据冲突',font=small,fill='white');canvas.save(VIS/f'L{f+1}_墙面{i+1:02d}_正视证据.png')
  records.append({'id':i+1,'normal_xy':plane['normal'].tolist(),'offset_m':float(plane['offset']),'along_origin_m':float(plane['lo']),'temporal_cells':int(temporal.sum()),'free_cells':int(free.sum()),'door_columns':int(validdoor.sum())})
 # Keep roof-derived boundary separately; it is inferred envelope, not observed physical wall.
 for j in [3,4]:masks[j]|=roof['envelope_boundary'].astype('uint8')
 board=Image.new('RGB',(4500,1500),BG);known=base['domain']|base['wall'];yy,xx=np.where(known);ys=slice(max(0,yy.min()-6),min(h,yy.max()+7));xs=slice(max(0,xx.min()-6),min(w,xx.max()+7))
 for j,name in enumerate(names):
  mask=masks[j]>0;np.savez_compressed(OUT/f'{name}_L{f+1}.npz',wall_candidates=mask,origin=org,resolution=res,envelope_boundary=roof['envelope_boundary'] if j>=3 else np.zeros(shape,bool));im=np.full((*shape,3),BG,np.uint8);im[mask]=(235,242,250);pic=Image.fromarray(np.flipud(im[ys,xs]).copy());sc=min(830/pic.width,1250/pic.height);pic=pic.resize((round(pic.width*sc),round(pic.height*sc)),Image.Resampling.NEAREST);panel=Image.new('RGB',(900,1500),BG);panel.paste(pic,((900-pic.width)//2,150));dr=ImageDraw.Draw(panel);dr.text((25,20),f'{f+1}层 · {name}',font=font,fill='white');dr.text((25,78),'诊断候选 · 未替换正式地图',font=small,fill=(190,201,213));panel.save(VIS/f'L{f+1}_{name}.png');board.paste(panel,(j*900,0));ncomp,lab,stats,_=cv2.connectedComponentsWithStats(mask.astype('uint8'),8);metrics.append({'floor':f+1,'method':name,'mask_area_m2':float(mask.sum()*res**2),'components':ncomp-1,'short_components':int(sum(max(x[2],x[3])*res<.5 for x in stats[1:]))})
 board.save(VIS/f'{f+1}层_五种证据对比.png');(OUT/f'L{f+1}_planes.json').write_text(json.dumps(records,ensure_ascii=False,indent=2));print('DONE',f+1,len(planes),flush=True)
(OUT/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2))
