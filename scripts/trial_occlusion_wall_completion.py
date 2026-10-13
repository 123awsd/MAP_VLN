"""Recover wall footprint using multi-height planes and depth occlusion evidence."""
from pathlib import Path
import json,numpy as np,cv2
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'outputs/room_quality_ceiling';OUT=ROOT/'outputs/wall_occlusion_completion';OUT.mkdir(exist_ok=True);VIS=ROOT.parent/'视频制作/家具遮挡墙面恢复';VIS.mkdir(exist_ok=True)
g=np.load(BASE/'geometry.npz');r=np.load(OUT/'fullheight_rays.npz');BG=(18,22,28);font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',28);small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',20);metrics=[]
for f,ground in enumerate([-.155,2.595,5.595]):
 base=np.load(BASE/f'D_墙面方向统一_L{f+1}.npz');org=base['origin'];res=float(base['resolution']);shape=base['wall'].shape;p,n=g[f'p{f}'],g[f'n{f}'];uv=np.floor((p[:,:2]-org)/res).astype(int);counts=np.zeros(shape,np.int32);np.add.at(counts,(uv[:,1],uv[:,0]),1);cloud=np.full((*shape,3),BG,np.uint8);shade=(35+65*np.clip(np.log1p(counts)/np.log(60),0,1)).astype('uint8');cloud[counts>0]=shade[counts>0,None]
 records=json.loads((ROOT/'outputs/wall_evidence_combinations'/f'L{f+1}_planes.json').read_text());masks=[np.zeros(shape,np.uint8)for _ in range(3)];evidence=[]
 for rec in records:
  normal=np.array(rec['normal_xy']);tangent=np.array([-normal[1],normal[0]]);off=rec['offset_m'];lo=rec['along_origin_m'];q=p[(abs(p[:,:2]@normal-off)<.08)&(abs(n[:,2])<.35)&(p[:,2]>ground+.3)];t=q[:,:2]@tangent;nt=int(np.ceil((t.max()-lo)/.1))+1
  if nt<3:continue
  hit=np.zeros((14,nt),bool);ti=np.floor((t-lo)/.1).astype(int);zi=np.floor((q[:,2]-ground)/.2).astype(int);ok=(ti>=0)&(ti<nt)&(zi>=0)&(zi<14);hit[zi[ok],ti[ok]]=1
  free=np.zeros_like(hit,dtype=np.uint16);occ=np.zeros_like(free)
  for i,pose in enumerate(r['poses']):
   start=pose[1:4]
   if not ground-.1<start[2]<ground+2.8:continue
   delta=r[f'r{i:04d}']-start;den=delta[:,:2]@normal;u=np.divide(off-start[:2]@normal,den,out=np.full(len(den),np.nan),where=abs(den)>1e-6);dist=np.linalg.norm(delta,axis=1);valid=(u>0)&(u*dist<6);xyz=start+u[:,None]*delta;tt=np.floor((xyz[:,:2]@tangent-lo)/.1).astype(int);zz=np.floor((xyz[:,2]-ground)/.2).astype(int);valid&=(tt>=0)&(tt<nt)&(zz>=0)&(zz<14)
   for grid,cond in [(free,u<1-.15/np.maximum(dist,.15)),(occ,u>1+.15/np.maximum(dist,.15))]:
    v=valid&cond;seen=np.zeros_like(hit);seen[zz[v],tt[v]]=1;grid+=seen
  # Multi-height evidence permits a high patch without four occupied height bins.
  upper=hit[7:].sum(0)>=2;full=(hit.sum(0)>=4)&hit[7:].any(0);anchor=upper|full
  anchor=cv2.morphologyEx(anchor.astype('uint8')[None,:],cv2.MORPH_CLOSE,np.ones((1,3),np.uint8))[0]>0
  # Reject short isolated patches (possible cabinet surfaces), retain runs >= 0.6m.
  clean=np.zeros(nt,bool);d=np.diff(np.r_[False,anchor,False].astype(int))
  for a,b in zip(np.where(d==1)[0],np.where(d==-1)[0]):
   if b-a>=6:clean[a:b]=1
  passthrough=(free[2:10]>=3).sum(0)>=3
  blocked=(occ[2:10]>=2).sum(0)>=2
  filled=clean.copy();d=np.diff(np.r_[False,~clean,False].astype(int));added=np.zeros(nt,bool)
  for a,b in zip(np.where(d==1)[0],np.where(d==-1)[0]):
   if a==0 or b==nt:continue
   # Extend only between measured portions of the same plane; require occlusion.
   if (b-a)*.1<=2.5 and blocked[a:b].mean()>=.6 and not passthrough[a:b].any():filled[a:b]=1;added[a:b]=1
  door=np.zeros(nt,bool);candidate=passthrough&(hit[2:10].sum(0)<=1)
  d=np.diff(np.r_[False,candidate,False].astype(int))
  for a,b in zip(np.where(d==1)[0],np.where(d==-1)[0]):
   if .5<=(b-a)*.1<=1.5 and a>0 and b<nt and clean[max(0,a-4):a].any() and clean[b:min(nt,b+4)].any():door[a:b]=1
  physical=filled&~door
  for j,opt in enumerate([clean,filled,physical]):
   for k in np.where(opt)[0]:
    ends=normal*off+np.array([lo+k*.1,lo+(k+1)*.1])[:,None]*tangent;xy=np.floor((ends-org)/res).astype(int);cv2.line(masks[j],tuple(xy[0]),tuple(xy[1]),1,1)
  evidence.append(dict(id=rec['id'],normal_xy=normal.tolist(),offset_m=off,observed_columns=int(clean.sum()),occlusion_completed_columns=int(added.sum()),pass_through_columns=int(door.sum())))
  np.savez_compressed(OUT/f'L{f+1}_plane{rec["id"]:02d}.npz',hit=hit,free_frames=free,occluded_frames=occ,observed_columns=clean,completed_columns=added,opening_columns=door,normal=normal,offset=off,along_origin=lo)
 yy,xx=np.where(counts>0);ys=slice(max(0,yy.min()-6),min(shape[0],yy.max()+7));xs=slice(max(0,xx.min()-6),min(shape[1],xx.max()+7));board=Image.new('RGB',(3600,1500),BG)
 opts=[('原始点云',np.zeros(shape,bool)),('多高度墙面支持',masks[0]>0),('遮挡证据补全',masks[1]>0),('补全并保留开口',masks[2]>0)]
 for j,(name,mask)in enumerate(opts):
  im=cloud.copy();im[mask]=250;pic=Image.fromarray(np.flipud(im[ys,xs]).copy());scale=min(830/pic.width,1220/pic.height);pic=pic.resize((round(pic.width*scale),round(pic.height*scale)),Image.Resampling.NEAREST);panel=Image.new('RGB',(900,1500),BG);panel.paste(pic,((900-pic.width)//2,160));dr=ImageDraw.Draw(panel);dr.text((24,20),f'{f+1}层 · {name}',font=font,fill='white');dr.text((24,75),'灰：原始点云；白：候选建筑边界',font=small,fill=(190,202,216));panel.save(VIS/f'{f+1}层_{name}.png');board.paste(panel,(900*j,0))
  if j:np.savez_compressed(OUT/f'L{f+1}_{name}.npz',wall_candidates=mask,origin=org,resolution=res)
 board.save(VIS/f'{f+1}层_恢复对比.png');(OUT/f'L{f+1}_evidence.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2));metrics.append({'floor':f+1,'planes':len(evidence),'occlusion_completed_length_m':sum(x['occlusion_completed_columns']for x in evidence)*.1,'opening_candidate_length_m':sum(x['pass_through_columns']for x in evidence)*.1});print(metrics[-1],flush=True)
(OUT/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2))
