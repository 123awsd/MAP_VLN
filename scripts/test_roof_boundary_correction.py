"""Test roof boundaries as hints rather than physical walls. Preserve measured coordinates."""
from pathlib import Path
import numpy as np,cv2,json
from scipy.ndimage import distance_transform_edt
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'outputs/room_quality_ceiling';OUT=ROOT/'outputs/roof_boundary_correction';OUT.mkdir(exist_ok=True);VIS=ROOT.parent/'视频制作/墙体边界优化对比';VIS.mkdir(exist_ok=True);g=np.load(BASE/'geometry.npz');font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',28);small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',21);BG=(18,22,28);records=[]
for f,z in enumerate([-.155,2.595,5.595]):
 d=np.load(BASE/f'D_墙面方向统一_L{f+1}.npz');org=d['origin'];res=float(d['resolution']);h,w=d['wall'].shape;p,n=g[f'p{f}'],g[f'n{f}'];uv=np.floor((p[:,:2]-org)/res).astype(int)
 counts=np.zeros((h,w),np.int32);np.add.at(counts,(uv[:,1],uv[:,0]),1);shade=(40+70*np.clip(np.log1p(counts)/np.log(60),0,1)).astype('uint8');cloud=np.full((h,w,3),BG,np.uint8);known=counts>0;cloud[known]=np.repeat(shade[known,None],3,axis=1)
 # Measured vertical patches at multiple heights; no roof-coordinate substitution.
 bins=np.zeros((14,h,w),np.uint8);rel=p[:,2]-z;vertical=(abs(n[:,2])<.35)&(rel>.3)&(rel<2.7)
 for b in range(14):
  xy=uv[vertical&(rel>=b*.2)&(rel<(b+1)*.2)];bins[b,xy[:,1],xy[:,0]]=1
 votes=sum(cv2.dilate(b,np.ones((3,3),np.uint8))for b in bins);direct=bins.any(0);high=bins[7:].any(0);strong=(votes>=4)&direct&(cv2.dilate(high.astype('uint8'),np.ones((3,3),np.uint8))>0)
 roof=np.load(BASE/f'E_天花板边界辅助_L{f+1}.npz');shell=roof['envelope_boundary'];distance,idx=distance_transform_edt(~strong,return_indices=True);near=shell&(distance*res<=.35);snapped=np.zeros((h,w),np.uint8);yy,xx=np.where(near);snapped[idx[0,yy,xx],idx[1,yy,xx]]=1
 # Snap every accepted hint to an observed vertical point; reject unsupported roof rim.
 snapped=cv2.morphologyEx(snapped,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8)).astype(bool)&(cv2.dilate(strong.astype('uint8'),np.ones((3,3),np.uint8))>0)
 roof_dist=distance_transform_edt(~roof['roof_observed'])*res;roof_verified=strong&(roof_dist<=.5)
 opts=[('原始点云',np.zeros_like(strong)),('旧版_天花板轮廓',shell|d['wall']),('方案1_观测墙面',strong),('方案2_边界回归墙面',d['wall']|snapped),('方案3_墙顶联合支持',roof_verified)]
 yy,xx=np.where(known);ys=slice(max(0,yy.min()-6),min(h,yy.max()+7));xs=slice(max(0,xx.min()-6),min(w,xx.max()+7));board=Image.new('RGB',(4500,1500),BG)
 for j,(name,mask) in enumerate(opts):
  np.savez_compressed(OUT/f'L{f+1}_{name}.npz',wall_candidates=mask,origin=org,resolution=res);im=cloud.copy();im[mask]=(250,250,250);pic=Image.fromarray(np.flipud(im[ys,xs]).copy());sc=min(830/pic.width,1230/pic.height);pic=pic.resize((round(pic.width*sc),round(pic.height*sc)),Image.Resampling.NEAREST);panel=Image.new('RGB',(900,1500),BG);panel.paste(pic,((900-pic.width)//2,150));dr=ImageDraw.Draw(panel);dr.text((25,20),f'{f+1}层 · {name}',font=font,fill='white');dr.text((25,78),'灰：完整观测点云；白：候选墙体',font=small,fill=(190,202,216));panel.save(VIS/f'{f+1}层_{name}.png');board.paste(panel,(j*900,0))
 board.save(VIS/f'{f+1}层_三方案与原始点云对比.png');records.append({'floor':f+1,'roof_rim_cells':int(shell.sum()),'roof_rim_with_vertical_support_within_35cm':int(near.sum()),'rejected_roof_rim_cells':int((shell&~near).sum()),'snapped_hint_cells':int(snapped.sum()),'measured_vertical_wall_cells':int(strong.sum())})
(OUT/'metrics.json').write_text(json.dumps(records,ensure_ascii=False,indent=2));print(records)
