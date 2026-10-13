from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT.parent/'视频制作/墙体证据组合测试';BASE=ROOT/'outputs/wall_evidence_combinations';cache=ROOT/'outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed';inc=np.load(cache/'increments.npz');raw=np.concatenate([inc[k]for k in inc.files if k.startswith('new_')]);font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',30);small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',22);BG=(18,22,28);names=['A_墙面聚合','B_多帧支持','C_门口证据','D_天花板辅助','E_组合证据'];ground=[-.155,2.595,5.595];tops=[2.495,5.495,8.05]
for f,z in enumerate(ground,1):
 ds=[np.load(BASE/f'{name}_L{f}.npz')for name in names];origin=ds[0]['origin'];res=float(ds[0]['resolution']);shape=ds[0]['wall_candidates'].shape;h,w=shape;p=raw[(raw[:,2]>=z-.05)&(raw[:,2]<tops[f-1])];uv=np.floor((p[:,:2]-origin)/res).astype(int);ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h);p=p[ok];uv=uv[ok];count=np.zeros(shape,np.int32);np.add.at(count,(uv[:,1],uv[:,0]),1);density=np.clip(np.log1p(count)/np.log(150),0,1);shade=(35+75*density).astype('uint8');cloud=np.full((*shape,3),BG,np.uint8);known=count>0;cloud[known]=np.repeat(shade[known,None],3,axis=1)
 union=known|np.logical_or.reduce([d['wall_candidates']for d in ds]);yy,xx=np.where(union);ys=slice(max(0,int(yy.min())-6),min(h,int(yy.max())+7));xs=slice(max(0,int(xx.min())-6),min(w,int(xx.max())+7));board=Image.new('RGB',(5400,1500),BG)
 for j,(name,d) in enumerate([('原始深度点云',None)]+list(zip(names,ds))):
  im=cloud.copy()
  if d is not None:im[d['wall_candidates']]=(250,250,250)
  pic=Image.fromarray(np.flipud(im[ys,xs]).copy());sc=min(830/pic.width,1230/pic.height);pic=pic.resize((round(pic.width*sc),round(pic.height*sc)),Image.Resampling.NEAREST);panel=Image.new('RGB',(900,1500),BG);panel.paste(pic,((900-pic.width)//2,150));dr=ImageDraw.Draw(panel);dr.text((25,20),f'{f}层 · {name}',font=font,fill='white');dr.text((25,78),'灰：原始点云投影；白：结构候选',font=small,fill=(200,210,225));panel.save(OUT/f'L{f}_{name}_原始点云叠加.png');board.paste(panel,(j*900,0))
 board.save(OUT/f'{f}层_原始点云与五方案对比.png')
 # Compact comparison: raw cloud beside current leading candidate D.
 compact=Image.new('RGB',(1800,1500),BG);compact.paste(board.crop((0,0,900,1500)),(0,0));compact.paste(board.crop((3600,0,4500,1500)),(900,0));compact.save(OUT/f'{f}层_原始点云与D方案.png');print('done',f,flush=True)
