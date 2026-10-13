from pathlib import Path
import numpy as np,json,cv2
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];cache=ROOT/'outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed';inc=np.load(cache/'increments.npz');p=np.concatenate([inc[k]for k in inc.files if k.startswith('new_')]);VIS=ROOT.parent/'视频制作/墙体提取_视频视图对照';VIS.mkdir(exist_ok=True);font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',26);BG=(255,255,255);ground=[-.155,2.595,5.595];base=[-.2,2.6,5.4];upper=[2.05,4.85,7.05]
for f in range(3):
 q=p[(p[:,2]>base[f]+.2)&(p[:,2]<upper[f])];wall=np.load(ROOT/'outputs/wall_patch_context'/f'L{f+1}_局部高度宽松.npz')['wall_points'];lo=q[:,:2].min(0);hi=q[:,:2].max(0);scale=min(1180/(hi-lo)[0],760/(hi-lo)[1]);center=(lo+hi)/2;W,H=1290,850
 def project(points):return np.round((points[:,:2]-center)*scale+[W/2,H/2]).astype(int)
 def raster(points):
  im=np.full((H,W,3),BG,np.uint8);uv=project(points);ok=(uv[:,0]>=0)&(uv[:,0]<W)&(uv[:,1]>=0)&(uv[:,1]<H);uv,points=uv[ok],points[ok];addr=uv[:,1]*W+uv[:,0];order=np.lexsort((-points[:,2],addr));ids=order[np.r_[addr[order][:-1]!=addr[order][1:],True]];uv=uv[ids];height=np.clip((points[ids,2]-ground[f])/2.05,0,1);shade=(190+30*height).astype('uint8');im[uv[:,1],uv[:,0]]=shade[:,None];return im
 clean=raster(q);overlay=clean.copy();uv=project(wall);ok=(uv[:,0]>=0)&(uv[:,0]<W)&(uv[:,1]>=0)&(uv[:,1]<H);uv=uv[ok];wallmask=np.zeros((H,W),np.uint8);wallmask[uv[:,1],uv[:,0]]=1;wallmask=cv2.dilate(wallmask,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)));overlay[wallmask>0]=(155,20,30)
 board=Image.new('RGB',(2580,970),BG)
 for j,(name,im)in enumerate([('视频同款高度裁剪与像素投影',clean),('同一底图叠加墙面候选',overlay)]):
  panel=Image.new('RGB',(W,970),BG);panel.paste(Image.fromarray(im),(0,110));d=ImageDraw.Draw(panel);d.text((24,20),f'{f+1}层 · {name}',font=font,fill=(30,30,30));d.text((24,65),'灰：视频显示范围；深红：全高度墙面候选',font=font,fill=(70,70,70));panel.save(VIS/f'L{f+1}_{j}.png');board.paste(panel,(j*W,0))
 board.save(VIS/f'L{f+1}_显示对照.png')
