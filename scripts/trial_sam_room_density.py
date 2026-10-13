"""Automatic SAM2 masks on measured point density; inspired by FloorSAM, not a reproduction."""
from pathlib import Path
import numpy as np,torch,cv2,json
from PIL import Image,ImageDraw,ImageFont
from transformers import pipeline
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/sam_density_rooms';OUT.mkdir(exist_ok=True);VIS=ROOT.parent/'视频制作/SAM整体房间候选';VIS.mkdir(exist_ok=True);modeldir='/home/uav/.cache/huggingface/hub/models--facebook--sam2.1-hiera-large/snapshots/665f8e2ad61cf5f53d65644ff27c8ee525124610';generator=pipeline('mask-generation',model=modeldir,device=0);torch.set_num_threads(4);g=np.load(ROOT/'outputs/room_quality_ceiling/geometry.npz');font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',26);small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',20);records=[]
for f,ground in enumerate([-.155,2.595,5.595]):
 p,n=g[f'p{f}'],g[f'n{f}'];org=p[:,:2].min(0)-.3;span=p[:,:2].max(0)+.3-org;res=max(span)/950;wh=np.ceil(span/res).astype(int)+1;shape=tuple(wh[::-1]);uv=np.floor((p[:,:2]-org)/res).astype(int);full=np.zeros(shape,np.float32);np.add.at(full,(uv[:,1],uv[:,0]),1)
 for name,use in [('空间密度',(p[:,2]>ground+.3)&(p[:,2]<ground+2.4))]:
  density=np.zeros(shape,np.float32);xy=uv[use];np.add.at(density,(xy[:,1],xy[:,0]),1);density=cv2.GaussianBlur(density,(9,9),1.5)
  gray=(255*np.clip(np.log1p(density)/np.log1p(max(density.max(),1)),0,1)).astype('uint8');im=Image.fromarray(np.flipud(gray).copy()).convert('RGB');im.save(OUT/f'L{f+1}_{name}_input.png')
  out=generator(im,points_per_batch=16,points_per_crop=32,pred_iou_thresh=.8,stability_score_thresh=.9,crops_n_layers=0);masks=np.array(out['masks'],dtype=bool);scores=np.array(out['scores']);keep=[]
  for i in np.argsort(scores)[::-1]:
   mask=masks[i];area=mask.sum()*res**2
   if not 1.5<area<100:continue
   if any((mask&prev).sum()/max((mask|prev).sum(),1)>.6 for prev in keep):continue
   # Reject masks mainly covering unobserved exterior.
   if (mask&np.flipud(full>0)).sum()/max(mask.sum(),1)<.55:continue
   keep.append(mask)
  np.savez_compressed(OUT/f'L{f+1}_{name}.npz',masks=np.array(keep),all_masks=masks,scores=scores,origin=org,resolution=res)
  canvas=im.copy();d=ImageDraw.Draw(canvas)
  for j,mask in enumerate(keep):
   contours,_=cv2.findContours(mask.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
   for c in contours:
    pts=[tuple(x)for x in c.reshape(-1,2)]
    if len(pts)>2:d.line(pts+[pts[0]],fill='white',width=1)
   y,x=np.where(mask);d.text((float(x.mean()),float(y.mean())),str(j+1),font=small,fill='white',stroke_width=2,stroke_fill='black')
  panel=Image.new('RGB',(max(canvas.width,900),canvas.height+120),(18,22,28));panel.paste(canvas,((panel.width-canvas.width)//2,120));d=ImageDraw.Draw(panel);d.text((22,20),f'{f+1}层 · {name} · SAM自动区域候选',font=font,fill='white');d.text((22,66),f'{len(keep)}个候选，允许重叠，尚未等同房间',font=small,fill='white');panel.save(VIS/f'L{f+1}_{name}.png');records.append(dict(floor=f+1,mode=name,candidates=len(keep),all_candidates=len(masks)));print(records[-1],flush=True)
(OUT/'metrics.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
