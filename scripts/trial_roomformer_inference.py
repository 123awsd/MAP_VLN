"""Official RoomFormer weights/inference with official PyTorch attention fallback."""
import sys,json
from pathlib import Path
import numpy as np,torch
from PIL import Image,ImageDraw,ImageFont
sys.path.insert(0,'/tmp/roomformer_code')
from models import build_model
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/roomformer_trial';OUT.mkdir(exist_ok=True);VIS=ROOT.parent/'视频制作/RoomFormer房间结构';VIS.mkdir(exist_ok=True)
c=torch.load('/tmp/roomformer_stru3d.pth',map_location='cpu',weights_only=False);a=c['args'];a.semantic_classes=-1;a.masked_attn=False
model=build_model(a,train=False);model.load_state_dict(c['model'],strict=True);model.eval().cuda();torch.set_num_threads(4)
g=np.load(ROOT/'outputs/room_quality_ceiling/geometry.npz');inc=np.load(ROOT/'outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed/increments.npz');raw=np.concatenate([inc[k]for k in inc.files if k.startswith('new_')]);font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',26);BG=(18,22,28);records=[]
for f,ground in enumerate([-.155,2.595,5.595]):
 p,n=g[f'p{f}'],g[f'n{f}'];lo=p[:,:2].min(0);hi=p[:,:2].max(0);span=hi-lo;lo=lo-.1*span;hi=hi+.1*span;span=hi-lo
 variants=[('体素密度分位归一',p),('体素密度对数归一',p),('原始密度分位归一',raw[(raw[:,2]>ground-.05)&(raw[:,2]<ground+2.65)])]
 for name,q in variants:
  uv=np.clip(np.round((q[:,:2]-lo)/span*256).astype(int),0,255);density=np.zeros((256,256),np.float32);np.add.at(density,(uv[:,1],uv[:,0]),1);density=(np.log1p(density)/np.log1p(density.max())) if '对数' in name else np.clip(density/max(np.quantile(density[density>0],.95),1),0,1);Image.fromarray((density*255).astype('uint8')).save(OUT/f'L{f+1}_{name}_input.png')
  with torch.inference_mode():pred=model([torch.from_numpy(density[None]).cuda()])
  probs=pred['pred_logits'][0].sigmoid().cpu().numpy();corners=pred['pred_coords'][0].cpu().numpy();polys=[]
  for score,poly in zip(probs,corners):
   poly=poly[score>.5]
   if len(poly)>=4:polys.append(poly)
  mapped=[(poly*span+lo).tolist()for poly in polys];(OUT/f'L{f+1}_{name}.json').write_text(json.dumps({'polygons_xy':mapped,'origin':lo.tolist(),'span':span.tolist()},ensure_ascii=False,indent=2))
  base=np.clip(np.sqrt(density)*150,0,150).astype('uint8');rgb=np.repeat(base[:,:,None],3,axis=2);im=Image.fromarray(np.flipud(rgb).copy()).resize((1024,1024));d=ImageDraw.Draw(im)
  for i,poly in enumerate(polys):
   px=poly*1024;px[:,1]=1024-px[:,1];pts=[tuple(x)for x in px];d.line(pts+[pts[0]],fill='white',width=2);center=px.mean(0);d.text(tuple(center),str(i+1),font=font,fill='white',stroke_width=2,stroke_fill='black')
  panel=Image.new('RGB',(1024,1130),BG);panel.paste(im,(0,106));d=ImageDraw.Draw(panel);d.text((22,20),f'{f+1}层 · {name} · {len(polys)}个房间候选',font=font,fill='white');panel.save(VIS/f'L{f+1}_{name}.png');records.append(dict(floor=f+1,variant=name,rooms=len(polys)));print(records[-1],flush=True)
(OUT/'metrics.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
