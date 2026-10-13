from pathlib import Path
import numpy as np,cv2,json
from scipy.ndimage import binary_fill_holes
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/sam_density_rooms';VIS=ROOT.parent/'视频制作/SAM整体房间候选';g=np.load(ROOT/'outputs/room_quality_ceiling/geometry.npz');font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',21);records=[]
for file in sorted(OUT.glob('L*.npz')):
 d=np.load(file);f=int(file.stem[1]);res=float(d['resolution']);masks=d['all_masks'];scores=d['scores'];org=d['origin'];shape=masks.shape[1:];p=g[f'p{f-1}'];xy=np.floor((p[:,:2]-org)/res).astype(int);count=np.zeros(shape,np.int32);np.add.at(count,(xy[:,1],xy[:,0]),1);domain=binary_fill_holes(cv2.morphologyEx((count>0).astype('uint8'),cv2.MORPH_CLOSE,np.ones((9,9),np.uint8)));domain=np.flipud(domain);keep=[]
 for i in np.argsort(scores)[::-1]:
  m=masks[i];area=m.sum()*res**2
  if not 1.5<area<120:continue
  if (m&domain).sum()/max(m.sum(),1)<.85:continue
  if any((m&v).sum()/max((m|v).sum(),1)>.65 for v in keep):continue
  keep.append(m)
 np.savez_compressed(OUT/(file.stem+'_audited.npz'),masks=np.array(keep),origin=org,resolution=res)
 im=Image.open(OUT/(file.stem+'_input.png')).convert('RGB');dr=ImageDraw.Draw(im)
 for i,m in enumerate(keep):
  contours,_=cv2.findContours(m.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  for c in contours:
   pts=[tuple(v)for v in c.reshape(-1,2)]
   if len(pts)>2:dr.line(pts+[pts[0]],fill='white',width=1)
  y,x=np.where(m);dr.text((x.mean(),y.mean()),str(i+1),font=font,fill='white',stroke_width=2,stroke_fill='black')
 im.save(VIS/(file.stem+'_范围筛选.png'));records.append(dict(input=file.stem,candidates=len(keep),all_masks=len(masks)));print(records[-1])
(OUT/'audited_metrics.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
