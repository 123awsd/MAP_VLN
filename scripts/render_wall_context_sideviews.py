from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/wall_patch_context';VIS=ROOT.parent/'视频制作/墙面片段结构验证';g=np.load(ROOT/'outputs/room_quality_ceiling/geometry.npz');font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',24)
for f in range(3):
 p=g[f'p{f}'];q=np.load(OUT/f'L{f+1}_局部高度宽松.npz')['wall_points'];canvas=Image.new('RGB',(1600,650),(18,22,28));d=ImageDraw.Draw(canvas)
 for j,axis in enumerate([0,1]):
  coords=[axis,2];lo=p[:,coords].min(0);span=p[:,coords].max(0)-lo;sc=min(750/span[0],480/span[1]);im=np.full((520,780,3),(18,22,28),np.uint8)
  for points,color in [(p,(65,65,65)),(q,(245,245,245))]:
   xy=((points[:,coords]-lo)*sc).astype(int);x=np.clip(xy[:,0],0,779);y=np.clip(510-xy[:,1],0,519);im[y,x]=color
  canvas.paste(Image.fromarray(im),(j*800+10,95));d.text((j*800+15,20),f'{f+1}层 · '+('X-Z侧视'if axis==0 else 'Y-Z侧视'),font=font,fill='white')
 canvas.save(VIS/f'L{f+1}_局部高度宽松_双侧视.png')
