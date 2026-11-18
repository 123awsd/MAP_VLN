from pathlib import Path
import subprocess, fitz
from PIL import Image, ImageEnhance, ImageFilter
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[4]
VIDEO=ROOT/'视频/视频制作/整理汇总/原子素材/实验视频/第三视角'
FF='/home/uav/.local/lib/python3.13/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
# Coordinates refer to the 480 x 270 inspection frames; boxes indicate only identifiable targets.
rows=[
('Dormitory',[], '宿舍第三视角拼接.mp4',[(4.7,(325,60,20),[]),(14.0,(255,110,25),[(0,148,101,254)]),(20.3,(150,121,22),[(220,129,234,142)]),(26.0,(304,134,25),[]),(43.0,(200,77,30),[]),(45.3,(285,184,35),[(407,184,479,240)])]),
('Laboratory',[], '实验室三任务短距离第三视角.mp4',[(12.1,(242,132,23),[]),(28.3,(240,161,23),[(111,103,125,133)]),(36.3,(208,123,20),[]),(52.5,(198,166,20),[]),(60.5,(218,160,21),[(103,182,135,214)]),(84.8,(205,130,20),[(302,128,391,236)])]),
('Pool table',[], '实验室台球桌.mp4',[(7.2,(232,118,19),[]),(16.9,(215,119,18),[]),(26.5,(241,121,18),[]),(31.3,(225,115,19),[]),(41.0,(207,127,18),[]),(50.6,(239,153,20),[])]),
('Table',[], '实验室桌子.mp4',[(1.5,(246,211,19),[]),(13.5,(239,146,20),[]),(19.5,(264,156,17),[]),(22.5,(247,164,17),[]),(28.5,(242,202,20),[]),(31.5,(273,201,22),[(165,146,174,163)])])]

# Fig. 8 layout: centered instructions, serif type, bold target nouns, and uniform image strips.
instructions = [
 ['"First check whether the kitchen **pot** is on, then check the **bottled water** under the bedroom table.',
  'Check the **cup** on the living-room refrigerator; if absent, search the living-room table."'],
 ['"Check for the **cup** on the lounge table and the **basketball** under the chair in the activity room;',
  'inspect whether the **trash can** next to the fan in the storage room is full."'],
 ['"Find the **ball** on the pool table in the recreation room."'],
 ['"Find the **cup** on the table in the lounge."'],
]
import re
doc=fitz.open(); page=doc.new_page(width=780,height=525)
yellow=(1,.94,0); green=(0,.67,.27)
lefts=[[0,0,60,60,120,120],[35,45,35,35,35,100],[55,45,60,60,45,55],[60,60,65,65,35,35]]
y=10
def centered_markup(line, baseline):
 parts=re.split(r'(\*\*.*?\*\*)',line); size=11.0
 def font(part): return 'tibo' if part.startswith('**') else 'tiro'
 def label(part): return part[2:-2] if part.startswith('**') else part
 total=sum(fitz.get_text_length(label(p),fontname=font(p),fontsize=size) for p in parts if p)
 x=(780-total)/2
 for part in parts:
  if part:
   content=label(part); f=font(part); page.insert_text((x,baseline),content,fontname=f,fontsize=size,color=(0,0,0)); x+=fitz.get_text_length(content,fontname=f,fontsize=size)
for ri,(_,_,video,panels) in enumerate(rows):
 lines=instructions[ri]
 for line in lines:
  centered_markup(line,y+10); y+=12
 y+=2
 for j,(t,(cx,cy,r),boxes) in enumerate(panels):
  img=BASE/'frames'/f'six_{ri}_{j}_{t}.jpg'
  if not img.exists(): subprocess.run([FF,'-hide_banner','-loglevel','error','-ss',str(t),'-i',str(VIDEO/video),'-frames:v','1','-q:v','2','-y',str(img)],check=True,stdin=subprocess.DEVNULL)
  x=12+j*127; w=120; h=90; scale=w/360; left=lefts[ri][j]
  upper=0
  cropped=BASE/'frames'/f'compact_layout_{ri}_{j}_{t}_{left}.jpg'
  if not cropped.exists():
   original=Image.open(img); k=original.width/480
   crop=original.crop((int(left*k),int(upper*k),int((left+360)*k),int((upper+270)*k))).convert('RGB')
   # Lift shadows gently while preserving white highlights, then apply restrained sharpening.
   gamma=[0.70, 0.75, 0.68, 0.67][ri]
   lut=[round(255*(v/255)**gamma) for v in range(256)]
   crop=crop.point(lut*3)
   crop=ImageEnhance.Contrast(crop).enhance(1.02)
   crop=crop.filter(ImageFilter.UnsharpMask(radius=1.2,percent=55,threshold=3))
   crop.save(cropped,quality=97,subsampling=0)
  page.insert_image(fitz.Rect(x,y,x+w,y+h),filename=str(cropped))
  shape=page.new_shape(); shape.draw_circle(fitz.Point(x+(cx-left)*scale,y+(cy-upper)*scale),r*scale)
  shape.finish(color=yellow,width=1.4,dashes='[4 2] 0'); shape.commit()
  for a,b,c,d in boxes: page.draw_rect(fitz.Rect(x+(a-left)*scale,y+(b-upper)*scale,x+(c-left)*scale,y+(d-upper)*scale),color=green,width=1.5)
 y+=94
# Crop the PDF page to the artwork.
page.set_cropbox(fitz.Rect(0,0,780,y))
out=BASE/'real_world_fig8_style.pdf'; doc.save(out,deflate=True)
page.get_pixmap(matrix=fitz.Matrix(3,3),alpha=False).save(BASE/'real_world_fig8_style.png')
print(out)
