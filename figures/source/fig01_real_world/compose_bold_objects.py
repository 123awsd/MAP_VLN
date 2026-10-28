from pathlib import Path
from PIL import Image,ImageDraw,ImageFont,ImageOps
import base64,html,io
r=Path(__file__).resolve().parent;o=r/'logicfly_bold_objects_figure';o.mkdir(exist_ok=True)
W,H=3200,1980;im=Image.new('RGB',(W,H),'white');d=ImageDraw.Draw(im)
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="{W}" height="{H}"><rect width="100%" height="100%" fill="white"/>']
font='/usr/share/fonts/truetype/liberation/LiberationSerif-Regular.ttf'
def text(x,y,s,size=26,color='#101010'):
 bold=s.startswith('Subgoal') or s=='Semantic recovery' or s.startswith('One instruction')
 face=font.replace('-Regular.ttf','-Bold.ttf') if bold else font
 d.text((x,y),s,font=ImageFont.truetype(face,size),fill=color);svg.append(f'<text x="{x}" y="{y+size}" font-family="Liberation Serif,Times New Roman,serif" font-size="{size}" font-weight="{"bold" if bold else "normal"}" fill="{color}">{html.escape(s)}</text>')
def pic(p,x,y,w,h,crop=False):
 a=Image.open(p).convert('RGB')
 if not crop:
  import numpy as np
  px=np.asarray(a).copy();mask=np.min(px,axis=2)<245
  px[mask]=np.clip((px[mask].astype(float)-255)*1.45+255,0,255).astype('uint8')
  a=Image.fromarray(px)
  from PIL import ImageChops
  box=ImageChops.difference(a,Image.new('RGB',a.size,'white')).getbbox()
  if box:a=a.crop(box)
 if crop:a=ImageOps.fit(a,(w,h),Image.Resampling.LANCZOS)
 else:
  a.thumbnail((w,h),Image.Resampling.LANCZOS);x+=(w-a.width)//2;y+=(h-a.height)//2
 im.paste(a,(x,y));b=io.BytesIO();a.save(b,format='PNG');enc=base64.b64encode(b.getvalue()).decode();svg.append(f'<image x="{x}" y="{y}" width="{a.width}" height="{a.height}" xlink:href="data:image/png;base64,{enc}"/>')
rows=[('01_laptop','FPV 00:23–00:49',['Check the laptop','on the lounge table.','If absent, find it.'],[24,35,37,44,46]),('02_toolbox','FPV 00:49–00:54',['Verify the toolbox','on the top shelf','in the storage room.'],[46,49,51,52,54]),('03_microwave','FPV 01:00–01:27',['Check whether the','corn in the tea-room','microwave has','finished heating.'],[54,60,74,82,84]),('04_cup','FPV 01:27–01:42',['Check for the cup','under the pool table','in the recreation room.'],[84,93,98,100,102])]
# A larger platform inset and a conversational instruction block.
pic(r/'hardware_assets/fig1_logicfly_crop.png',25,40,570,340,False)
instruction="I'm heading out soon. Please take care of a few things before I leave. See whether my cup is under the pool table in the recreation room. See if the laptop is on the lounge table. If it's not there, please help me find it. Verify that the toolbox is on the top shelf in the storage room. Determine whether the coron in the tea room microwave has finished heating."
pic(r/'hardware_assets/user_fig1.png',650,122,150,180,False)
text(694,318,'User',26,'#333333')
# Preserve the user's instruction verbatim; wrap only for typography.
text(850,29,"I'm heading out soon. Please take care of a few things before I leave.",56,'#171717')
items=[
 'See whether my cup is under the pool table in the recreation room.',
 "See if the laptop is on the lounge table. If it's not there, please help me find it.",
 'Verify that the toolbox is on the top shelf in the storage room.',
 'Determine whether the coron in the tea room microwave has finished heating.'
]
available=3170-891
bullet_size=next(size for size in range(70,38,-1) if max(d.textlength(v,font=ImageFont.truetype(font,size)) for v in items)<=available)
for j,item in enumerate(items):
 yy=117+j*73
 d.ellipse((860,yy+26,872,yy+38),fill='#171717')
 svg.append(f'<circle cx="866" cy="{yy+32}" r="6" fill="#171717"/>')
 import re
 xx=891
 for token in re.split(r'(\bcup\b|\blaptop\b|\btoolbox\b|\bcoron\b)',item):
  if not token:continue
  is_object=token in {'cup','laptop','toolbox','coron'}
  face=font.replace('-Regular.ttf','-Bold.ttf') if is_object else font
  ff=ImageFont.truetype(face,bullet_size)
  d.text((xx,yy),token,font=ff,fill='#171717')
  svg.append(f'<text x="{xx}" y="{yy+bullet_size}" font-family="Liberation Serif,Times New Roman,serif" font-size="{bullet_size}" font-weight="{"bold" if is_object else "normal"}" fill="#171717">{html.escape(token)}</text>')
  xx+=d.textlength(token,font=ff)
stage_labels=[['Laptop','search & recovery'],['Toolbox','verification'],['Microwave','inspection'],['Cup','verification']]
for i,(n,ran,lines,times) in enumerate(rows):
 y=450+i*375
 pic(r/'stage_corrected_views'/f'{n}_selected.png',10,y,570,305)
 caption='Subgoal '+chr(65+i)+': '+stage_labels[i][0]
 width=d.textlength(caption,font=ImageFont.truetype(font.replace('-Regular.ttf','-Bold.ttf'),32))
 text(295-width/2,y+317,caption,32,'#101010')
 a={'01_laptop':23,'02_toolbox':49,'03_microwave':60,'04_cup':87}[n]
 for k,t in enumerate(times):
  x=610+k*515
  source=r/'map_supported_results'/n
  idx=(t-a)*(4 if n=='02_toolbox' else 2)
  if k==0 and n!='01_laptop':
   prev,idx={'02_toolbox':('01_laptop',46),'03_microwave':('02_toolbox',20),'04_cup':('03_microwave',48)}[n]
   source=r/'map_supported_results'/prev
  pic(source/f'frame_{idx:03d}.jpg',x,y,507,330,True)
  if i==0 and k>=3:text(x+110,y+339,'Semantic recovery',25,'#00594e')
text(25,1953,'Pink: recorded EKF camera path   |   Blue: observation frusta (uniform display size)   |   Green: start   |   Yellow: end   |   Mapping RGB-D + LiDAR-supported flight VGGT depth; EKF path; approximate alignment',20,'#333333')
svg.append('</svg>');im.save(o/'main_figure.png',dpi=(300,300));im.save(o/'main_figure.pdf',resolution=300);(o/'main_figure.svg').write_text('\n'.join(svg));im.resize((1600,990),Image.Resampling.LANCZOS).save(o/'preview.jpg',quality=95)
