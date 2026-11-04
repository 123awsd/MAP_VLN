from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
root=Path('/home/uav/map_VLN'); src=root/'PRE_MAP_VLN/figures/source/fig07_joint_planning/fig7_home_p3_scene.png'; out=root/'PRE_MAP_VLN/figures/source/fig07_joint_planning/fig7_home_p3_scene_annotated.png'
im=Image.open(src).convert('RGBA'); d=ImageDraw.Draw(im,'RGBA'); fp='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'; fb=ImageFont.truetype(fp,28); fs=ImageFont.truetype(fp,20); fsm=ImageFont.truetype(fp,17)
d.rounded_rectangle((25,22,540,76),radius=11,fill=(255,255,255,225)); d.text((45,36),'Joint task–viewpoint planning',font=fb,fill=(39,51,59,255))
d.rounded_rectangle((620,28,1175,82),radius=11,fill=(255,255,255,225),outline=(229,138,74,255),width=3); d.text((640,43),'A: not found  →  replan',font=fs,fill=(205,107,48,255))
for xy,label,col in [((1010,205),'A',(25,119,139,255)),((440,205),'B',(25,119,139,255)),((1010,700),'C',(25,119,139,255)),((75,700),'D',(229,107,48,255))]: d.text(xy,label,font=fb,fill=col,stroke_width=3,stroke_fill=(255,255,255,220))
d.text((1120,55),'Start',font=fsm,fill=(39,51,59,255),stroke_width=3,stroke_fill=(255,255,255,220))
y=860; d.line((35,y,95,y),fill=(75,86,92,230),width=4); d.text((110,y-13),'candidate views',font=fsm,fill=(100,112,120,255)); d.line((285,y,350,y),fill=(25,119,139,245),width=7); d.text((365,y-13),'selected trajectory',font=fsm,fill=(25,119,139,255))
im.convert('RGB').save(out,quality=95); print(out)
