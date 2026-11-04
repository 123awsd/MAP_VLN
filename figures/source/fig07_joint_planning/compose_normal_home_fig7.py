from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
root=Path('/home/uav/map_VLN'); src=root/'PRE_MAP_VLN/figures/source/fig07_joint_planning/fig7_normal_home_scene.png'; out=root/'PRE_MAP_VLN/figures/source/fig07_joint_planning/fig7_normal_home_scene_annotated.png'
im=Image.open(src).convert('RGBA'); d=ImageDraw.Draw(im,'RGBA'); fp='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'; fb=ImageFont.truetype(fp,27); fs=ImageFont.truetype(fp,20); fsm=ImageFont.truetype(fp,17)
d.rounded_rectangle((25,22,555,76),radius=11,fill=(255,255,255,225)); d.text((45,36),'Task-conditioned viewpoint planning',font=fb,fill=(39,51,59,255))
d.rounded_rectangle((600,28,1175,82),radius=11,fill=(255,255,255,225),outline=(229,138,74,255),width=3); d.text((620,43),'A: not found  →  replan from A',font=fs,fill=(205,107,48,255))
for xy,label,col in [((967,420),'A',(120,132,140,255)),((480,120),'B',(38,128,146,255)),((210,360),'C',(38,128,146,255)),((695,700),'D',(229,138,74,255))]: d.text(xy,label,font=fb,fill=col,stroke_width=3,stroke_fill=(255,255,255,230))
d.text((1117,402),'Start',font=fsm,fill=(39,51,59,255),stroke_width=3,stroke_fill=(255,255,255,220))
y=860; d.line((35,y,98,y),fill=(120,132,140,230),width=4); d.text((113,y-13),'candidate views',font=fsm,fill=(100,112,120,255)); d.line((285,y,350,y),fill=(38,128,146,245),width=7); d.text((365,y-13),'selected / trajectory',font=fsm,fill=(38,128,146,255))
im.convert('RGB').save(out,quality=95); print(out)
