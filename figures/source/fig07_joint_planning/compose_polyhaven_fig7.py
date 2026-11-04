from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
root=Path('/home/uav/map_VLN'); src=root/'PRE_MAP_VLN/figures/source/fig07_joint_planning/fig7_polyhaven_room_scene.png'; out=root/'PRE_MAP_VLN/figures/source/fig07_joint_planning/fig7_polyhaven_room_scene_annotated.png'
im=Image.open(src).convert('RGBA'); d=ImageDraw.Draw(im,'RGBA')
fp='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'; fb=ImageFont.truetype(fp,28); fs=ImageFont.truetype(fp,22); fsmall=ImageFont.truetype(fp,19)
# title panel
d.rounded_rectangle((28,24,520,78),radius=12,fill=(255,255,255,225)); d.text((48,37),'Joint task–viewpoint planning',font=fb,fill=(39,51,59,255))
# callout
d.rounded_rectangle((590,32,1170,88),radius=12,fill=(255,255,255,225),outline=(229,138,74,255),width=3); d.text((615,47),'A: not found  →  activate D',font=fs,fill=(205,107,48,255))
# labels near markers; positions follow this camera view
for xy,label,col in [((963,434),'A',(120,132,140,255)),((475,150),'B',(38,128,146,255)),((214,395),'C',(38,128,146,255)),((702,706),'D',(229,138,74,255))]:
    d.text(xy,label,font=fb,fill=col,stroke_width=3,stroke_fill=(255,255,255,225))
d.text((1122,438),'Start',font=fsmall,fill=(39,51,59,255),stroke_width=3,stroke_fill=(255,255,255,220))
# legend
y=862
d.line((40,y,105,y),fill=(120,132,140,220),width=4); d.text((120,y-14),'A* guide',font=fsmall,fill=(100,112,120,255)); d.line((280,y,350,y),fill=(38,128,146,245),width=7); d.text((365,y-14),'MINCO trajectory',font=fsmall,fill=(38,128,146,255))
im.convert('RGB').save(out,quality=95); print(out)
