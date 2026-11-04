"""Compose legend and condition text around the 3-D Blender rendering."""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
H=Path(__file__).resolve().parent
im=Image.open(H/'start_room_astar_render.png').convert('RGB')
out=Image.new('RGB',(1600,1465),'white'); out.paste(im,(0,0)); d=ImageDraw.Draw(out)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',27); small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',25); bold=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',27)
d.line((45,1293,1555,1293),fill='#d8dee2',width=2)
def item_line(x,y,color,label,dash=False):
 if dash:
  for i in range(0,80,24): d.line((x+i,y,x+min(i+13,80),y),fill=color,width=6)
 else: d.line((x,y,x+80,y),fill=color,width=6)
 d.text((x+96,y-18),label,font=font,fill='#30383e')
item_line(60,1334,'#858a8d','Executed')
item_line(412,1334,'#4b5155','Initial plan',True)
item_line(825,1334,'#00aacc','Updated plan')
d.polygon([(1270,1348),(1294,1316),(1318,1348)],outline='#e88a1b',width=4); d.text((1330,1315),'Active',font=font,fill='#30383e')
d.text((60,1390),'Initial: A → C → B',font=bold,fill='#495258')
d.text((500,1390),'A: not found  →  D active',font=bold,fill='#b96915')
d.text((1080,1390),'Updated: D → B → C',font=bold,fill='#087b93')
out.save(H/'fig7_start_room_astar_draft.png',dpi=(300,300))
out.save(H/'fig7_start_room_astar_draft.pdf','PDF',resolution=300.0)
