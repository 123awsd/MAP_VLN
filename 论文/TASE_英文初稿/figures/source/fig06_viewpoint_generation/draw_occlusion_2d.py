from PIL import Image,ImageDraw
from pathlib import Path
import json,math
out=Path(__file__).resolve().parent
g=json.loads((out/'occlusion_inset_geometry.json').read_text());p=g['position'];yaw=math.atan2(-p[1],-p[0]);forward=(math.cos(yaw),math.sin(yaw));side=(-forward[1],forward[0])
im=Image.new('RGBA',(1400,1000),(255,255,255,0));d=ImageDraw.Draw(im)
def project(q):
 dx=q[0]-p[0];dy=q[1]-p[1]
 return (240+(dx*forward[0]+dy*forward[1])*250,500-(dx*side[0]+dy*side[1])*250)
# Top-down tabletop, aligned in the same scene coordinate system.
table=Image.new('RGBA',im.size);td=ImageDraw.Draw(table);td.polygon([project(q) for q in [(-1,-.5),(1,-.5),(1,.5),(-1,.5)]],fill='#E5D6C4',outline='#A18F7A',width=4);im.alpha_composite(table)
# Visibility silhouette is ray-clipped against the real chair mesh and exported from Blender.
ends=json.loads((out/'fov_section_points.json').read_text())
beam=Image.new('RGBA',im.size);bd=ImageDraw.Draw(beam);bd.polygon([project(p)]+[project(q) for q in ends],fill=(34,150,175,80));im.alpha_composite(beam);d=ImageDraw.Draw(im)
# Draw chair footprints at camera height, including their actual spacing.
for polygon in sorted(g['chair_backs'],key=lambda poly:sum((sum(v[k] for v in poly)/len(poly)-p[k])**2 for k in (0,1)))[:1]:
 pts=[project(q) for q in polygon];d.polygon(pts,fill='#B58A60',outline='#765737',width=4)
source=project(p);length=2.35;half=math.tan(math.radians(69/2))
for sign in [-1,1]:
 q=[p[0]+length*(forward[0]+side[0]*half*sign),p[1]+length*(forward[1]+side[1]*half*sign)];d.line([source,project(q)],fill='#167D91',width=4)
d.ellipse((source[0]-9,source[1]-9,source[0]+9,source[1]+9),fill='#167D91')
# Compact 2D camera FoV near its origin.
near=[]
for sign in [-1,1]:near.append(project([p[0]+.24*(forward[0]+side[0]*half*sign),p[1]+.24*(forward[1]+side[1]*half*sign)]))
d.line([near[0],near[1]],fill='#167D91',width=4)
im.save(out/'occlusion_inset_3d.png')
