from pathlib import Path
from PIL import Image,ImageDraw,ImageFont,ImageOps
import base64,html,io
r=Path(__file__).resolve().parent;o=r/'logicfly_no_fpv_tilted_figure';o.mkdir(exist_ok=True)
W,H=3200,2220;im=Image.new('RGB',(W,H),'white');d=ImageDraw.Draw(im)
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
# Black dashed instruction frame with generous gaps and inner padding.
x0,y0,x1,y1=620,7,3190,423
# Draw continuous rounded perimeter dashes, including at the four corners.
import math
radius=24;dash,gap=34,28;stroke=7
perimeter=[]
def straight(a,b):
 steps=max(1,int(math.dist(a,b)))
 for t in range(steps):
  u=t/steps;perimeter.append((a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1])))
def arc(cx,cy,a,b):
 steps=max(1,int(radius*(b-a)))
 for t in range(steps):
  angle=a+(b-a)*t/steps;perimeter.append((cx+radius*math.cos(angle),cy+radius*math.sin(angle)))
straight((x0+radius,y0),(x1-radius,y0));arc(x1-radius,y0+radius,-math.pi/2,0)
straight((x1,y0+radius),(x1,y1-radius));arc(x1-radius,y1-radius,0,math.pi/2)
straight((x1-radius,y1),(x0+radius,y1));arc(x0+radius,y1-radius,math.pi/2,math.pi)
straight((x0,y1-radius),(x0,y0+radius));arc(x0+radius,y0+radius,math.pi,math.pi*1.5)
for start in range(0,len(perimeter),dash+gap):d.line(perimeter[start:start+dash+1],fill='#000000',width=stroke,joint='curve')
svg.append(f'<rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" rx="{radius}" ry="{radius}" fill="none" stroke="#000000" stroke-width="{stroke}" stroke-dasharray="{dash} {gap}"/>')
instruction="I'm heading out soon. Please take care of a few things before I leave. See whether my cup is under the pool table in the recreation room. See if the laptop is on the lounge table. If it's not there, please help me find it. Verify that the toolbox is on the top shelf in the storage room. Determine whether the coron in the tea room microwave has finished heating."


# Preserve the user's instruction verbatim; wrap only for typography.
text(660,29,"I'm heading out soon. Please take care of a few things before I leave.",56,'#171717')
items=[
 'See whether my cup is under the pool table in the recreation room.',
 "See if the laptop is on the lounge table. If it's not there, please help me find it.",
 'Verify that the toolbox is on the top shelf in the storage room.',
 'Determine whether the coron in the tea room microwave has finished heating.'
]
available=3170-701
bullet_size=next(size for size in range(70,38,-1) if max(d.textlength(v,font=ImageFont.truetype(font,size)) for v in items)<=available)
for j,item in enumerate(items):
 yy=117+j*73
 d.ellipse((670,yy+26,682,yy+38),fill='#171717')
 svg.append(f'<circle cx="676" cy="{yy+32}" r="6" fill="#171717"/>')
 import re
 xx=701
 for token in re.split(r'(\bcup\b|\blaptop\b|\btoolbox\b|\bcoron\b)',item):
  if not token:continue
  is_object=token in {'cup','laptop','toolbox','coron'}
  face=font.replace('-Regular.ttf','-Bold.ttf') if is_object else font
  ff=ImageFont.truetype(face,bullet_size)
  d.text((xx,yy),token,font=ff,fill='#171717')
  svg.append(f'<text x="{xx}" y="{yy+bullet_size}" font-family="Liberation Serif,Times New Roman,serif" font-size="{bullet_size}" font-weight="{"bold" if is_object else "normal"}" fill="#171717">{html.escape(token)}</text>')
  xx+=d.textlength(token,font=ff)

# Compact horizontal whole-scene projection, preserving the existing four rows.
import numpy as np,trimesh
pcl=trimesh.load(r/'full_trajectory_scene/fused_scene.ply');p=np.asarray(pcl.vertices);colors=np.asarray(pcl.colors)[:,:3]
z=np.load(r/'odom_corrected_results/full_trajectory/poses.npz');cent=z['centers'];rots=z['rotations'];ts=z['times']
az,el=np.deg2rad([-5,15]);right=np.array([np.cos(az),0,np.sin(az)]);up=np.array([-np.sin(az)*np.sin(el),np.cos(el),np.cos(az)*np.sin(el)]);roll=np.deg2rad(2);B=np.stack([right,-up],1)@np.array([[np.cos(roll),-np.sin(roll)],[np.sin(roll),np.cos(roll)]])
xy=p@B;path=cent@B;lo,hi=np.stack([xy.min(0),xy.max(0)]);lo=np.minimum(lo,path.min(0));hi=np.maximum(hi,path.max(0));scale=3120/(hi[0]-lo[0]);org=(lo+hi)/2
project=lambda v:(v@B-org)*scale+[1600,650]
uv=project(p);pixels=np.round(uv).astype(int);valid=(pixels[:,0]>10)&(pixels[:,0]<3190)&(pixels[:,1]>445)&(pixels[:,1]<865);pixels=pixels[valid];cc=colors[valid];order=np.argsort(p[valid]@np.cross(right,up));layer=np.full((H,W,3),255,dtype='uint8')
for dx,dy in [(a,b) for a in range(-2,3) for b in range(-2,3) if a*a+b*b<=5]:layer[pixels[order,1]+dy,pixels[order,0]+dx]=cc[order]
asset=Image.fromarray(layer).crop((0,445,3200,867));asset.save(o/'horizontal_map.png');pic(o/'horizontal_map.png',0,445,3200,422,True)
path=project(cent)
def line(points,color,width):
 d.line([tuple(q) for q in points],fill=color,width=width,joint='curve');svg.append('<polyline points="'+' '.join(f'{u:.2f},{v:.2f}' for u,v in points)+f'" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"/>')
line(path,'white',15);line(path,'#e40046',8)
# Observation cones stay in the top layer of the 3D rendering.
for t in [35,46,54,84,102]:
 i=np.argmin(abs(ts-t));corn=np.array([[-.505,-.405,1],[.688,-.405,1],[.688,.388,1],[-.505,.388,1]])*1.4;wc=corn@rots[i].T;corn=wc[:,[0,2,1]]*[1,1,-1]+cent[i];anchor=path[i];pr=project(np.vstack([cent[i],corn]));fac=75/np.ptp(pr,axis=0).max()
 for edge in [np.array([cent[i],c]) for c in corn]+[np.array([corn[k],corn[(k+1)%4]]) for k in range(4)]:
  seg=anchor+(project(edge)-anchor)*fac;line(seg,'white',7);line(seg,'#005bff',3)
 x,y=anchor;d.ellipse((x-7,y-7,x+7,y+7),fill='#ffb400',outline='white',width=2)
x,y=path[0];d.ellipse((x-10,y-10,x+10,y+10),fill='#00ad8e',outline='white',width=3)


def rounded_pic(p,x,y,w,h):
 a=ImageOps.fit(Image.open(p).convert('RGB'),(w,h),Image.Resampling.LANCZOS)
 mask=Image.new('L',(w,h),0);ImageDraw.Draw(mask).rounded_rectangle((0,0,w-1,h-1),radius=20,fill=255)
 a=a.convert('RGBA');a.putalpha(mask);im.paste(a,(x,y),a)
 d.rounded_rectangle((x,y,x+w-1,y+h-1),radius=20,outline='#9ba8b4',width=2)
 b=io.BytesIO();a.save(b,format='PNG');enc=base64.b64encode(b.getvalue()).decode();svg.append(f'<image x="{x}" y="{y}" width="{w}" height="{h}" xlink:href="data:image/png;base64,{enc}"/>');svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="20" fill="none" stroke="#9ba8b4" stroke-width="2"/>')

stage_labels=[['Laptop','search & recovery'],['Toolbox','verification'],['Microwave','inspection'],['Cup','verification']]
for i,(n,ran,lines,times) in enumerate(rows):
 y=865+i*330
 pic(r/'stage_corrected_views'/f'{n}_selected.png',15,y,430,260)
 caption='Subgoal '+chr(65+i)+': '+stage_labels[i][0]
 width=d.textlength(caption,font=ImageFont.truetype(font.replace('-Regular.ttf','-Bold.ttf'),29))
 text(230-width/2,y+272,caption,29,'#101010')
 third_times={'01_laptop':[19,31,37,47,53],'02_toolbox':[53,59,63,66,69],'03_microwave':[69,87,103,127,129],'04_cup':[129,139,149,159,163]}[n]
 for k,t in enumerate(third_times):
  x=475+k*542
  pic(r/'third_person_assets'/f'rgb_{t:03d}.jpg',x,y,532,299,True)
  if i==0 and k>=3:text(x+120,y+302,'Semantic recovery',22,'#00594e')
text(25,2193,'Pink: recorded EKF camera path   |   Blue: observation frusta (uniform display size)   |   Green: start   |   Yellow: end   |   Mapping RGB-D + LiDAR-supported flight VGGT depth; EKF path; approximate alignment',20,'#333333')
svg.append('</svg>');im.save(o/'main_figure.png',dpi=(300,300));im.save(o/'main_figure.pdf',resolution=300);(o/'main_figure.svg').write_text('\n'.join(svg));im.resize((1600,1110),Image.Resampling.LANCZOS).save(o/'preview.jpg',quality=95)
