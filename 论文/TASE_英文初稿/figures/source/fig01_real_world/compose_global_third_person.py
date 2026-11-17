from pathlib import Path
from PIL import Image,ImageDraw,ImageFont,ImageOps
import base64,html,io
r=Path(__file__).resolve().parent;o=r/'logicfly_global_third_person_figure';o.mkdir(exist_ok=True)
W,H=3200,3200;im=Image.new('RGB',(W,H),'white');d=ImageDraw.Draw(im)
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

# Whole-scene map with linked first-person observations.
import numpy as np,trimesh
text(30,460,'Continuous execution & first-person observations',44,'#14294b')
a=Image.open(r/'full_trajectory_scene/view_-60.png').crop((350,150,1700,1320))
a=ImageOps.pad(a,(1870,1330),method=Image.Resampling.LANCZOS,color='white');a.save(o/'global_map_crop.png')
pic(o/'global_map_crop.png',25,530,1870,1330,True)
# Reproduce the scene projection so callouts point to measured camera positions.
p=np.asarray(trimesh.load(r/'full_trajectory_scene/fused_scene.ply').vertices)
z=np.load(r/'odom_corrected_results/full_trajectory/poses.npz');cent=z['centers'];ts=z['times']
az,el=np.deg2rad([-60,55]);right=np.array([np.cos(az),0,np.sin(az)]);up=np.array([-np.sin(az)*np.sin(el),np.cos(el),np.cos(az)*np.sin(el)]);B=np.stack([right,-up],1);xy=p@B;path=cent@B;lo,hi=np.quantile(xy,[.002,.998],axis=0);lo=np.minimum(lo,path.min(0));hi=np.maximum(hi,path.max(0));scale=min(1800/(hi[0]-lo[0]),1080/(hi[1]-lo[1]));proj=(path-(lo+hi)/2)*scale+[1000,740]
# ImageOps.fit center-crops to destination aspect.
scl=min(1870/1350,1330/1170);cropx=(1350*scl-1870)/2;cropy=(1170*scl-1330)/2
cards=[(46,'A · Laptop recovered','01_laptop',46),(54,'B · Toolbox','02_toolbox',20),(84,'C · Microwave','03_microwave',48),(102,'D · Cup','04_cup',30)]
for j,(t,title,n,idx) in enumerate(cards):
 yy=550+j*320;x,y=proj[np.argmin(abs(ts-t))];x=25+(x-350)*scl-cropx;y=530+(y-150)*scl-cropy
 pts=[(x,y),(1945+j*45,y),(2120,yy+145),(2230,yy+145)]
 d.line(pts,fill='white',width=9,joint='curve');d.line(pts,fill='#315a8b',width=4,joint='curve');d.ellipse((x-9,y-9,x+9,y+9),fill='#ffb400',outline='white',width=3)
 svg.append('<polyline points="'+' '.join(f'{u},{v}' for u,v in pts)+'" fill="none" stroke="#315a8b" stroke-width="4"/>')
 text(2230,yy,title,35,'#14294b');fp=Image.open(r/'map_supported_results'/n/f'frame_{idx:03d}.jpg');fp=ImageOps.pad(fp,(920,260),method=Image.Resampling.LANCZOS,color='#f4f6f8');fp.save(o/f'observation_{j}.png');pic(o/f'observation_{j}.png',2230,yy+47,920,260,True)
 d.rectangle((2230,yy+47,3150,yy+307),outline='#315a8b',width=3)
 svg.append(f'<rect x="2230" y="{yy+47}" width="920" height="260" fill="none" stroke="#315a8b" stroke-width="3"/>')
text(30,1880,'Third-person execution sequence',44,'#14294b')
for j,(name,_,_,_) in enumerate(rows):
 yy=1950+j*295
 label=['A · Laptop search & recovery','B · Toolbox verification','C · Microwave inspection','D · Cup verification'][j]
 text(25,yy,label,32,'#14294b')
 third_times={'01_laptop':[19,31,37,47,53],'02_toolbox':[53,59,63,66,69],'03_microwave':[69,87,103,127,129],'04_cup':[129,139,149,159,163]}[name]
 for k,t in enumerate(third_times):pic(r/'third_person_assets'/f'rgb_{t:03d}.jpg',25+k*635,yy+43,625,235,True)
text(25,3152,'VGGT + mapping RGB-D + LiDAR support | Recorded EKF trajectory with approximate video alignment | Blue frusta: observation poses',24,'#333333')
svg.append('</svg>');im.save(o/'main_figure.png',dpi=(300,300));im.save(o/'main_figure.pdf',resolution=300);(o/'main_figure.svg').write_text('\n'.join(svg));im.resize((1600,1600),Image.Resampling.LANCZOS).save(o/'preview.jpg',quality=95)
