from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import json,math
out=Path(__file__).resolve().parent.parent
dest=Path(__file__).resolve().parent
S=2;W,H=2200,1320
im=Image.new('RGB',(W*S,H*S),'white');crop=(480,220,1520,1233);scale=1.38;origin=(5,20)
main=Image.open(dest/'global_large_icons.png').convert('RGB').crop(crop);main=main.resize((round(main.width*scale*S),round(main.height*scale*S)),Image.Resampling.LANCZOS);im.paste(main,(origin[0]*S,origin[1]*S));d=ImageDraw.Draw(im)
def dash(a,b):
 length=math.dist(a,b)
 for t in range(0,math.ceil(length),18):
  e=min(length,t+10);d.line([(round((a[0]+(b[0]-a[0])*v/length)*S),round((a[1]+(b[1]-a[1])*v/length)*S)) for v in (t,e)],fill='#202020',width=4*S)
def marked(local,key):
 local=local.copy();draw=ImageDraw.Draw(local);x,y=json.loads((out/'cross_positions.json').read_text())[key]
 radius=84 if key=='collision' else 70
 for sign in (-1,1):draw.line([(x-radius,y-sign*radius),(x+radius,y+sign*radius)],fill='#D52F3B',width=22 if key=='collision' else 20)
 return local
def place(local,box):
 bb=local.getchannel('A').point(lambda a:255 if a>15 else 0).getbbox()
 if bb:local=local.crop(bb)
 f=min((box[2]-box[0]-24)*S/local.width,(box[3]-box[1]-24)*S/local.height);local=local.resize((round(local.width*f),round(local.height*f)),Image.Resampling.LANCZOS)
 im.paste(local,(round((box[0]+box[2])*S/2-local.width/2),round((box[1]+box[3])*S/2-local.height/2)),local)
def rounded_dash(box,radius):
 x0,y0,x1,y1=box;pts=[]
 for cx,cy,start in [(x1-radius,y0+radius,-90),(x1-radius,y1-radius,0),(x0+radius,y1-radius,90),(x0+radius,y0+radius,180)]:
  for j in range(25):
   a=math.radians(start+90*j/24);pts.append((cx+radius*math.cos(a),cy+radius*math.sin(a)))
 pts.append(pts[0]);distance=0
 for a,b in zip(pts,pts[1:]):
  length=math.dist(a,b);n=max(1,math.ceil(length))
  for j in range(n):
   t0=j/n;t1=(j+1)/n
   if (distance+length*(t0+t1)/2)%18<10:
    d.line([((a[0]+(b[0]-a[0])*t)*S,(a[1]+(b[1]-a[1])*t)*S) for t in (t0,t1)],fill='#202020',width=4*S)
  distance+=length
def callout(proj,box):
 x=origin[0]+(proj[0]-crop[0])*scale;y=origin[1]+(proj[1]-crop[1])*scale;r=42
 rounded_dash((x-r,y-r,x+r,y+r),12)
 dash((x+r,y-r+12),(box[0],box[1]+35));dash((x+r,y+r-12),(box[0],box[3]-35))
 rounded_dash(box,24)
local=Image.open(out/'occlusion_inset_3d.png').convert('RGBA')
box1=(1490,160,2180,680);box2=(1490,745,2180,1265)
place(marked(local,'occlusion'),box1);place(marked(Image.open(out/'collision_explanation.png').convert('RGBA'),'collision'),box2)
callout(json.loads((out/'occlusion_inset_geometry.json').read_text())['projection'],box1);callout(json.loads((out/'collision_callout.json').read_text())['projection'],box2)
im.save(dest/'scene_large_icons.png')
