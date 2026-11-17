from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import json,math
out=Path(__file__).resolve().parent
data=json.loads((out/'polished_projection.json').read_text())
S=2;im=Image.new('RGBA',(4000,2800),'white');d=ImageDraw.Draw(im)
ink='#243443';gray='#667888';teal='#007D91';red='#D5454D';purple='#D5454D'
def line(pts,fill,w=2):d.line([(round(x*S),round(y*S)) for x,y in pts],fill=fill,width=round(w*S),joint='curve')
def circle(p,r,fill,outline=None,w=1):
 x,y=p;d.ellipse(((x-r)*S,(y-r)*S,(x+r)*S,(y+r)*S),fill=fill,outline=outline,width=int(w*S))
def text(p,t,size=24,color=ink,bold=False):
 font='/usr/share/fonts/truetype/dejavu/DejaVuSans'+('-Bold' if bold else '')+'.ttf'
 d.text((p[0]*S,p[1]*S),t,font=ImageFont.truetype(font,size*S),fill=color)
# Uniform, restrained guide strokes behind the scene.
for ring in data['rings']:
 length=0
 for a,b in zip(ring,ring[1:]):
  seg=math.dist(a,b)
  if int(length/12)%2==0:line([a,b],'#A8B5BE',2.0)
  length+=seg
scene=Image.open(out/'polished_scene.png').convert('RGBA').resize(im.size,Image.Resampling.LANCZOS);im.alpha_composite(scene);d=ImageDraw.Draw(im)
# Flat candidate symbols with white rims, omit symbols competing with retained views.
for p in data['dots']:
 if min(math.dist(p,v['p']) for v in data['views'])<55:continue
 if any(math.dist(p,q)<32 for q in data['collisions']+data['occluded']):continue
 circle(p,10.5,gray,'white',2)
for v in data['views']:
 p=v['p'];cs=v['corners']
 for c in cs:line([p,c],teal,2.7)
 line(cs+[cs[0]],teal,2.7)
 circle(p,10,'white',teal,2.5);circle(p,5,teal)
# Red marks collision; yellow marks any geometric visibility loss.
for x,y in data['collisions']:
 for sign in [-1,1]:
  line([(x-17,y-sign*17),(x+17,y+sign*17)],red,8)
for x,y in data['occluded']:
 d.rectangle(((x-18)*S,(y-18)*S,(x+18)*S,(y+18)*S),outline='#D9AD45',width=5*S)
# No title, captions, guides, envelope, ray examples or legend in this version.
im.convert('RGB').resize((2000,1400),Image.Resampling.LANCZOS).save(out/'candidate_selection_polished.png')
im.convert('RGB').save(out/'candidate_selection_polished.pdf',resolution=300)
