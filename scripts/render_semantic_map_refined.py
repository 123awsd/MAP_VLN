"""Restrained editorial presentation of existing measured room and Boxer results."""
from pathlib import Path
import json
import numpy as np
from PIL import Image,ImageDraw,ImageFont,ImageFilter
import imageio_ffmpeg
R=Path(__file__).resolve().parents[1];O=R/'outputs/videos/semantic_map_refined_14s';O.mkdir(parents=True,exist_ok=True)
G=R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz';D=R/'outputs/coverage700_wall_room_preview/manual_room305_boundary';B=R/'outputs/boxer/coverage700_sam31_20261018/fusion_fixed/boxer_summary.json'
a=np.load(G);objects=json.loads(B.read_text())['objects'];W,H=1600,900
BG=(245,244,240);INK=(40,53,57);TEAL=(29,119,119);palette=[(173,201,194),(189,203,217),(219,199,173),(193,190,211),(204,211,184),(219,191,181)]
font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
F={s:ImageFont.truetype(font,s)for s in [15,18,20,23,28,42]}
def ease(v):v=np.clip(v,0,1);return float(v*v*(3-2*v))
def project(p,f):
 p=np.asarray(p);x=-(p[:,0]-1)*.94+(p[:,1]-8)*.342;y=(p[:,0]-1)*.342+(p[:,1]-8)*.94
 return np.c_[650+x*43,510+y*31-(p[:,2]-[0,2.75,5.75][f])*23]
def raster(im,xy,rgba):
 q=np.rint(xy).astype(int);valid=(q[:,0]>=0)&(q[:,0]<1300)&(q[:,1]>=0)&(q[:,1]<1020);ar=np.array(im);ar[q[valid,1],q[valid,0]]=rgba;return Image.fromarray(ar)
def text_chip(d,xy,text,size=18):
 x,y=xy;b=d.textbbox((0,0),text,font=F[size]);tw=b[2]-b[0];d.rounded_rectangle((x-tw/2-7,y-13,x+tw/2+7,y+15),radius=5,fill=(250,249,246,235));d.text((x,y),text,font=F[size],fill=INK,anchor='mm')
raws=[];structs=[];sems=[]
for f in range(3):
 b=np.load(D/f'L{f+1}_constrained.npz');l=b['labels'];r=float(b['resolution']);org=b['origin_xy'];z=[0,2.75,5.75][f]
 p=a[f'p{f}'][::2];q=project(p,f)
 raw=Image.new('RGBA',(1300,1020));raw=raster(raw,q,(115,129,131,165));raws.append(raw)
 st=Image.new('RGBA',(1300,1020));d=ImageDraw.Draw(st)
 anchors=[]
 for k in np.unique(l[l>0]):
  mask=l==k;ys,xs=np.nonzero(mask);j=np.argmin((xs-np.median(xs))**2+(ys-np.median(ys))**2);anchors.append((project([[org[0]+(xs[j]+.5)*r,org[1]+(ys[j]+.5)*r,z]],f)[0],f'{f+1}{int(k):02d}'))
  for y in range(l.shape[0]):
   row=l[y]==k;dv=np.diff(np.r_[False,row,False].astype(int))
   for x,e in zip(np.where(dv==1)[0],np.where(dv==-1)[0]):
    pp=[[org[0]+x*r,org[1]+y*r,z],[org[0]+e*r,org[1]+y*r,z],[org[0]+e*r,org[1]+(y+1)*r,z],[org[0]+x*r,org[1]+(y+1)*r,z]]
    d.polygon([tuple(v)for v in project(pp,f)],fill=palette[(int(k)-1)%len(palette)]+(235,))
 # Keep faint measured structure, with extracted wall evidence above the fills.
 context=raster(Image.new('RGBA',st.size),q[::4],(93,107,111,48));st=Image.alpha_composite(st,context)
 yy,xx=np.nonzero(b['measured_wall']);wp=np.c_[org[0]+(xx+.5)*r,org[1]+(yy+.5)*r,np.full(len(xx),z+.12)];st=raster(st,project(wp,f),(66,79,80,230))
 d=ImageDraw.Draw(st)
 for xy,label in anchors:text_chip(d,xy,label)
 structs.append(st)
 sem=st.copy();d=ImageDraw.Draw(sem)
 edges=[(i,j)for i in range(8)for j in range(i+1,8)if bin(i^j).count('1')==1]
 occupied=[]
 for o in objects:
  if o['floor']!=f+1:continue
  pp=project(o['corners_xyz'],f)
  for i,j in edges:d.line([tuple(pp[i]),tuple(pp[j])],fill=TEAL+(240,),width=3)
  # Sparse labels: avoid competing with room IDs and each other.
  if o['label'] in ['bed','sofa','cabinet','chair','table','sink']:
   xy=pp.mean(axis=0)+[0,-22]
   if all(np.linalg.norm(xy-old)>100 for old in occupied) and len(occupied)<5:
    occupied.append(xy);text_chip(d,xy,o['label'],20)
 sems.append(sem)
def blend_layers(f,u,v):
 return Image.blend(Image.blend(raws[f],structs[f],u),sems[f],v)
def place(im,layer,cx,cy,scale,opacity=1):
 size=(int(layer.width*scale),int(layer.height*scale));layer=layer.resize(size,Image.Resampling.LANCZOS)
 if opacity<1:layer.putalpha(layer.getchannel('A').point(lambda x:int(x*opacity)))
 im.alpha_composite(layer,(round(cx-size[0]/2),round(cy-size[1]/2)))
def frame(t):
 im=Image.new('RGBA',(W,H),BG+(255,));u=ease((t-1.5)/3);v=ease((t-5)/2.8);spread=ease((t-9)/3.5)
 # One coherent hero view becomes a three-floor composition with no camera reset.
 scale=(.82-.025*min(t,9)/9)*(1-spread)+.55*spread
 hero=blend_layers(1,u,v);place(im,hero,800,486,scale)
 if spread>0:
  place(im,sems[0],800-520*spread,486,.55,spread)
  place(im,sems[2],800+520*spread,486,.55,spread)
 d=ImageDraw.Draw(im)
 title='从实测空间，到语义地图'
 d.text((75,54),'MEASURED → STRUCTURED → SEMANTIC',font=F[15],fill=(108,125,124))
 d.text((72,84),title,font=F[42],fill=INK)
 d.text((75,151),'700秒探索数据  /  房间结构  /  SAM 3.1 + Boxer',font=F[18],fill=(119,128,127))
 # A restrained phase label; no time counter or progress bar.
 stage='实测点云' if t<2 else ('房间结构' if t<5 else ('物体语义' if t<9 else '三层联合展示'))
 d.rounded_rectangle((1285,75,1525,120),radius=22,fill=(229,234,229));d.text((1405,97),stage,font=F[20],fill=TEAL,anchor='mm')
 if spread>.02:
  alpha=spread
  c=tuple(round(BG[k]*(1-alpha)+INK[k]*alpha)for k in range(3))
  for x,label in [(280,'01  一楼'),(800,'02  二楼'),(1320,'03  三楼')]:d.text((x,745),label,font=F[23],fill=c,anchor='mm')
 else:d.text((800,797),'02  /  二楼 · 实测房间与物体',font=F[20],fill=(98,115,113),anchor='mm')
 d.text((75,854),'实测结果离线展示 · 房间沿用当前局部人工修正版',font=F[15],fill=(146,151,147))
 d.text((1525,854),'69 OBJECTS / 3 FLOORS',font=F[15],fill=(113,133,128),anchor='ra')
 return im.convert('RGB')
for t in [0,4.5,8,13.8]:frame(t).save(O/f'preview_{t}.jpg')
writer=imageio_ffmpeg.write_frames(str(O/'semantic_map_refined_14s.mp4'),(W,H),fps=30,macro_block_size=2,codec='libx264',quality=8,output_params=['-movflags','+faststart']);writer.send(None)
for i in range(420):
 writer.send(np.asarray(frame(i/30)))
 if i%90==0:print(i,flush=True)
writer.close()
(O/'manifest.json').write_text(json.dumps({'cloud':str(G),'rooms':str(D),'objects':str(B),'duration':14,'fps':30,'resolution':[W,H],'presentation':'aligned fades, neutral cloud, muted room palette, uniform teal OBBs; final panels are separate floors','manual_room_revision':True},indent=2))
print(O)
