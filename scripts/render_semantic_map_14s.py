"""14s presentation of recorded geometry and existing room/object products; no inference."""
from pathlib import Path
import json, math, colorsys
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg
R=Path(__file__).resolve().parents[1];O=R/'outputs/videos/semantic_map_14s_20261018';O.mkdir(parents=True,exist_ok=True)
G=R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz'
D=R/'outputs/coverage700_wall_room_preview/manual_room305_boundary'
B=R/'outputs/boxer/coverage700_sam31_20261018/fusion_fixed/boxer_summary.json'
a=np.load(G);rooms=[np.load(D/f'L{f}_constrained.npz')for f in (1,2,3)];objects=json.loads(B.read_text())['objects']
W,H=1280,720;fps=30;bg=(247,249,252)
font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';fonts={s:ImageFont.truetype(font,s)for s in (14,17,20,24,32)}
def col(i):return tuple(int(v*255)for v in colorsys.hsv_to_rgb((i*.61803398875)%1,.52,.83))
def smooth(v):v=np.clip(v,0,1);return v*v*(3-2*v)
def proj(p,angle,zoom=1,center=(610,410),floor=0):
 p=np.asarray(p).copy();p[:,2]+=floor*3.2;p-=np.array([1,8,6.3]);c,s=np.cos(angle),np.sin(angle)
 x=-p[:,0]*c+p[:,1]*s;y=p[:,0]*s+p[:,1]*c
 return np.stack([center[0]+x*22*zoom,center[1]+(y*.48-p[:,2]*.88)*22*zoom],1)
cloud=[];walls=[];runs=[];labels=[]
for f in range(3):
 p=a[f'p{f}'];p=p[::max(1,len(p)//100000)];cloud.append(p)
 b=rooms[f];l=b['labels'];res=float(b['resolution']);origin=b['origin_xy'];z=[.05,2.8,5.8][f]
 yy,xx=np.nonzero(b['measured_wall']);walls.append(np.c_[origin[0]+(xx+.5)*res,origin[1]+(yy+.5)*res,np.full(len(xx),z+.2)])
 rr=[];ll=[]
 for k in np.unique(l[l>0]):
  mask=l==k;ys,xs=np.nonzero(mask);idx=np.argmin((xs-np.median(xs))**2+(ys-np.median(ys))**2);y,x=ys[idx],xs[idx];ll.append((np.array([[origin[0]+(x+.5)*res,origin[1]+(y+.5)*res,z]]),f'{f+1}{int(k):02d}'))
  for y in range(0,l.shape[0],2):
   row=l[y]==k;diff=np.diff(np.r_[False,row,False].astype(int));starts=np.where(diff==1)[0];ends=np.where(diff==-1)[0]
   for x,e in zip(starts,ends):
    rr.append((np.array([[origin[0]+x*res,origin[1]+y*res,z],[origin[0]+e*res,origin[1]+y*res,z],[origin[0]+e*res,origin[1]+(y+2)*res,z],[origin[0]+x*res,origin[1]+(y+2)*res,z]]),col(int(k)+f*7)))
 runs.append(rr);labels.append(ll)
edges=[(i,j)for i in range(8)for j in range(i+1,8)if bin(i^j).count('1')==1]
classes=sorted({o['label']for o in objects});cc={k:col(i+2)for i,k in enumerate(classes)}
def points(im,p,colors):
 q=np.rint(p).astype(int);ok=(q[:,0]>=0)&(q[:,0]<W)&(q[:,1]>=0)&(q[:,1]<H)
 ar=np.asarray(im).copy();ar[q[ok,1],q[ok,0]]=colors[ok]if getattr(colors,'ndim',0)>1 else colors
 return Image.fromarray(ar)
def scene(angle,semantic=False,zoom=1,center=(610,410),only=None):
 im=Image.new('RGB',(W,H),bg);d=ImageDraw.Draw(im)
 floors=range(3)if only is None else [only]
 for f in floors:
  if semantic:
   for p,c in runs[f]:d.polygon([tuple(v)for v in proj(p,angle,zoom,center,f)],fill=tuple(int(.55*x+.45*255)for x in c))
  p=cloud[f][::7] if semantic else cloud[f];q=proj(p,angle,zoom,center,f)
  if semantic:colors=np.array([189,195,204],dtype=np.uint8)
  else:
   v=np.clip((p[:,2]-[0,2.75,5.75][f])/2.1,0,1);colors=np.array([col(int(x*10)+f*4)for x in v]);colors=(colors*.62+255*.38).astype('uint8')
  im=points(im,q,colors);d=ImageDraw.Draw(im)
  if semantic:
   q=proj(walls[f],angle,zoom,center,f);im=points(im,q,np.array([65,75,88],dtype=np.uint8));d=ImageDraw.Draw(im)
   for o in objects:
    if o['floor']!=f+1:continue
    q=proj(o['corners_xyz'],angle,zoom,center,f);c=cc[o['label']]
    for i,j in edges:d.line([tuple(q[i]),tuple(q[j])],fill=c,width=2)
   for p,label in labels[f]:
    x,y=proj(p,angle,zoom,center,f)[0];d.text((x,y),label,font=fonts[14],fill=(42,52,67),anchor='mm',stroke_width=2,stroke_fill=bg)
  anchor=proj(np.array([[-8.5,18.5,[0,2.75,5.75][f]]]),angle,zoom,center,f)[0];d.text(tuple(anchor),f'{f+1}F',font=fonts[20],fill=(53,65,84),stroke_width=3,stroke_fill=bg)
 return im
# Cache dense height colors / projections reuse the held camera during the wipe.
angle=.16;base=scene(angle);sem=scene(angle,True)
writer=imageio_ffmpeg.write_frames(str(O/'semantic_map_14s.mp4'),(W,H),fps=fps,codec='libx264',pix_fmt_in='rgb24',pix_fmt_out='yuv420p',quality=8,output_params=['-movflags','+faststart']);writer.send(None)
for i in range(420):
 t=i/fps
 if t<3:
  im=base.copy(); title='实测稠密点云'; sub='700秒探索记录 · 三层空间总览'
 elif t<7:
  x=int(smooth((t-3)/4)*W);im=base.copy();im.paste(sem.crop((0,0,x,H)),(0,0));d=ImageDraw.Draw(im)
  if 2<x<W-2:d.line((x,115,x,640),fill=(31,140,202),width=2)
  title='从几何地图到结构与语义地图';sub='房间分区 / 编号 / SAM 3.1 + Boxer 三维物体框'
 elif t<10.5:
  angle=.16+.18*smooth((t-7)/3.5);im=scene(angle,True);title='多楼层结构与物体地图';sub='三层分开展示 · 69个融合物体框'
 else:
  im=Image.new('RGB',(W,H),bg);im.paste(sem.crop((330,110,920,650)).resize((590,540),Image.Resampling.LANCZOS),(50,120));inset=scene(.16,True,zoom=2.0,center=(635,460),only=1)
  # Dedicated local view around the measured second floor; crop retains geometry and boxes.
  crop=inset.crop((270,210,1050,650)).resize((550,310),Image.Resampling.LANCZOS)
  d=ImageDraw.Draw(im);d.rounded_rectangle((700,300,1260,656),radius=12,fill='white',outline=(205,212,222),width=2);im.paste(crop,(705,340));d=ImageDraw.Draw(im);d.text((718,308),'二楼局部 · 房间与物体细节',font=fonts[20],fill=(35,48,68))
  # Label a small number of predicted object centers within the inset.
  n=0
  for o in objects:
   if o['floor']!=2:continue
   q=proj(np.array([o['center_xyz']]),.16,2.0,(635,460),1)[0];x,y=(q-[270,210])*[550/780,310/440]+[705,340]
   if 730<x<1210 and 365<y<620 and n<6:
    d.text((x,y),o['label'],font=fonts[14],fill=cc[o['label']],stroke_width=2,stroke_fill='white');n+=1
  title='房间结构 + 三维物体提取';sub='全局总览与局部语义细节'
 d=ImageDraw.Draw(im);d.rectangle((0,0,W,108),fill=bg);d.text((42,22),title,font=fonts[32],fill=(29,44,64));d.text((44,69),sub,font=fonts[17],fill=(91,107,128));d.line((44,106,1236,106),fill=(220,226,234),width=1)
 d.rectangle((0,672,W,H),fill=bg);d.text((42,683),'实测数据离线结果｜分层展示偏移｜房间含局部人工边界修正｜物体框为预测',font=fonts[14],fill=(105,116,132));d.text((1235,683),f'{t:04.1f}s / 14s',font=fonts[14],fill=(105,116,132),anchor='ra')
 d.line((42,711,42+1196*i/419,711),fill=(39,141,193),width=3)
 if i in [60,165,270,390]:im.save(O/f'preview_{i:03d}.jpg')
 writer.send(np.asarray(im))
 if i%60==0:print('frame',i,flush=True)
writer.close()
(O/'manifest.json').write_text(json.dumps({'duration_seconds':14,'fps':30,'frames':420,'resolution':[W,H],'cloud':str(G),'rooms':str(D),'objects':str(B),'object_count':len(objects),'presentation_only':True,'floor_display_offset_m':3.2,'manual_room_correction':True,'gt_used':False},indent=2))
print(O/'semantic_map_14s.mp4')
