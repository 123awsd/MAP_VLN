from pathlib import Path
import sys,json
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R/'source'))
import numpy as np,trimesh
from PIL import Image,ImageDraw
from vggt.utils.geometry import unproject_depth_map_to_point_map
O=R/'map_supported_views';O.mkdir(exist_ok=True)
for name in ['01_laptop','02_toolbox','03_microwave','04_cup']:
 src=R/'map_supported_results'/name
 if not (src/'cloud.ply').exists():continue
 g=trimesh.load(src/'cloud.ply');pts=np.asarray(g.vertices);rgb=np.asarray(g.colors)[:,:3];centers=np.load(src/'poses.npz')['centers']
 variants=[]
 for percentile in [65]:
  p=pts.copy();c=rgb.copy()
  if len(p)>450000:
   ix=np.linspace(0,len(p)-1,450000,dtype=int);p=p[ix];c=c[ix]
  for az,el in [(-60,60),(0,60),(60,60)]:
   a,b=np.deg2rad([az,el]);right=np.array([np.cos(a),0,np.sin(a)]);up=np.array([-np.sin(a)*np.sin(b),np.cos(b),np.cos(a)*np.sin(b)]);basis=np.stack([right,-up],axis=1)
   xy=p@basis;path=centers@basis;low,high=np.quantile(xy,[.005,.995],axis=0);low=np.minimum(low,path.min(0));high=np.maximum(high,path.max(0));scale=min(900/max(high[0]-low[0],1e-6),620/max(high[1]-low[1],1e-6));origin=(low+high)/2
   xy=(xy-origin)*scale+[500,340];path=(path-origin)*scale+[500,340];ix=xy.astype(int);valid=(ix[:,0]>1)&(ix[:,0]<998)&(ix[:,1]>1)&(ix[:,1]<678);ix=ix[valid];colors=c[valid];depth=p[valid]@np.cross(right,up);order=np.argsort(depth)
   arr=np.full((680,1000,3),255,dtype=np.uint8)
   for dx,dy in [(0,0),(1,0),(0,1)]:arr[ix[order,1]+dy,ix[order,0]+dx]=np.clip(colors[order]*.78+255*.22,0,255).astype('uint8')
   im=Image.fromarray(arr);d=ImageDraw.Draw(im);line=[tuple(q) for q in path];d.line(line,fill='white',width=9);d.line(line,fill='#ef476f',width=5)
   for j,col in [(0,'#1bc9ac'),(-1,'#ffb400')]:
    x,y=path[j];d.ellipse((x-9,y-9,x+9,y+9),fill=col,outline='white',width=2)
   im.save(O/f'{name}_p{percentile}_az{az}.png');variants.append((percentile,az,im))
 sheet=Image.new('RGB',(1500,1020),'white');d=ImageDraw.Draw(sheet)
 for i,(q,az,im) in enumerate(variants):
  im=im.resize((500,340));sheet.paste(im,((i%3)*500,(i//3)*340));d.text((i%3*500+10,i//3*340+8),f'confidence percentile {q}; azimuth {az}',fill='black')
 sheet.save(O/f'{name}_variants.jpg',quality=94)
 print(name,flush=True)
