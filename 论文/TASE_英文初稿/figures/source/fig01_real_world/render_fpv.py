import sys

import trimesh,numpy as np
from PIL import Image,ImageDraw
from pathlib import Path
root=Path(__file__).parent
out=root/'fpv_views';out.mkdir(exist_ok=True)
def render(p,az,el):
 scene=trimesh.Scene(trimesh.load(p.with_name('cloud.ply')))
 vs=[];cs=[]
 for name in scene.graph.nodes_geometry:
  transform,key=scene.graph[name];g=scene.geometry[key]
  if len(g.vertices)<100:continue
  v=trimesh.transform_points(g.vertices,transform)
  c=g.colors if hasattr(g,'colors') else g.visual.vertex_colors
  vs.append(v);cs.append(c[:,:3])
 v=np.concatenate(vs);c=np.concatenate(cs)
 lo,hi=np.quantile(v,[.015,.985],axis=0);keep=((v>=lo)&(v<=hi)).all(1);v=v[keep];c=c[keep];v-=np.median(v,axis=0)
 a,e=np.deg2rad([az,el]);right=np.array([np.cos(a),0,np.sin(a)]);up=np.array([-np.sin(a)*np.sin(e),np.cos(e),np.cos(a)*np.sin(e)]);depth=np.cross(right,up)
 xy=np.stack([v@right,-v@up],1);z=v@depth
 low,high=np.quantile(xy,[.005,.995],axis=0);scale=min(760/(high[0]-low[0]),620/(high[1]-low[1]));xy=(xy-(low+high)/2)*scale+[400,340]
 ix=xy.astype(int);keep=(ix[:,0]>1)&(ix[:,0]<798)&(ix[:,1]>1)&(ix[:,1]<678)
 ix=ix[keep];c=c[keep];z=z[keep];order=np.argsort(z)
 arr=np.full((680,800,3),255,dtype=np.uint8)
 for dx,dy in [(0,0),(1,0),(0,1)]:arr[ix[order,1]+dy,ix[order,0]+dx]=c[order]
 return Image.fromarray(arr)
for p in sorted((root/'fpv_results').glob('*/scene.glb')):
 sheet=Image.new('RGB',(2400,1360),'white')
 for j,(a,e) in enumerate([(-30,25),(0,25),(30,25),(-30,45),(0,45),(30,45)]):
  im=render(p,a,e);im.save(out/f'{p.parent.name}_{j}.png');sheet.paste(im,(j%3*800,j//3*680))
 sheet.thumbnail((1200,680));sheet.save(out/f'{p.parent.name}_views.jpg')
 print(p.parent.name,flush=True)
