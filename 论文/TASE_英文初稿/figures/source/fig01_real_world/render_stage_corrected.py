from pathlib import Path
import sys,json
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R/'source'))
import numpy as np,trimesh
from PIL import Image,ImageDraw
from vggt.utils.geometry import unproject_depth_map_to_point_map
O=R/'stage_corrected_views';O.mkdir(exist_ok=True)
for name in ['01_laptop','02_toolbox','03_microwave','04_cup']:
 src=R/'stage_corrected_results'/name
 if not (src/'cloud.ply').exists():continue
 g=trimesh.load(src/'cloud.ply');pts=np.asarray(g.vertices);rgb=np.asarray(g.colors)[:,:3];poses=np.load(src/'poses.npz');centers=poses['centers'];rotations=poses['rotations']
 selected_times={'01_laptop':[24,35,37,44,46],'02_toolbox':[49,50,51,52,54],'03_microwave':[60,67,74,82,84],'04_cup':[87,93,98,100,102]}[name]
 observation_times={'01_laptop':[35,46],'02_toolbox':[54],'03_microwave':[84],'04_cup':[102]}[name]
 indices=[int(np.argmin(abs(poses['times']-t))) for t in observation_times]
 frusta=[]
 for fi in indices:
  length=1.2 if name in ['01_laptop','02_toolbox'] else 1.8
  local=np.array([[-.505,-.405,1],[.688,-.405,1],[.688,.388,1],[-.505,.388,1]])*length
  world=local@rotations[fi].T
  corners=world[:,[0,2,1]]*np.array([1,1,-1])+centers[fi]
  edges=[np.array([centers[fi],q]) for q in corners]+[np.array([corners[j],corners[(j+1)%4]]) for j in range(4)]
  frusta.append((fi,corners,edges))
 scene=trimesh.Scene(g);scene.add_geometry(trimesh.load_path(centers),node_name='EKF_path')
 for fi,_,edges in frusta:scene.add_geometry(trimesh.load_path(np.array(edges)),node_name=f'camera_frustum_{fi}')
 scene.export(O/f'{name}_trajectory_frusta.glb')
 variants=[]
 for percentile in [65]:
  p=pts.copy();c=rgb.copy()
  if len(p)>450000:
   ix=np.linspace(0,len(p)-1,450000,dtype=int);p=p[ix];c=c[ix]
  for az,el in [(-60,60),(0,60),(60,60)]:
   a,b=np.deg2rad([az,el]);right=np.array([np.cos(a),0,np.sin(a)]);up=np.array([-np.sin(a)*np.sin(b),np.cos(b),np.cos(a)*np.sin(b)]);basis=np.stack([right,-up],axis=1)
   xy=p@basis;path=centers@basis;low,high=np.quantile(xy,[.005,.995],axis=0);allcorners=np.concatenate([q for _,q,_ in frusta])@basis;low=np.minimum(low,np.minimum(path.min(0),allcorners.min(0)));high=np.maximum(high,np.maximum(path.max(0),allcorners.max(0)));scale=min(900/max(high[0]-low[0],1e-6),620/max(high[1]-low[1],1e-6));origin=(low+high)/2
   xy=(xy-origin)*scale+[500,340];path=(path-origin)*scale+[500,340];ix=xy.astype(int);valid=(ix[:,0]>1)&(ix[:,0]<998)&(ix[:,1]>1)&(ix[:,1]<678);ix=ix[valid];colors=c[valid];depth=p[valid]@np.cross(right,up);order=np.argsort(depth)
   arr=np.full((680,1000,3),255,dtype=np.uint8)
   for dx,dy in [(0,0),(1,0),(0,1)]:arr[ix[order,1]+dy,ix[order,0]+dx]=np.clip(colors[order]*.78+255*.22,0,255).astype('uint8')
   im=Image.fromarray(arr);d=ImageDraw.Draw(im)
   # Execution path is a foreground annotation; render it after geometry and frusta.
   line=[tuple(q) for q in path];d.line(line,fill='white',width=17);d.line(line,fill='#e40046',width=10)
   for jj in [len(path)//3,2*len(path)//3]:
    if jj<1:continue
    delta=path[jj]-path[jj-1];norm=np.linalg.norm(delta)
    if norm<1:continue
    v=delta/norm;normal=np.array([-v[1],v[0]]);tip=path[jj];tri=[tuple(tip),tuple(tip-v*18+normal*9),tuple(tip-v*18-normal*9)];d.polygon(tri,fill='#e40046')
   for j,col in [(0,'#1bc9ac'),(-1,'#ffb400')]:
    x,y=path[j];d.ellipse((x-12,y-12,x+12,y+12),fill=col,outline='white',width=2)
   # Frusta are topmost annotation glyphs, uniformly sized to toolbox reference.
   for fi,corners,edges in frusta:
    anchor=(centers[fi]@basis-origin)*scale+[500,340]
    projected=(np.vstack([centers[fi],corners])@basis-origin)*scale+[500,340]
    reference_extent=113.30126944098703
    factor=reference_extent/max(np.ptp(projected,axis=0).max(),1e-6)
    for edge in edges:
     pp=(edge@basis-origin)*scale+[500,340];pp=anchor+(pp-anchor)*factor
     seg=[tuple(q) for q in pp]
     d.line(seg,fill='white',width=9);d.line(seg,fill='#005bff',width=5)
   im.save(O/f'{name}_p{percentile}_az{az}.png');variants.append((percentile,az,im))
 sheet=Image.new('RGB',(1500,1020),'white');d=ImageDraw.Draw(sheet)
 for i,(q,az,im) in enumerate(variants):
  im=im.resize((500,340));sheet.paste(im,((i%3)*500,(i//3)*340));d.text((i%3*500+10,i//3*340+8),f'Camera frusta; azimuth {az}',fill='black')
 sheet.save(O/f'{name}_variants.jpg',quality=94)
 print(name,flush=True)
