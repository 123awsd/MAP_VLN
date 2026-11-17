from pathlib import Path
import json
import numpy as np,trimesh
from scipy.spatial import cKDTree
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parent;O=R/'full_trajectory_scene';O.mkdir(exist_ok=True)
z=np.load(R/'odom_corrected_results/full_trajectory/poses.npz');cent=z['centers'];rot=z['rotations'];times=z['times']
a=trimesh.load(R/'mapping_rgbd/mapping_rgbd.ply');p=np.asarray(a.vertices);col=np.asarray(a.colors)[:,:3];dist,_=cKDTree(cent[:,[0,2]]).query(p[:,[0,2]],workers=4);keep=(dist<3.5)&(p[:,1]>-.4)&(p[:,1]<2.7);p=p[keep];col=col[keep]
g=trimesh.load(R/'map_supported_results/full_trajectory/cloud.ply');q=np.asarray(g.vertices);qc=np.asarray(g.colors)[:,:3];d,_=cKDTree(p).query(q,workers=4);q=q[d<.18];qc=qc[d<.18];d,_=cKDTree(q).query(p,workers=4);keep=d>.065;p=np.concatenate([p[keep],q]);col=np.concatenate([col[keep],qc]);vox=np.floor(p/.04).astype(int);_,ix,cnt=np.unique(vox,axis=0,return_inverse=True,return_counts=True);p=np.column_stack([np.bincount(ix,weights=p[:,k])/cnt for k in range(3)]);col=np.column_stack([np.bincount(ix,weights=col[:,k])/cnt for k in range(3)]).astype('uint8');cloud=trimesh.points.PointCloud(p,colors=col);cloud.export(O/'fused_scene.ply');scene=trimesh.Scene(cloud);scene.add_geometry(trimesh.load_path(cent),node_name='Recorded_EKF_camera_trajectory')
observations=[(35,'A1: Lounge'),(46,'A2: Laptop recovered'),(54,'B: Toolbox'),(84,'C: Microwave'),(102,'D: Cup')];frusta=[]
for t,label in observations:
 i=int(np.argmin(abs(times-t)));corn=np.array([[-.505,-.405,1],[.688,-.405,1],[.688,.388,1],[-.505,.388,1]])*1.4;world=corn@rot[i].T;corn=world[:,[0,2,1]]*np.array([1,1,-1])+cent[i];edges=[np.array([cent[i],c]) for c in corn]+[np.array([corn[j],corn[(j+1)%4]]) for j in range(4)];frusta.append((i,label,corn,edges));scene.add_geometry(trimesh.load_path(np.asarray(edges)),node_name=label)
scene.export(O/'full_trajectory.glb');np.savetxt(O/'trajectory.csv',np.column_stack([times,z['bag_seconds'],cent]),delimiter=',',header='video_sec,bag_sec,viewer_x_m,viewer_y_m,viewer_z_m',comments='')
font='/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf';bold='/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf'
def render(az,el):
 W,H=2000,1400;a,b=np.deg2rad([az,el]);right=np.array([np.cos(a),0,np.sin(a)]);up=np.array([-np.sin(a)*np.sin(b),np.cos(b),np.cos(a)*np.sin(b)]);B=np.stack([right,-up],1);xy=p@B;path=cent@B;lo,hi=np.quantile(xy,[.002,.998],axis=0);lo=np.minimum(lo,path.min(0));hi=np.maximum(hi,path.max(0));scale=min(1800/(hi[0]-lo[0]),1080/(hi[1]-lo[1]));org=(lo+hi)/2
 project=lambda v:(v@B-org)*scale+[1000,740]
 xy=project(p);path=project(cent);idx=np.round(xy).astype(int);valid=(idx[:,0]>1)&(idx[:,0]<W-2)&(idx[:,1]>130)&(idx[:,1]<H-80);idx=idx[valid];cc=col[valid];order=np.argsort(p[valid]@np.cross(right,up));arr=np.full((H,W,3),255,dtype='uint8')
 for dx,dy in [(0,0),(1,0),(0,1)]:arr[idx[order,1]+dy,idx[order,0]+dx]=cc[order]
 im=Image.fromarray(arr);d=ImageDraw.Draw(im);d.text((45,25),'Full real-world execution trajectory',font=ImageFont.truetype(bold,42),fill='#111111');d.text((45,85),'Joint VGGT reconstruction + mapping RGB-D + LiDAR support + recorded EKF poses',font=ImageFont.truetype(font,26),fill='#333333')
 d.line([tuple(x) for x in path],fill='white',width=15);d.line([tuple(x) for x in path],fill='#e40046',width=8)
 for i,label,corn,edges in frusta:
  anchor=path[i];proj=project(np.vstack([cent[i],corn]));factor=65/max(np.ptp(proj,axis=0).max(),1e-6)
  for edge in edges:
   v=anchor+(project(edge)-anchor)*factor;seg=[tuple(x) for x in v];d.line(seg,fill='white',width=7);d.line(seg,fill='#005bff',width=3)
  x,y=anchor;d.ellipse((x-6,y-6,x+6,y+6),fill='#ffb400',outline='white',width=2)
  dx,dy={'A1: Lounge':(20,-42),'A2: Laptop recovered':(-280,-24),'B: Toolbox':(20,-38),'C: Microwave':(20,-40),'D: Cup':(20,16)}[label];tx,ty=x+dx,y+dy;ff=ImageFont.truetype(bold,24);tx=max(15,min(tx,W-d.textlength(label,font=ff)-15));box=d.textbbox((tx,ty),label,font=ff);d.rounded_rectangle((box[0]-5,box[1]-3,box[2]+5,box[3]+3),radius=4,fill='white');d.text((tx,ty),label,font=ff,fill='#14294b')
 x,y=path[0];d.ellipse((x-10,y-10,x+10,y+10),fill='#00ad8e',outline='white',width=2);d.text((x+15,y+10),'Start',font=ImageFont.truetype(bold,25),fill='#006d59')
 d.text((40,H-65),'Reference-assisted visualization. EKF is onboard odometry; video synchronization is approximate.',font=ImageFont.truetype(font,23),fill='#333333');return im
for az in [-60,0,60]:
 im=render(az,55);im.save(O/f'view_{az}.png',dpi=(300,300));print('rendered',az,flush=True)
sheet=Image.new('RGB',(3000,700),'white')
for j,az in enumerate([-60,0,60]):a=Image.open(O/f'view_{az}.png');a.thumbnail((1000,700));sheet.paste(a,(j*1000,0))
sheet.save(O/'views_comparison.jpg',quality=95)
(O/'method.json').write_text(json.dumps({'VGGT':'80 first-person frames jointly processed, encoded video time 23–102 seconds, 1-second sampling','geometry':'VGGT predicted depths anchored to prior LiDAR and EKF poses, fused with archived mapping RGB-D geometry','trajectory':'Actual recorded EKF camera poses, approximate time alignment, not external ground truth','observation_times':observations,'points':len(p),'flight_points_accepted':len(q)},indent=2))
