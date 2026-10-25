from pathlib import Path
import sys,json
import numpy as np,cv2,trimesh
R=Path(__file__).resolve().parent/'odom_correction';root=R.parent
base=Path('/home/uav/map_VLN/真机实验素材/成功场次/实验室4任务加主动搜索')
p=next((base/'02_地图与场景/fastlio_complete').glob('*.pcd'))
with p.open('rb') as f:
 while not f.readline().startswith(b'DATA'):pass
 offset=f.tell()
m=np.memmap(p,dtype='<f4',mode='r',offset=offset).reshape(-1,4);m=np.asarray(m[::8,:3]).copy();m=m[np.isfinite(m).all(1)];vox=np.floor(m/.06).astype('int32');_,idx=np.unique(vox,axis=0,return_index=True);m=m[idx];print('map support',len(m),flush=True)
ki=json.loads((R/'camera_info.json').read_text());K=np.array(ki['K']).reshape(3,3);K[0]*=518/1280;K[1]*=294/720;invK=np.linalg.inv(K)
for name in ['full_trajectory']:
 src=root/('fpv_tuned_results' if name=='02_toolbox' else 'fpv_results')/name;z=dict(np.load(src/'predictions.npz'));keep=z['times']<=102 if name=='04_cup' else np.ones(len(z['times']),bool);z={k:v[keep] for k,v in z.items()}
 pose=np.load(root/'odom_corrected_results'/name/'poses.npz');rot=pose['rotations'];viewpos=pose['centers'];pos=viewpos[:,[0,2,1]]*np.array([1,-1,1]);allp=[];allc=[];scales=[];coverage=[]
 H,W=z['depth'].shape[1:3];yy,xx=np.indices((H,W));rays=np.stack([xx,yy,np.ones_like(xx)],-1)@invK.T
 for i in range(len(pos)):
  near=m[np.linalg.norm(m-pos[i],axis=1)<18];cam=(near-pos[i])@rot[i];valid=(cam[:,2]>.3);cam=cam[valid];uv=cam@K.T;uv=np.round(uv[:,:2]/uv[:,2:]).astype(int);valid=(uv[:,0]>=0)&(uv[:,0]<W)&(uv[:,1]>=0)&(uv[:,1]<H);uv=uv[valid];cam=cam[valid]
  dep=np.full(H*W,np.inf);np.minimum.at(dep,uv[:,1]*W+uv[:,0],cam[:,2]);dep=dep.reshape(H,W).astype('float32');dep=cv2.erode(dep,np.ones((3,3),np.uint8));finite=np.isfinite(dep)
  dv=z['depth'][i,...,0];cf=z['confidence'][i];use=finite&(dv>0)&(cf>np.percentile(cf,50));ratio=dep[use]/dv[use]
  if len(ratio)<100:continue
  scale=np.median(ratio);scales.append(float(scale));vdepth=dv*scale;residual=np.abs(vdepth-dep)
  good=finite&(cf>=np.percentile(cf,40))&(residual<np.maximum(.6,dep*.20));coverage.append(float(good.mean()))
  fused=np.where(good,dep*.85+vdepth*.15,0);pts=rays*fused[...,None];pts=pts@rot[i].T+pos[i];allp.append(pts[good]);allc.append(z['rgb'][i][good])
 points=np.concatenate(allp);colors=np.concatenate(allc);vox=np.floor(points/.04).astype(int);_,idx,cnt=np.unique(vox,axis=0,return_inverse=True,return_counts=True);pts=np.column_stack([np.bincount(idx,weights=points[:,j])/cnt for j in range(3)]);col=np.column_stack([np.bincount(idx,weights=colors[:,j])/cnt for j in range(3)]).astype('uint8')
 out=root/'map_supported_results'/name;out.mkdir(parents=True,exist_ok=True);view=lambda x:x[:,[0,2,1]]*np.array([1,1,-1]);g=trimesh.points.PointCloud(view(pts),colors=col);g.export(out/'cloud.ply');scene=trimesh.Scene(g);scene.add_geometry(trimesh.load_path(viewpos),node_name='EKF_camera_reference');scene.export(out/'scene.glb')
 import shutil
 shutil.copyfile(root/'odom_corrected_results'/name/'poses.npz',out/'poses.npz')
 for f in src.glob('frame_*.jpg'):shutil.copyfile(f,out/f.name)
 (out/'report.json').write_text(json.dumps({'method':'LiDAR-supported VGGT depth with recorded EKF camera poses','depth_fusion':'0.85 projected prior LiDAR depth + 0.15 scaled VGGT depth; only confidence-filtered agreeing pixels retained','depth_scales_per_frame':scales,'mean_pixel_coverage':float(np.mean(coverage)),'point_count':len(pts),'caveat':'Geometry uses the prior LiDAR map; not pure VGGT. Approximate video synchronization; onboard EKF is not external ground truth.'},indent=2));print(name,len(pts),np.median(scales),np.mean(coverage),flush=True)
