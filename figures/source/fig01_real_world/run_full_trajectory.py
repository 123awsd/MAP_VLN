from pathlib import Path
import sys,time,json,gc,argparse
R=Path(__file__).resolve().parent
sys.path.insert(0,str(R/'source'))
import torch,numpy as np,cv2,trimesh
from vggt.models.vggt import VGGT
from vggt.utils.load_fn import load_and_preprocess_images
from vggt.utils.pose_enc import pose_encoding_to_extri_intri
from vggt.utils.geometry import unproject_depth_map_to_point_map
ap=argparse.ArgumentParser();ap.add_argument('--segment',default='full_trajectory');ap.add_argument('--step',type=float,default=1);args=ap.parse_args()
assert torch.cuda.is_available(),'Run with host GPU access'
bounds={'full_trajectory':(23,102.01)}
a,b=bounds[args.segment]
video=next(Path('/home/uav/map_VLN/真机实验素材/成功场次/实验室4任务加主动搜索').rglob('*_rgb.mp4'));out=R/'fpv_results'/args.segment;out.mkdir(parents=True,exist_ok=True)
c=cv2.VideoCapture(str(video));fps=c.get(cv2.CAP_PROP_FPS);n=c.get(cv2.CAP_PROP_FRAME_COUNT);assert fps>0
paths=[];times=np.arange(a,b,args.step)
for i,t in enumerate(times):
 c.set(cv2.CAP_PROP_POS_MSEC,float(t)*1000);ok,f=c.read()
 if ok:
  p=out/f'frame_{i:03d}.jpg';cv2.imwrite(str(p),f);paths.append(str(p))
c.release();print('Frames',len(paths),'GPU',torch.cuda.get_device_name(0),flush=True)
# Keep camera/depth branches; tracking and duplicate point-head output are not required.
model=VGGT(enable_point=False,enable_track=False).eval()
state=torch.load(R/'weights/model.pt',map_location='cpu',weights_only=True,mmap=True)
state={k:v for k,v in state.items() if not k.startswith(('point_head.','track_head.'))}
model.load_state_dict(state,strict=True);del state;gc.collect();model=model.cuda()
images=load_and_preprocess_images(paths).cuda();torch.cuda.reset_peak_memory_stats();start=time.time()
with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16):pred=model(images)
extr,intr=pose_encoding_to_extri_intri(pred['pose_enc'],images.shape[-2:])
depth=pred['depth'][0].float().cpu().numpy();conf=pred['depth_conf'][0].float().cpu().numpy();extr=extr[0].float().cpu().numpy();intr=intr[0].float().cpu().numpy()
colors=(images.permute(0,2,3,1).float().cpu().numpy()*255).clip(0,255).astype('uint8')
points=unproject_depth_map_to_point_map(depth,extr,intr);peak=torch.cuda.max_memory_allocated()/1e9
np.savez_compressed(out/'predictions.npz',depth=depth,confidence=conf,extrinsics=extr,intrinsics=intr,rgb=colors,times=times[:len(paths)])
keep=np.isfinite(points).all(-1)&(conf>=np.percentile(conf,50))&(depth[...,0]>0)
p=points[keep];col=colors[keep];lo,hi=np.quantile(p,[.005,.995],axis=0);keep=((p>=lo)&(p<=hi)).all(1);p=p[keep];col=col[keep]
if len(p)>600000:
 idx=np.linspace(0,len(p)-1,600000,dtype=int);p=p[idx];col=col[idx]
# OpenCV camera convention -> y-up for display; no inferred UAV flight path.
p=p*np.array([1,-1,-1]);cloud=trimesh.points.PointCloud(p,colors=col);cloud.export(out/'cloud.ply');scene=trimesh.Scene(cloud);scene.export(out/'scene.glb')
report={'segment':args.segment,'source_range_seconds':[a,b],'frames':len(paths),'sample_step_seconds':args.step,'input_shape':list(images.shape),'seconds_inference_and_export':time.time()-start,'peak_allocated_gb':peak,'points':len(p),'gpu':torch.cuda.get_device_name(0),'model':'facebook/VGGT-1B','note':'Onboard first-person RGB; times refer to source video; scale is monocular and uncalibrated.'}
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
