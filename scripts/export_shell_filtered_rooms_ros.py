"""Full-record offline surface/ray evidence reconstruction, not TSDF replay."""
from pathlib import Path
import sys,json,numpy as np,rosbag
R=Path('/workspace/shared');sys.path.insert(0,str(R/'scripts'));from repair_depth_validity import invalidate_safety_shell_depth
O=R/'outputs/coverage700_wall_room_preview/shell_filtered_final';O.mkdir(exist_ok=True)
frames={round(f['capture_wall_s'],6):f for f in map(json.loads,(R/'outputs/coverage_lifecycle_700_r1_20261016/capture_trace.jsonl').read_text().splitlines())};cfg=json.loads((R/'scene_repairs/00337_three_floor_shell/repair_collision_only.json').read_text());shells=[(v['falcon_min'],v['falcon_max'])for v in cfg['objects']];points=[];rayvotes=np.zeros((3,420,380),np.uint16);origin=np.array([-8.5,-2.5]);grounds=[-.155,2.595,5.595];seen=set();counts=[0,0,0]
def rot(q):
 x,y,z,w=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
y,x=np.indices((480,640));sample=(x%4==0)&(y%4==0)
with rosbag.Bag(str(R/'outputs/coverage_lifecycle_700_r1_20261016/trial.bag')) as bag:
 for _,m,_ in bag.read_messages(topics=['/uav_simulator/depth_image']):
  stamp=round(m.header.stamp.to_sec(),6);f=frames.get(stamp)
  if f is None or stamp in seen:continue
  seen.add(stamp);d=np.frombuffer(m.data,'<u2').reshape(m.height,m.step//2)[:,:m.width].astype(np.float32)/1000;position=np.array(f['position']);rotation=rot(f['orientation_xyzw']);filtered,bad=invalidate_safety_shell_depth(d,position,rotation,shells);valid=(filtered>=.15)&(filtered<=5)&sample
  q=np.column_stack(((x[valid]-320)*filtered[valid]/320,(y[valid]-240)*filtered[valid]/320,filtered[valid]))@rotation.T+position;points.append(q.astype(np.float32));counts[0]+=1;counts[1]+=int(bad.sum());counts[2]+=len(q)
  # Deterministic bounded ray sample; only valid endpoints, never old FREE.
  q=q[::max(1,len(q)//160)][:160]
  if len(q):
   delta=q-position;length=np.linalg.norm(delta,axis=1);steps=np.arange(0,5,.05);t=steps[None,:]/np.maximum(length[:,None],1e-6);v=steps[None,:]<(length[:,None]-.15);pts=(position+delta[:,None,:]*t[:,:,None])[v];ij=np.floor((pts[:,:2]-origin)/.05).astype(int);good=(ij[:,0]>=0)&(ij[:,0]<380)&(ij[:,1]>=0)&(ij[:,1]<420)
   for fi,g in enumerate(grounds):
    sel=good&(pts[:,2]>=g+.15)&(pts[:,2]<g+2.75);pix=ij[sel];frame=np.zeros((420,380),np.uint8);frame[pix[:,1],pix[:,0]]=1;rayvotes[fi]+=frame
  if len(points)>=100:
   p=np.concatenate(points);_,ids=np.unique(np.floor(p/.04).astype(np.int32),axis=0,return_index=True);points=[p[ids]]
  if len(seen)%500==0:print('frames',len(seen),flush=True)
p=np.concatenate(points);_,ids=np.unique(np.floor(p/.04).astype(np.int32),axis=0,return_index=True);np.savez_compressed(O/'reconstructed_evidence.npz',points=p[ids],ray_votes=rayvotes);(O/'reconstruction.json').write_text(json.dumps({'matched_frames':counts[0],'invalid_pixels':counts[1],'sampled_valid_endpoints':counts[2],'surface_pixel_stride':4,'surface_voxel_m':.04,'ray_sample_max_per_frame':160,'ray_step_m':.05,'not_online_TSDF_replay':True,'old_FREE_used':False},indent=2));print(counts,flush=True)
