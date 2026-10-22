"""Check production Habitat adapter against native camera/pose projections."""
import sys,json,csv,subprocess
from pathlib import Path
import numpy as np,torch
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'boxer_ext'),str(R/'third_party/boxer')]
from habitat_loader import HabitatLoader
from utils.tw.pose import PoseTW
from utils.tw.obb import ObbTW
O=R/'outputs/boxer/coverage700_measured_20261017';loader=HabitatLoader(O/'episode');loader._prefetch_thread.join();loader.resize=960;checks=[]
for index in [8,26,99,200,261,287]:
 d=loader.load(index);p=loader.frames[index]
 with np.load(p)as raw:
  pos=raw['position'];q=raw['orientation_xyzw'];rgb=raw['rgb'];depth=raw['depth_m']
 cam=d['cam0'];pose=d['T_world_rig0'];pts=d['sdp_w'];valid=torch.isfinite(pts).all(-1);pts=pts[valid];local=pose.inverse()*pts;uv,ok=cam.unsqueeze(0).project(local.unsqueeze(0));uv,ok=uv[0],ok[0]
 expected=local[:,:2]/local[:,2:3]*torch.tensor([480.,640.])+torch.tensor([480.,480.])
 err=(uv[ok]-expected[ok]).abs().max().item()
 # Independent inverse pinhole at explicit resized pixels.
 yy,xx=np.mgrid[0:960:29,0:960:31];z=d['depth0'][yy,xx];v=z>0;pixels=np.c_[xx[v],yy[v]];xyz=np.c_[(pixels[:,0]-480)*z[v]/480,(pixels[:,1]-480)*z[v]/640,z[v]];world=xyz@pose.R.numpy().T+pos
 back=pose.inverse()*torch.from_numpy(world.astype(np.float32));proj,ok2=cam.unsqueeze(0).project(back.unsqueeze(0));proj=proj[0];roundtrip=float(np.max(np.abs(proj.numpy()-pixels)))
 checks.append({'frame':p.name,'image_shape':list(d['img0'].shape),'depth_shape':list(d['depth0'].shape),'camera_pose_position_error_m':float(np.max(np.abs(pose.t.numpy()-pos))),'rotation_orthogonality_error':float(np.max(np.abs(pose.R.numpy().T@pose.R.numpy()-np.eye(3)))),'camera_project_vs_analytic_error_px':err,'pixel_world_pixel_max_error_px':roundtrip,'valid_sdp_points':int(valid.sum()),'fx_resized':480.,'fy_resized':640.})
 assert err<.01 and roundtrip<.01
# Compare file format with native constructor and reader, not a hand-assumed size convention.
from utils.file_io import read_obb_csv
rows=list(csv.DictReader((O/'detections/episode/boxer_3dbbs.csv').open()));obs=torch.cat(list(read_obb_csv(str(O/'detections/episode/boxer_3dbbs.csv')).values()))
size_err=float(np.max(np.abs(obs.bb3_diagonal.numpy()-[[float(v['scale_'+c])for c in 'xyz']for v in rows])))
cent_err=float(np.max(np.abs(obs.T_world_object.t.numpy()-[[float(v[c+'_world_object'])for c in ['tx','ty','tz']]for v in rows])))
assert size_err<1e-5 and cent_err<1e-5
report={'adapter_projection_checks':checks,'native_csv_size_roundtrip_error_m':size_err,'native_csv_center_roundtrip_error_m':cent_err,'reference_native_commit':'1f86542','scope':'geometric adapter checks only; no claim about object accuracy or inference precision','online_algorithm_changed':False}
(O/'native_geometry_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
