from pathlib import Path
import sys,json
import numpy as np,cv2,habitat_sim
from habitat_sim.utils.common import quat_from_angle_axis
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent;sys.path.insert(0,str(ROOT/'PRE_MAP_VLN/scripts'));from habitat_scene_repairs import load_scene_repair
cache=ROOT/'PRE_MAP_VLN/outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed';command=np.load(cache.parent/'command_xyz_yaw.npy');original=np.load(cache/'camera_pose.npy');targets=np.load(cache/'increments.npz')['targets'];config=json.loads((OUT/'生成配置.json').read_text());n=int(config['fps']*config['duration_seconds']);ts=np.linspace(targets[0],targets[-1],n);dense=np.arange(min(original[0,0],command[0,0]),ts[-1]+.01,.01);indices=np.clip(np.searchsorted(command[:,0],dense,side='right')-1,0,len(command)-1);desired=command[indices,4];yaw=float(desired[0]);velocity=0.;history=[];vmax=np.pi/2;amax=np.pi/2;dt=.01
for target in desired:
 error=float(np.arctan2(np.sin(target-yaw),np.cos(target-yaw)));wanted=np.sign(error)*min(vmax,np.sqrt(2*amax*abs(error)));velocity+=np.clip(wanted-velocity,-amax*dt,amax*dt);yaw+=velocity*dt;history.append(yaw)
yaws=np.interp(ts,dense,np.unwrap(history));positions=np.column_stack([np.interp(ts,original[:,0],original[:,i])for i in [1,2,3]])
S=np.array([[0,0,-1],[-1,0,0],[0,1,0]],float);generated=json.loads((ROOT/'PRE_MAP_VLN/outputs/stage1_3d/00337-CFVBbU9Rsyb/dormant_recovery_20260829/generated_3d_config.json').read_text());origin=np.array(generated['initial_agent_habitat_xyz']);cfg=habitat_sim.SimulatorConfiguration();cfg.scene_id=generated['scene'];cfg.scene_dataset_config_file=generated['scene_config'];cfg.enable_physics=True
sensor=habitat_sim.CameraSensorSpec();sensor.uuid='rgb';sensor.sensor_type=habitat_sim.SensorType.COLOR;sensor.resolution=[960,1280];sensor.position=[0.,1.,0.];sensor.hfov=90;ag=habitat_sim.agent.AgentConfiguration();ag.sensor_specifications=[sensor];frames=OUT/'yaw_matched_rgb';frames.mkdir(exist_ok=True);rows=[]
with habitat_sim.Simulator(habitat_sim.Configuration(cfg,[ag]))as sim:
 repair=load_scene_repair(sim,ROOT/'PRE_MAP_VLN/scene_repairs/00337_three_floor_shell/repair.json',generated['scene']);agent=sim.initialize_agent(0)
 for i,(stamp,pos,yaw)in enumerate(zip(ts,positions,yaws)):
  st=agent.get_state();st.position=origin+S.T@(pos-[0,0,1]);st.rotation=quat_from_angle_axis(float(yaw),np.array([0,1,0]));agent.set_state(st);rgb=sim.get_sensor_observations()['rgb'][...,:3];rgb=cv2.resize(rgb,(640,480),interpolation=cv2.INTER_AREA);cv2.imwrite(str(frames/f'rgb_{i:04d}.jpg'),rgb[...,::-1]);optical=agent.get_state().sensor_states['rgb'].rotation
  # Optical RGB quaternion converted from Habitat world to FALCON world.
  import quaternion
  mat=S@quaternion.as_rotation_matrix(optical)@np.diag([1.,-1.,-1.]);q=quaternion.from_rotation_matrix(mat);rows.append([stamp,*pos,q.x,q.y,q.z,q.w])
  if i%120==0:print('RGB',i,'/',n,flush=True)
np.save(OUT/'yaw_matched_camera_pose.npy',np.array(rows));r=np.abs(np.diff(yaws)/np.diff(ts));report={'mode':'offline rerender following planning/pos_cmd yaw with rate and acceleration limits','max_yaw_rate_rad_s':vmax,'max_yaw_acceleration_rad_s2':amax,'measured_output_max_yaw_rate_rad_s':float(r.max()),'position_source':'recorded sensor positions linearly resampled','pointcloud_source':'unchanged original observations','frames':n};(OUT/'yaw_validation.json').write_text(json.dumps(report,indent=2));print('done',report,flush=True)
