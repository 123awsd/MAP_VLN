#!/usr/bin/env python3
"""Run one isolated case, preserving effective ROS params, binaries and captures."""
import argparse,json,os,pathlib,shutil,subprocess,time,hashlib,signal
ROOT=pathlib.Path(__file__).resolve().parents[1]
CONTAINER='consistency-audit-20261015'
p=argparse.ArgumentParser();p.add_argument('variant');p.add_argument('case');p.add_argument('--duration',type=float,default=504);p.add_argument('--port',type=int,default=11461);p.add_argument('--faults',action='store_true');p.add_argument('--full-candidates',action='store_true');p.add_argument('--light-trace',action='store_true');a=p.parse_args()
source=ROOT/'runtime/consistency_validation_20261015'/a.variant
out=pathlib.Path('/data/map_VLN/PRE_MAP_VLN/outputs')/a.case
if out.exists():raise SystemExit('Fresh output required')
out.mkdir();(out/'bridge').mkdir();(ROOT/'outputs'/a.case).symlink_to(out)
shutil.copytree(source,out/'source_snapshot')
shutil.copy2(source/'config.yaml',out/'config.yaml')
inside='/workspace/shared/outputs/'+a.case
checkpoint='/workspace/shared/runtime/consistency_validation_20261015/'+a.variant
launch=(ROOT/'config/trials/falcon_file_bridge.launch.template').read_text().replace('@TRIAL_DIR@',inside)
launch=launch.replace('</launch>','</launch>')
launch=launch.replace('/planning/pos_cmd"','/planning/pos_cmd /execution/ack /execution/status /uav_simulator/odometry"')
(out/'trial.launch').write_text(launch)
manifest={'variant':a.variant,'duration':a.duration,'base':'exploration_baseline_restore_504','seed':7,'collision_radius_m':.07,'planner_clearance_m':.07,'yaw':'planner','source_sha256':{str(q.relative_to(source)):hashlib.sha256(q.read_bytes()).hexdigest() for q in source.rglob('*') if q.is_file()},'started_wall':time.time()}
manifest['trace_mode']='full_candidates' if not a.light_trace and (a.full_candidates or a.duration<504) else 'light_execution_and_search'
manifest['external_asset_sha256']={str(q.relative_to(ROOT)):hashlib.sha256(q.read_bytes()).hexdigest() for q in [ROOT/'scene_repairs/00337_three_floor_shell/repair.json',ROOT/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb',ROOT/'data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json',ROOT/'scripts/capture_exploration_floor_volumes.py',ROOT/'scripts/run_realtime_trial_planner.sh']}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
def dex(cmd,**kw):return subprocess.run(['docker','exec',CONTAINER,'bash','-lc',cmd],**kw)
env=f'export ROS_MASTER_URI=http://127.0.0.1:{a.port} ROS_HOSTNAME=127.0.0.1; source /opt/ros/noetic/setup.bash; source /workspace/falcon_ws/devel/setup.bash; '
plannerlog=(out/'planner_start.log').open('w')
planner=subprocess.Popen(['docker','exec','-e','EXPLORATION_RESOURCE_DURATION='+str(a.duration+180),'-e','EXPLORATION_CHECKPOINT_DIR='+checkpoint,'-e','CONSISTENCY_TRACE_CANDIDATES='+('1' if manifest['trace_mode']=='full_candidates' else '0'),'-e','EXPLORATION_DIAGNOSTIC_TRACE='+str(out/'decision_trace.jsonl'),CONTAINER,'bash','/workspace/shared/scripts/run_realtime_trial_planner.sh',inside,str(a.port)],stdout=plannerlog,stderr=subprocess.STDOUT)
capture=None;sim=None;injector=None
try:
    deadline=time.monotonic()+360
    while time.monotonic()<deadline:
        if planner.poll() is not None:raise RuntimeError('Planner startup failed: '+str(out/'planner_start.log'))
        if (out/'runtime_source_audit.json').exists():
            check=dex(env+'rosparam get /fast_planner/execution_clearance',stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
            if check.returncode==0:break
        time.sleep(2)
    else:raise RuntimeError('Startup timeout')
    dex(env+f'rosparam dump {inside}/effective_params.yaml',check=True)
    dex(f'python3 /workspace/shared/scripts/snapshot_consistency_runtime.py {inside}/complete_runtime_identity.json',check=True)
    # Preserve all upstream configs: snapshot overrides alone do not identify a run.
    dex(f'cp -r /workspace/falcon_ws/src/FALCON/falcon_planner/pathfinding/config {inside}/upstream_astar_config',check=True)
    capturelog=(out/'floor_capture.log').open('w')
    capture=subprocess.Popen(['docker','exec','-e','EXPLORATION_RESOURCE_DURATION='+str(a.duration+180),CONTAINER,'bash','-lc',env+f'python3 /workspace/shared/scripts/capture_exploration_floor_volumes.py {inside}'],stdout=capturelog,stderr=subprocess.STDOUT)
    if a.faults:
        injector=subprocess.Popen(['docker','exec',CONTAINER,'bash','-lc',env+f'python3 /workspace/shared/scripts/inject_consistency_faults.py {inside}'],stdout=(out/'injection.log').open('w'),stderr=subprocess.STDOUT)
    simcmd=[str(ROOT/'.envs/habitat/bin/python'),str(out/'source_snapshot/run_habitat_falcon.py'),'--scene',str(ROOT/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb'),'--scene-config',str(ROOT/'data/scene_datasets/hm3d/example/hm3d_annotated_example_basis.scene_dataset_config.json'),'--bridge-dir',str(out/'bridge'),'--seed','7','--initial-position','9.714411735534668','-2.6005380153656006','-2.374816417694092','--duration',str(a.duration),'--hz','5','--follow-falcon','--allow-vertical-motion','--use-planner-yaw','--synchronized-frames','--scene-collision-check','--collision-radius','0.07','--scene-repair-config',str(ROOT/'scene_repairs/00337_three_floor_shell/repair.json'),'--completion-file',str(out/'bridge/exploration_complete.json'),'--result-file',str(out/'result.json')]
    (out/'simulator_command.json').write_text(json.dumps(simcmd,indent=2))
    with (out/'habitat.log').open('w') as log:
        sim=subprocess.Popen(simcmd,cwd=ROOT.parent,stdout=log,stderr=subprocess.STDOUT)
        rc=sim.wait(timeout=a.duration+90)
    if rc:raise RuntimeError(f'Habitat failed: {rc}')
    planner.wait(timeout=30)
    capture.wait(timeout=15)
    manifest['completed_wall']=time.time();manifest['status']='completed'
except BaseException as exc:
    manifest['status']='error';manifest['error']=str(exc);raise
finally:
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    if sim and sim.poll() is None:sim.terminate()
    # Shutdown only this master's nodes; never kill other experiments.
    dex(env+'rosnode kill -a',stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    for task in [injector,capture,planner]:
        if task and task.poll() is None:task.terminate()
print(json.dumps({'case':a.case,'status':manifest['status'],'output':str(out)}))
