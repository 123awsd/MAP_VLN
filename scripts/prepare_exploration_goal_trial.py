"""Prepare an isolated data-drive experiment, with reproducible source snapshots."""
import argparse,hashlib,json,shutil,re
import yaml
from datetime import datetime, timezone
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--source',default='exploration_baseline_restore_504');p.add_argument('--inherit-source-config',action='store_true');p.add_argument('--gain',type=int,default=64);p.add_argument('--entry-observe',action='store_true',default=None);p.add_argument('--no-entry-observe',dest='entry_observe',action='store_false');a=p.parse_args()
root=Path(__file__).resolve().parents[1];src=root/'outputs'/a.source;out=Path('/data/map_VLN/PRE_MAP_VLN/outputs')/a.name
effective_config=(src/'config.yaml' if a.inherit_source_config else root/'runtime/falcon_selected_yaw_checkpoint/config.yaml').read_text()
if a.entry_observe is None:a.entry_observe='entry_observe: true' in effective_config
execution_clearance=float(yaml.safe_load(effective_config)['fast_planner']['execution_clearance'])
if not Path('/data').is_mount():raise RuntimeError('Data drive is not mounted')
out.mkdir();(out/'bridge').mkdir()
for n in ['config.yaml','trial.launch','run_simulator.sh','assess_bag.py','export_video_cache.py','export_camera_commands.py','restore_original_cloud_sampling.py','run_postprocess.sh']:
 s=(src/n).read_text().replace(a.source,a.name)
 if n=='config.yaml':s=effective_config.replace('min_unique_gain: 64',f'min_unique_gain: {a.gain}').replace('entry_observe: true',f'entry_observe: {str(a.entry_observe).lower()}')
 if n=='run_simulator.sh':s=re.sub(r'--collision-radius\s+[0-9.]+','--collision-radius '+str(execution_clearance),s)
 (out/n).write_text(s)
for folder in ['postprocess','original_style_video']:
 (out/folder).mkdir()
 for f in (src/folder).rglob('*.py'):
  dest=out/folder/f.relative_to(src/folder);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(f.read_text().replace(a.source,a.name))
for folder in ['room_wall_guided_trial','room_apf_latest_trial']:(out/'postprocess'/folder).mkdir(exist_ok=True)
(out/'original_style_video/生成配置.json').write_text((src/'original_style_video/生成配置.json').read_text().replace(a.source,a.name))
(out/'source_snapshot').mkdir();shutil.copy2(out/'config.yaml',out/'source_snapshot/config.yaml');files=list((root/'runtime/falcon_selected_yaw_checkpoint').glob('*.cpp'))+list((root/'runtime/falcon_selected_yaw_checkpoint').glob('*.h'))+[root/'scripts/run_habitat_falcon.py',root/'scripts/habitat_scene_repairs.py'];hashes={}
for f in files:shutil.copy2(f,out/'source_snapshot'/f.name);hashes[f.name]=hashlib.sha256(f.read_bytes()).hexdigest()
m=json.loads((src/'trial_manifest.json').read_text());m.update(storage=str(out),parent_trial=a.source,prepared_at_utc=datetime.now(timezone.utc).isoformat(),entry_observe=a.entry_observe,min_unique_gain=a.gain,execution_clearance_m=execution_clearance,collision_radius_m=execution_clearance,configuration_origin='source' if a.inherit_source_config else 'current_runtime',source_sha256=hashes);(out/'trial_manifest.json').write_text(json.dumps(m,indent=2));(root/'outputs'/a.name).symlink_to(out,target_is_directory=True);print(out)
