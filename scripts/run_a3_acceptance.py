"""Full A3 acceptance with conservative read-only early-failure monitoring."""
import argparse,json,time,subprocess,math,collections
from pathlib import Path
from exploration_stop_monitor import hold_evidence
p=argparse.ArgumentParser();p.add_argument('case');p.add_argument('--variant',default='A3_observation_lifecycle');a=p.parse_args();root=Path(__file__).resolve().parents[1];out=root/'outputs'/a.case
import hashlib
source=root/'runtime/consistency_validation_20261015'/a.variant
identity=json.loads((source.parent/(a.variant+'_manifest.json')).read_text())['source_sha256']
assert all(hashlib.sha256((source/k).read_bytes()).hexdigest()==v for k,v in identity.items())
proc=subprocess.Popen(['python3',str(root/'scripts/run_consistency_case.py'),a.variant,a.case,'--duration','504','--light-trace']);frames=collections.deque();events=collections.deque();coverage=collections.deque();offset=0;frameoffset=0;last_sequence=-1;trigger=None
try:
 while proc.poll()is None:
  trace=out/'decision_trace.jsonl'
  if trace.exists():
   with trace.open()as f:
    f.seek(offset)
    while True:
     start=f.tell();line=f.readline()
     if not line or not line.endswith('\n'):f.seek(start);break
     try:e=json.loads(line)
     except ValueError:continue
     if e['event']in ['planning_end','emergency_stop','server_reject','published','server_switch']:events.append(e)
    offset=f.tell()
  capture=out/'capture_trace.jsonl'
  if capture.exists():
   with capture.open()as f:
    f.seek(frameoffset)
    while True:
     start=f.tell();line=f.readline()
     if not line or not line.endswith('\n'):f.seek(start);break
     try:r=json.loads(line)
     except ValueError:continue
     frames.append(r)
    frameoffset=f.tell()
  if frames:
   now=frames[-1]['capture_wall_s']
   while frames and frames[0]['capture_wall_s']<now-46:frames.popleft()
   while events and events[0]['wall_s']<now-46:events.popleft()
   vol=out/'floor_volumes.json'
   if vol.exists():
    try:
     v=json.loads(vol.read_text()).get('map_coverage_latest')
     if v and(not coverage or coverage[-1][0]!=v[1]):coverage.append((v[1],v[0]))
    except (ValueError,OSError):pass
   while coverage and coverage[0][0]<now-46:coverage.popleft()
   duration=now-frames[0]['capture_wall_s'];radius=max(math.dist(f['position'],frames[0]['position'])for f in frames)
   failures=sum(e['event']=='planning_end'and e.get('result')==0 for e in events);blocked=sum(f.get('preview_motion_blocked',False)for f in frames);increase=coverage[-1][1]-coverage[0][1]if len(coverage)>1 else None
   enough_coverage=len(coverage)>1 and coverage[-1][0]-coverage[0][0]>=40
   hold=hold_evidence(frames)
   if trigger is None and duration>=45 and radius<.05 and enough_coverage and increase<=.02 and(failures>=15 or blocked>=100 or hold['command_hold_without_progress']):
    trigger={'classification':'early_failed_diagnostic_not_504_comparable','reason':'45s bounded actual displacement and negligible coverage with command hold, repeated planning failure, or geometry blockage',**hold,'window_s':duration,'max_distance_from_window_start_m':radius,'coverage_increment_m3':increase,'planning_failures':failures,'blocked_frames':blocked,'capture_wall_s':now}
    (out/'early_failure.json').write_text(json.dumps(trigger,indent=2));dest=out/'bridge/exploration_complete.json';tmp=dest.with_suffix('.tmp');tmp.write_text(json.dumps(trigger));tmp.replace(dest)
  time.sleep(1)
 rc=proc.wait()
 if rc:raise SystemExit(rc)
 subprocess.run(['python3',str(root/'scripts/audit_consistency_case.py'),str(out)],check=True)
 audit=json.loads((out/'consistency_audit.json').read_text());verdict={'completed_504':audit['capture_duration_s']>=503 and trigger is None,'early_failure':trigger,'actual_duration_s':audit['capture_duration_s'],'source_matches_frozen_candidate':True,'snapshot_diagnostics_enabled':False,'B_C_combined':False};(out/'acceptance_status.json').write_text(json.dumps(verdict,indent=2));print(json.dumps(verdict))
finally:
 if proc.poll()is None:proc.terminate()
