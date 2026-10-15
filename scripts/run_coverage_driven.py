"""Experiment supervisor: resource cap is not exploration completion.
No GT inputs, no modification to planner motion or scoring.
"""
import argparse,collections,json,math,subprocess,time,shutil
from pathlib import Path
from exploration_stop_monitor import hold_evidence
from coverage_progress_monitor import classify_no_progress,checkpoint_evidence
p=argparse.ArgumentParser();p.add_argument('variant');p.add_argument('case');p.add_argument('--resource-cap',type=float,default=1800);a=p.parse_args();root=Path(__file__).resolve().parents[1];out=root/'outputs'/a.case
if not math.isfinite(a.resource_cap)or a.resource_cap<=0:raise SystemExit('positive finite resource cap required')
if out.exists():raise SystemExit('Fresh output required')
vol=None
proc=subprocess.Popen(['python3',str(root/'scripts/run_consistency_case.py'),a.variant,a.case,'--duration',str(a.resource_cap),'--light-trace']);frames=collections.deque();coverage=collections.deque();events=collections.deque();fo=eo=0;checkpoints=set();reason=None;last_status=0
try:
 while proc.poll()is None:
  for name,isframes in [('capture_trace.jsonl',True),('decision_trace.jsonl',False)]:
   path=out/name
   if not path.exists():continue
   with path.open()as f:
    f.seek(fo if isframes else eo)
    while True:
     pos=f.tell();line=f.readline()
     if not line or not line.endswith('\n'):f.seek(pos);break
     try:e=json.loads(line)
     except ValueError:continue
     if isframes:frames.append(e)
     elif e['event']in ['planning_end','emergency_stop','observation_selected','observation_executing','observation_cancelled','observation_settled']:events.append(e)
    if isframes:fo=f.tell()
    else:eo=f.tell()
  if frames:
   now=frames[-1]['capture_wall_s'];first=frames[0]['capture_wall_s']
   if 't0'not in locals():t0=first
   elapsed=now-t0
   while frames and frames[0]['capture_wall_s']<now-181:frames.popleft()
   while events and events[0]['wall_s']<now-181:events.popleft()
   fv=out/'floor_volumes.json'
   if fv.exists():
    try:
     vol=json.loads(fv.read_text());v=vol.get('map_coverage_latest')
     if v and(not coverage or coverage[-1][0]!=v[1]):coverage.append((v[1],v[0]))
    except (ValueError,OSError):pass
   while coverage and coverage[0][0]<now-181:coverage.popleft()
   for cp in [300,504,900,1200,1500,1800]:
    if elapsed>=cp and cp not in checkpoints:
     checkpoints.add(cp);d=out/'coverage_checkpoints';d.mkdir(exist_ok=True);(d/f'{cp}s.json').write_text(json.dumps(checkpoint_evidence(cp,elapsed,frames[-1],vol),indent=2))
   short=[f for f in frames if f['capture_wall_s']>=now-60];cov=[v for v in coverage if v[0]>=now-65];radius=max((math.dist(f['position'],short[0]['position'])for f in short),default=0);gain=cov[-1][1]-cov[0][1]if len(cov)>1 else None;hold=hold_evidence(short);blocked=sum(f.get('preview_motion_blocked',False)for f in short)
   # Never declare saturation from these conditions: only a failed/no-progress diagnostic.
   if reason is None:
    reason=classify_no_progress(list(frames),list(coverage))
    if reason:reason['elapsed_s']=elapsed
   if reason is None and elapsed>120:
    available=shutil.disk_usage(out).free
    if available<25*1024**3:reason={'classification':'disk_resource_limit_incomplete','available_bytes':available,'elapsed_s':elapsed}
   if reason is not None and not(out/'coverage_stop.json').exists():
    (out/'coverage_stop.json').write_text(json.dumps(reason,indent=2));dest=out/'bridge/exploration_complete.json';tmp=dest.with_suffix('.tmp');tmp.write_text(json.dumps(reason));tmp.replace(dest)
   if elapsed-last_status>=30:
    last_status=elapsed;(out/'coverage_live_status.json').write_text(json.dumps({'elapsed_s':elapsed,'known_m3':coverage[-1][1]if coverage else None,'window60_radius_m':radius,'window60_known_gain_m3':gain,'hold':hold,'blocked_frames60':blocked,'recent_events':dict(collections.Counter(e['event']for e in events)),'stop':reason},indent=2))
  time.sleep(2)
 rc=proc.wait()
 if not out.exists():raise SystemExit(rc or 'runner produced no output')
 (out/'coverage_supervisor_result.json').write_text(json.dumps({'returncode':rc,'stop':reason,'resource_cap_s':a.resource_cap,'checkpoints':sorted(checkpoints),'completion_claim':False,'note':'Resource cap/stall is not saturation. Requires offline opportunity/frontier/task audit.'},indent=2));raise SystemExit(rc)
finally:
 if proc.poll()is None:proc.terminate()
