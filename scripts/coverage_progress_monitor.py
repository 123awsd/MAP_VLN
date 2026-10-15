"""Pure experiment evidence checks; never declares coverage saturation."""
import math
from exploration_stop_monitor import hold_evidence

def classify_no_progress(frames,coverage,window_s=60.):
    if not frames:return None
    now=frames[-1]['capture_wall_s']
    q=[f for f in frames if now-window_s<=f['capture_wall_s']<=now]
    c=[v for v in coverage if now-window_s-5<=v[0]<=now]
    if len(q)<2 or now-q[0]['capture_wall_s']<window_s-1:return None
    if len(c)<2 or c[-1][0]-c[0][0]<window_s-2 or now-c[-1][0]>5:return None
    # Missing frames cannot be treated as continuous stillness.
    if max(b['capture_wall_s']-a['capture_wall_s']for a,b in zip(q,q[1:]))>2:return None
    radius=max(math.dist(f['position'],q[0]['position'])for f in q)
    gain=c[-1][1]-c[0][1]
    if gain<-.01:return None  # reset/inconsistent counter is not zero progress
    hold=hold_evidence(q);blocked=[f for f in q if f.get('preview_motion_blocked',False)]
    blocked_ids={f.get('blocked_trajectory_id')for f in blocked}
    repeated_block=len(blocked)>=100
    if radius>=.08 or gain>.03 or not(hold['command_hold_without_progress']or repeated_block):return None
    return {'classification':'stalled_incomplete','radius_60s_m':radius,'coverage_gain_m3':gain,'blocked_frames':len(blocked),'blocked_trajectory_ids':sorted(v for v in blocked_ids if v is not None),**hold}

def checkpoint_evidence(requested,elapsed,frame,volume):
    capture=frame['capture_wall_s'];copy=volume if volume else None
    stamps=(volume or {}).get('snapshot_wall_times',{})
    lags={k:capture-v for k,v in stamps.items()}
    return {'requested_capture_elapsed_s':requested,'actual_capture_elapsed_s':elapsed,'snapshot':copy,'capture':frame,'occupancy_component_lag_from_capture_s':lags,'snapshot_is_consistent_map':False,'offline_surface_and_motion_audit_required':True,'note':'Asynchronous published grids; not an exact simultaneous occupancy snapshot. Capture elapsed is wall-clock sensor interval; no independent physics clock is recorded.'}
