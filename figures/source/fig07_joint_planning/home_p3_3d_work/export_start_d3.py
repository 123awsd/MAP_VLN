from pathlib import Path
import json
H=Path(__file__).resolve().parent; d=json.loads((H/'conditional_plans_start_d3.json').read_text()); out={}
for name,plan in [('initial',d['initial_plan']),('updated',d['updated_plan'])]:
 phases=[]
 for seg,visit in zip(plan['segments'],plan['visits']):
  p=visit['pose']; phases.append([seg['from_xyz_m']]+seg['points_xyz_m']+[[p['x'],p['y'],p['z']]])
 out[name]={'phases':phases}
(H/'start_d3'/'astar_points.json').write_text(json.dumps(out,separators=(',',':')))
