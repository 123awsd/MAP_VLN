from pathlib import Path
import json,sys
sys.path.insert(0,'/tmp/fig7_project_code/real_fly/stage2_runtime/scripts')
from saved_minco_artifact import read_trajectory
H=Path(__file__).resolve().parent
out={}
for cycle in ('initial','updated'):
 start,phases=read_trajectory(H/'minco_trial'/f'{cycle}_minco.txt')
 out[cycle]={'start':start,'phases':[p['points'] for p in phases]}
(H/'minco_trial'/'sampled_minco.json').write_text(json.dumps(out,separators=(',',':')))
