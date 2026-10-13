from pathlib import Path
import json,numpy as np
ROOT=Path(__file__).resolve().parents[1]; OUT=ROOT/'outputs/room_quality_trial'; base=[]; stair=[]
for f in range(1,4):
 d=np.load(OUT/f'Q_楼梯独立空间_L{f}.npz'); base.append(d); mp=json.load(open(OUT/f'L{f}_stair_label_mapping.json')); rid=int(next(iter(mp))); stair.append(d['labels']==rid)
common=stair[0]&stair[1]&stair[2]
areas=[]
for f,d in enumerate(base,1):
 labels=d['labels'].copy(); mp=json.load(open(OUT/f'L{f}_stair_label_mapping.json')); rid=int(next(iter(mp)))
 fringe=(labels==rid)&~common
 # Keep observed stair approach cells as a separate logical label; only the common core is SF1.
 labels[common]=0
 approach=int(labels.max()+1); labels[fringe]=approach
 core=int(labels.max()+1); labels[common]=core
 assert np.all((labels>0)<=d['domain'])
 np.savez_compressed(OUT/f'R_楼梯统一面积_L{f}.npz',labels=labels,domain=d['domain'],wall=d['wall'],origin=d['origin'],resolution=d['resolution'])
 (OUT/f'R{f}_stair_label_mapping.json').write_text(json.dumps({str(core):'SF1',str(approach):'SF1连接带'},ensure_ascii=False,indent=2))
areas.append(float(common.sum()*float(d['resolution'])**2))
(OUT/'stairspaces_equal_area.json').write_text(json.dumps({'policy':'common observed XY footprint across all three floors; fringe returned to nearest observed room','common_area_m2':areas,'per_floor_area_m2':areas,'source':'intersection of three observed stairwell masks','changed_planner':False},ensure_ascii=False,indent=2))
print('common_area_m2',areas)
