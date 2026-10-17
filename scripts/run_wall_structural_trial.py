from pathlib import Path
import json
import numpy as np
from scipy.spatial import cKDTree
from extract_walls_structural_evidence import extract_with_evidence
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'wall_structural_evidence';O.mkdir(exist_ok=True)
G=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz')
cfg=dict(envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.6,face_density=.25,reject_low_horizontal=True,horizontal_drop=.45,refine_wall_plane=True,wall_tolerance=.04,local_top_attachment=.4)
report=[]
for f,g in enumerate([-.155,2.595,5.595],1):
 p=G[f'p{f-1}'];n=G[f'n{f-1}'];old=np.load(B/'measured'/f'L{f}_small_room_preserved.npz')['wall_points'];oldmask=cKDTree(old).query(p,workers=2)[0]<1e-5
 audit={};print('baseline audit',f,flush=True);baseline,_,_,_=extract_with_evidence(p,n,g,{**cfg,'flat_ceiling_only':f<3,'audit':audit})
 bm=cKDTree(baseline).query(p,workers=2)[0]<1e-5
 assert np.array_equal(bm,oldmask),'Baseline reproduction mismatch'
 print('structural variant',f,flush=True);walls,roof,records,meta=extract_with_evidence(p,n,g,{**cfg,'flat_ceiling_only':f<3,'structural_support':True})
 newmask=cKDTree(walls).query(p,workers=2)[0]<1e-5
 row={'floor':f,'baseline_exact_match':True,'old_points':int(oldmask.sum()),'new_points':int(newmask.sum()),'added':int((newmask&~oldmask).sum()),'removed':int((oldmask&~newmask).sum()),'rejection_exposures':{k:int((v&~oldmask).sum())for k,v in audit.items()}}
 if f==1:
  roi=(abs(p[:,0]-3.025)<.12)&(p[:,1]>12)&(p[:,1]<15.8)&(abs(n[:,2])<.35)&(p[:,2]>g+.15)
  row['diagnostic_wall']={'candidates':int(roi.sum()),'before':int((roi&oldmask).sum()),'after':int((roi&newmask).sum()),'rejection_exposures':{k:int((v&roi&~oldmask).sum())for k,v in audit.items()}}
 np.savez_compressed(O/f'L{f}_walls.npz',wall_points=walls,old_selected=oldmask,new_selected=newmask,**audit)
 report.append(row);print(row,flush=True)
(O/'audit.json').write_text(json.dumps({'uses_GT':False,'coordinate_rules':False,'note':'rejection exposures overlap across proposals; not exclusive attribution','results':report},ensure_ascii=False,indent=2))
