"""Audit detection/fusion provenance without rerunning any neural network."""
from pathlib import Path
import sys,csv,json
from collections import Counter,defaultdict
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'third_party/boxer'))
import torch
from utils.file_io import read_obb_csv
from utils.fuse_3d_boxes import BoundingBox3DFuser
O=R/'outputs/boxer/coverage700_measured_20261017';D=O/'detections/episode';raw=list(csv.DictReader((D/'boxer_3dbbs.csv').open()));two=list(csv.DictReader((D/'owl_2dbbs.csv').open()))
assert len(raw)==len(two)
for a,b in zip(raw,two):assert a['time_ns']==b['time_ns'] and a['name']==b['name']
score=[{'row':i,'name':a['name'],'score2d':float(b['prob']),'combined':float(a['prob']),'implied_score3d':2*float(a['prob'])-float(b['prob'])}for i,(a,b)in enumerate(zip(raw,two))]
confidence_kept=[i for i,a in enumerate(raw)if float(a['prob'])>=.45]
timed=read_obb_csv(str(D/'boxer_3dbbs.csv'));all_obbs=torch.cat(list(timed.values()),dim=0)
assert np.allclose(all_obbs.T_world_object.t.numpy(),[[float(a[k+'_world_object'])for k in ('tx','ty','tz')]for a in raw],atol=1e-5)
fuser=BoundingBox3DFuser(iou_threshold=.2,min_detections=4,conf_threshold=.45);inst=fuser.fuse(all_obbs,frame_ids=torch.tensor([int(v["time_ns"])for v in raw],dtype=torch.int64))
frames={}
for p in (O/'episode').glob('frame_*.npz'):
 with np.load(p)as a:frames[int(a['time_ns'])]=(a['position'].copy(),a['orientation_xyzw'].copy())
clusters=[]
for k,v in enumerate(inst,1):
 ids=[confidence_kept[i]for i in v.detection_indices];members=[raw[i]for i in ids];ts=sorted(set(int(a['time_ns'])for a in members));pos=np.array([frames[t][0]for t in ts]);span=float(np.linalg.norm(np.ptp(pos,axis=0)))
 clusters.append({'object_id':f'O{k:03d}','winner_label':v.obb.text_string()[0]if isinstance(v.obb.text_string(),list)else v.obb.text_string(),'support_count':len(ids),'unique_frames':len(ts),'position_bbox_diagonal_m':span,'member_labels':dict(Counter(a['name']for a in members)),'raw_row_indices':ids,'below_045_detector_members':sum(float(two[i]['prob'])<.45 for i in ids),'time_span_s':(ts[-1]-ts[0])*1e-9})
counts={label:{'detections_2d':sum(a['name']==label for a in two),'combined_ge045':sum(a['name']==label and float(a['prob'])>=.45 for a in raw),'fused_winner':sum(v['winner_label']==label for v in clusters)}for label in sorted(set(a['name']for a in two))}
report={'raw_rows':len(raw),'two_three_order_labels_match':True,'combined_ge045':len(confidence_kept),'detector_below045_but_combined_pass':sum(v['score2d']<.45 and v['combined']>=.45 for v in score),'mixed_label_clusters':sum(len(v['member_labels'])>1 for v in clusters),'same_frame_repeated_support_clusters':sum(v['support_count']>v['unique_frames']for v in clusters),'camera_position_extent_below025m_clusters':sum(v['position_bbox_diagonal_m']<.25 for v in clusters),'category_counts':counts,'clusters':clusters,'scores':score,'semantic_accuracy_evaluated':False}
(O/'error_chain_audit_postfix.json').write_text(json.dumps(report,indent=2));print({k:v for k,v in report.items()if k not in ('clusters','scores')})
