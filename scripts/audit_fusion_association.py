#!/usr/bin/env python3
"""Find same-frame, spatially separate 2D members inside fused 3D clusters.

This checks exact CSV association; image separation is a review cue, not truth.
"""
import argparse,csv,json,itertools
from collections import defaultdict
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--boxes',type=Path,required=True)
p.add_argument('--proposals',type=Path,required=True)
p.add_argument('--fusion-audit',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
def read(path):
 with path.open()as f:return list(csv.DictReader(f))
boxes,proposals=read(a.boxes),read(a.proposals)
if len(boxes)!=len(proposals):raise ValueError('Exact 2D/3D row association unavailable')
for b,r in zip(boxes,proposals):
 if (b['time_ns'],b['name'])!=(r['time_ns'],r['name']):raise ValueError('Row order/name association differs')
def overlap(i,j):
 x,y=proposals[i],proposals[j];a=[float(x[k])for k in ['x1','y1','x2','y2']];b=[float(y[k])for k in ['x1','y1','x2','y2']]
 inter=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]));union=max(0,a[2]-a[0])*max(0,a[3]-a[1])+max(0,b[2]-b[0])*max(0,b[3]-b[1])-inter
 return inter/union if union>0 else None
records=[]
for instance in json.loads(a.fusion_audit.read_text())['instances']:
 frames=defaultdict(list)
 for i in instance['source_csv_row_indices']:frames[boxes[i]['time_ns']].append(i)
 pairs=[]
 for stamp,ids in frames.items():
  for i,j in itertools.combinations(ids,2):
   iou=overlap(i,j)
   if iou is not None and iou<.1:pairs.append({'time_ns':int(stamp),'source_rows':[i,j],'labels':[boxes[i]['name'],boxes[j]['name']],'proposal_iou':iou})
 records.append({'fused_index':instance['fused_index'],'name':instance['name'],'same_frame_multiple_members':sum(len(ids)>1 for ids in frames.values()),'separated_proposal_pair_count':len(pairs),'separated_proposal_pairs':pairs})
a.output.write_text(json.dumps({'clusters':records,'exact_2d_3d_row_association_passed':True,'clusters_with_separated_same_frame_members':sum(r['separated_proposal_pair_count']>0 for r in records),'fusion_correctness_proven':False,'limitations':'Two distinct projected proposals may reflect duplicate parts or occlusion; spatial separation is not an automatic cannot-link rule. Review exact members and 3D geometry before changing fusion.'},indent=2)+'\n')
print('Clusters with separate same-frame proposals:',sum(r['separated_proposal_pair_count']>0 for r in records))
