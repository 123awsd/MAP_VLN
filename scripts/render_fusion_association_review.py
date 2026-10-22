#!/usr/bin/env python3
"""Display exact same-frame fusion members and the projected fused 3D box."""
import argparse,csv,json,itertools
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from audit_boxer_depth_support import rotation
p=argparse.ArgumentParser(description=__doc__)
for name in ['episode','proposals','fused','review','output']:p.add_argument('--'+name,type=Path,required=True)
p.add_argument('--candidate-audit',type=Path)
p.add_argument('--candidate-fused',type=Path)
a=p.parse_args()
if bool(a.candidate_audit)!=bool(a.candidate_fused):p.error('Both candidate inputs required')
def rows(path):
 with path.open()as f:return list(csv.DictReader(f))
proposals=rows(a.proposals);fused=rows(a.fused);review=json.loads(a.review.read_text());frames={}
for path in a.episode.glob('frame_*.npz'):
 with np.load(path)as f:frames[int(f['time_ns'])]=path
candidate_instances=json.loads(a.candidate_audit.read_text())['instances'] if a.candidate_audit else []
candidate_rows=rows(a.candidate_fused) if a.candidate_fused else []
cases=[r for r in review['clusters']if r['separated_proposal_pair_count']][:6]
canvas=Image.new('RGB',(1280,330*((len(cases)+1)//2)),'white');d=ImageDraw.Draw(canvas)
for n,c in enumerate(cases):
 pair=min(c['separated_proposal_pairs'],key=lambda r:r['proposal_iou']);path=frames[pair['time_ns']]
 with np.load(path)as f:im=Image.fromarray(f['rgb']).resize((640,280));position=f['position'];R=rotation(f['orientation_xyzw'])
 draw=ImageDraw.Draw(im)
 for color,index in zip(['red','blue'],pair['source_rows']):
  row=proposals[index];b=[float(row[k])for k in ['x1','y1','x2','y2']];b=[b[0]*640/float(row['img_width']),b[1]*280/float(row['img_height']),b[2]*640/float(row['img_width']),b[3]*280/float(row['img_height'])]
  draw.rectangle(b,outline=color,width=3);draw.text((max(0,b[0]),max(0,b[1])),row['name'],fill=color)
 projected=[(fused[c['fused_index']], '#208030')]
 for instance in candidate_instances:
  if set(pair['source_rows']) & set(instance['source_csv_row_indices']):projected.append((candidate_rows[instance['fused_index']], '#e08010'))
 for row,color in projected:
  centre=np.array([float(row[k])for k in ['tx_world_object','ty_world_object','tz_world_object']]);size=np.array([float(row[k])for k in ['scale_x','scale_y','scale_z']]);Q=rotation([float(row[k])for k in ['qx_world_object','qy_world_object','qz_world_object','qw_world_object']]);signs=np.array(list(itertools.product([-1,1],repeat=3)));world=(signs*size/2)@Q.T+centre;camera=(world-position)@R
  for i,j in itertools.combinations(range(8),2):
   if np.count_nonzero(signs[i]!=signs[j])!=1 or min(camera[i,2],camera[j,2])<=.01:continue
   uv=320*camera[[i,j],:2]/camera[[i,j],2:3]+np.array([320,240]);uv[:,1]*=280/480
   if np.isfinite(uv).all() and np.abs(uv).max()<10000:draw.line([tuple(uv[0]),tuple(uv[1])],fill=color,width=2)
 x,y=(n%2)*640,(n//2)*330;canvas.paste(im,(x,y+50));d.text((x+5,y+5),f"Cluster {c['fused_index']} ({c['name']}) | {path.name}",fill='black');d.text((x+5,y+24),'Red/blue: members | Green: original | Orange: constrained',fill='black')
canvas.save(a.output)
