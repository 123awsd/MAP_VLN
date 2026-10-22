#!/usr/bin/env python3
"""Display first/last supported views of each changed fused child."""
import argparse,csv,json,itertools
from collections import defaultdict
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from audit_boxer_depth_support import rotation
p=argparse.ArgumentParser(description=__doc__)
for name in ['episode','review','proposals','baseline','candidate','output']:p.add_argument('--'+name,type=Path,required=True)
a=p.parse_args()
def rows(path):
 with path.open()as f:return list(csv.DictReader(f))
proposals=rows(a.proposals);baseline=rows(a.baseline);candidate=rows(a.candidate);groups=defaultdict(list)
for r in json.loads(a.review.read_text())['changed_members']:groups[r['candidate_cluster']].append(r)
def draw_box(draw,row,pos,R,color):
 centre=np.array([float(row[k])for k in ['tx_world_object','ty_world_object','tz_world_object']]);size=np.array([float(row[k])for k in ['scale_x','scale_y','scale_z']]);Q=rotation([float(row[k])for k in ['qx_world_object','qy_world_object','qz_world_object','qw_world_object']]);signs=np.array(list(itertools.product([-1,1],repeat=3)));cam=((signs*size/2)@Q.T+centre-pos)@R
 for i,j in itertools.combinations(range(8),2):
  if np.count_nonzero(signs[i]!=signs[j])!=1 or min(cam[i,2],cam[j,2])<=.01:continue
  uv=320*cam[[i,j],:2]/cam[[i,j],2:3]+[320,240];uv[:,1]*=280/480
  if np.isfinite(uv).all()and np.abs(uv).max()<10000:draw.line([tuple(uv[0]),tuple(uv[1])],fill=color,width=2)
canvas=Image.new('RGB',(1280,330*len(groups)),'white');d=ImageDraw.Draw(canvas)
for n,(cid,views)in enumerate(sorted(groups.items())):
 views=sorted(views,key=lambda r:r['frame'])
 for col,r in enumerate([views[0],views[-1]]):
  with np.load(a.episode/r['frame'])as f:im=Image.fromarray(f['rgb']).resize((640,280));pos=f['position'];R=rotation(f['orientation_xyzw'])
  draw=ImageDraw.Draw(im);pr=proposals[r['source_row']];b=np.array([float(pr[k])for k in ['x1','y1','x2','y2']])*([640/float(pr['img_width']),280/float(pr['img_height'])]*2);draw.rectangle(b.tolist(),outline='blue',width=2)
  draw_box(draw,baseline[r['baseline_cluster']],pos,R,'#208030');draw_box(draw,candidate[cid],pos,R,'#e08010')
  x,y=col*640,n*330;canvas.paste(im,(x,y+50));d.text((x+5,y+5),f"Child {cid} ({candidate[cid]['name']}) | {r['frame']}",fill='black');d.text((x+5,y+25),'Blue: proposal | Green: baseline | Orange: split fusion',fill='black')
canvas.save(a.output)
