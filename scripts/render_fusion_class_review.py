#!/usr/bin/env python3
"""Show original RGB and class proposals for ambiguous fusion members.

Same-class image proposals are contextual review evidence, not instance truth.
"""
import argparse,csv,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--audit',type=Path,required=True)
p.add_argument('--episode',type=Path,required=True)
p.add_argument('--proposals',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
r=json.loads(a.audit.read_text())
clusters=sorted([i for i in r['instances'] if i['mixed_class_review_required']],key=lambda i:i['selected_label_member_fraction'])[:6]
proposals={}
with a.proposals.open() as f:
 for row in csv.DictReader(f):proposals.setdefault(int(row['time_ns']),[]).append(row)
canvas=Image.new('RGB',(1280,len(clusters)*330),'white');d=ImageDraw.Draw(canvas)
reviews=[]
for ci,c in enumerate(clusters):
 frames=c['support_frames'];selected=[frames[0],frames[len(frames)//2]]
 reviews.append({'fused_index':c['fused_index'],'member_labels':c['member_label_counts'],'frames':selected,'automatic_semantic_verdict':None})
 for col,name in enumerate(selected):
  with np.load(a.episode/name) as f:
   rgb=f['rgb'];stamp=int(f['time_ns'])
  im=Image.fromarray(rgb).resize((640,280));draw=ImageDraw.Draw(im)
  for row in proposals.get(stamp,[]):
   if row['name'] not in c['member_label_counts']:continue
   box=[float(row[k]) for k in ['x1','y1','x2','y2']]
   box=[box[0]*640/float(row['img_width']),box[1]*280/float(row['img_height']),box[2]*640/float(row['img_width']),box[3]*280/float(row['img_height'])]
   draw.rectangle(box,outline='red',width=2);draw.text((max(0,box[0]),max(0,box[1])),row['name'],fill='red')
  x,y=col*640,ci*330;canvas.paste(im,(x,y+50));d.text((x+5,y+5),f"Cluster {c['fused_index']}: {c['name']} | {c['member_label_counts']}",fill='black');d.text((x+5,y+24),name,fill='black')
a.output.parent.mkdir(parents=True,exist_ok=True);canvas.save(a.output)
a.output.with_suffix('.json').write_text(json.dumps({'review_samples':reviews,'limitations':'Red boxes show same-class proposals in member frames; association to the exact fused instance is not asserted. Human semantic review required.'},indent=2)+'\n')
