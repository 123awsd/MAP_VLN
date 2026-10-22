#!/usr/bin/env python3
"""Render frozen same-frame detector reviews; no semantic accuracy estimates."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--episode',type=Path,required=True)
p.add_argument('--cases',type=Path,required=True)
p.add_argument('--baseline',type=Path,required=True)
p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args();cases=json.loads(a.cases.read_text())['cases']
def rows(path):
 out={}
 with path.open()as stream:
  for r in csv.DictReader(stream):out.setdefault(int(r['time_ns']),[]).append(r)
 return out
models=[('OWLv2',rows(a.baseline)),('Strict GroundingDINO',rows(a.candidate))]
canvas=Image.new('RGB',(1280,330*len(cases)),'white');draw=ImageDraw.Draw(canvas);review=[]
for index,case in enumerate(cases):
 with np.load(a.episode/case['frame'])as f:rgb=f['rgb'];stamp=int(f['time_ns'])
 record={'frame':case['frame'],'issue':case['issue'],'models':{}}
 for col,(name,proposals) in enumerate(models):
  image=Image.fromarray(rgb).resize((640,280));d=ImageDraw.Draw(image)
  selected=[r for r in proposals.get(stamp,[])if r['name']in case['review_labels']]
  record['models'][name]=[{'name':r['name'],'prob':float(r['prob']),'box_xyxy':[float(r[k])for k in ['x1','y1','x2','y2']]}for r in selected]
  for r in selected:
   box=[float(r[k])for k in ['x1','y1','x2','y2']];box=[box[0]*640/float(r['img_width']),box[1]*280/float(r['img_height']),box[2]*640/float(r['img_width']),box[3]*280/float(r['img_height'])]
   d.rectangle(box,outline='red',width=2);d.text((max(0,box[0]),max(0,box[1])),f"{r['name']} {float(r['prob']):.2f}",fill='red')
  x,y=col*640,index*330;canvas.paste(image,(x,y+50));draw.text((x+5,y+5),f"{name} | {case['frame']}",fill='black');draw.text((x+5,y+24),case['issue'],fill='black')
 review.append(record)
a.output.parent.mkdir(parents=True,exist_ok=True);canvas.save(a.output);a.output.with_suffix('.json').write_text(json.dumps({'reviews':review,'semantic_accuracy_proven':False,'limitations':'Only frozen problem cases and specified labels are displayed; not a complete recall evaluation or exact object association.'},indent=2)+'\n')
