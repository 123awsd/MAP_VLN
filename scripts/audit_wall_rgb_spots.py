"""Small visual spot audit, not a full labeled benchmark.

Pixels were selected by visually inspecting original RGBs, independently of
SegFormer predictions. These checks concern observed walls/furniture, not hidden
walls or image-space area accuracy. Coordinates are original 640x480 pixels.
"""
from pathlib import Path
import json
import numpy as np
from scipy.spatial import cKDTree
from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/wall_refinement'
SRC=ROOT/'outputs/semantic_structure_trial/rgbd'
# (floor, frame, class, pixels). Roofs and ambiguous wooden doors are excluded.
SPOTS=[
 (1,'0003','wall',[(100,100),(280,180),(400,300)]),
 (1,'0006','wall',[(80,140),(180,250),(260,120)]),
 (1,'0009','wall',[(300,60),(480,100),(600,160)]),
 (1,'0000','nonwall',[(115,280),(290,200),(380,320)]),
 (1,'0015','nonwall',[(480,230),(470,330),(90,300)]),
 (2,'0022','wall',[(210,120),(280,100),(580,120)]),
 (2,'0025','wall',[(120,200),(200,300),(160,80)]),
 (2,'0046','wall',[(100,200),(280,250),(460,150)]),
 (2,'0022','nonwall',[(420,420),(540,410),(280,350)]),
 (2,'0034','nonwall',[(180,260),(330,300),(470,300)]),
 (3,'0080','wall',[(120,160),(280,280),(380,370)]),
 (3,'0098','wall',[(90,110),(100,260),(140,360)]),
 (3,'0099','wall',[(80,435),(210,410),(580,300)]),
 (3,'0096','nonwall',[(190,350),(430,250),(310,260)]),
 (3,'0101','nonwall',[(200,410),(320,400),(420,420)]),
]

def main():
    geo=np.load(ROOT/'outputs/room_quality_ceiling/geometry.npz')
    methods={'baseline':[ROOT/'outputs/wall_patch_context'/f'L{f+1}_局部高度宽松.npz'for f in range(3)]}
    for rnd in range(1,5):
        dest=OUT/f'round{rnd}'
        names=sorted({p.stem.split('_',1)[1]for p in dest.glob('L1_*.npz')})
        for name in names: methods[f'round{rnd}/{name}']=[dest/f'L{f+1}_{name}.npz'for f in range(3)]
    queries=[[]for _ in range(3)]; truth=[[]for _ in range(3)];spots=[]
    for f,frame,category,pixels in SPOTS:
        d=np.load(SRC/f'{frame}.npz');im=Image.open(SRC/f'{frame}.png');draw=ImageDraw.Draw(im)
        for u,v in pixels:
            depth=float(d['depth'][v,u]);point=np.array([(u-320)*depth/320,(v-240)*depth/320,depth])@d['rotation'].T+d['position']
            valid=.1<depth<4.95
            item=dict(floor=f,frame=frame,category=category,pixel=[u,v],depth=depth,valid=valid,xyz=point.tolist())
            spots.append(item)
            if valid:queries[f-1].append(point);truth[f-1].append(category=='wall')
            color=(180,20,20)if category=='wall'else(0,90,200)
            draw.ellipse((u-7,v-7,u+7,v+7),outline=color,width=3)
        im.save(OUT/f'audit_L{f}_{frame}_{category}.png')
    results=[]
    for method,files in methods.items():
        floors=[]
        for f in range(3):
            q=np.asarray(queries[f]);labels=np.asarray(truth[f]);p=geo[f'p{f}']
            measured,_=cKDTree(p).query(q)
            # Query must have actual depth evidence represented in this cloud.
            usable=measured<.1
            walls=np.load(files[f])['wall_points'];dist,_=cKDTree(walls).query(q);hit=dist<.1
            floors.append(dict(floor=f+1,wall_found=int((hit&labels&usable).sum()),wall_samples=int((labels&usable).sum()),nonwall_hits=int((hit&~labels&usable).sum()),nonwall_samples=int((~labels&usable).sum()),unrepresented=int((~usable).sum()),distances=dist.tolist()))
        results.append(dict(method=method,floors=floors,wall_found=sum(x['wall_found']for x in floors),wall_samples=sum(x['wall_samples']for x in floors),nonwall_hits=sum(x['nonwall_hits']for x in floors),nonwall_samples=sum(x['nonwall_samples']for x in floors)))
    report=dict(note='45 visually chosen RGB pixels; small spot check, no whole-scene accuracy claim; 10cm measured-point proximity.',spots=spots,results=results)
    (OUT/'rgb_spot_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    for r in results:print(r['method'],r['wall_found'],'/',r['wall_samples'],'nonwall',r['nonwall_hits'],'/',r['nonwall_samples'])

if __name__=='__main__':main()
