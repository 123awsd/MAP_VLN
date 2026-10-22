#!/usr/bin/env python3
"""Use existing 700 s room labels, recorded RGB, and SAM3.1+Boxer boxes for room semantics.

Preparation is offline. --execute is the only mode that contacts DashScope.
No map, room boundary, detection, or object box is modified.
"""
from __future__ import annotations
import argparse, csv, json, sys
from pathlib import Path
from collections import defaultdict
import numpy as np
import cv2
from scipy.spatial.transform import Rotation

R=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(R/'scripts'))
from classify_room_semantics_qwen import QwenRoomSemanticClassifier, atomic_json


def room_at(pos, labels):
    floor=int(np.digitize(float(pos[2]),[2.3,5.3]))+1
    a=labels[floor]
    ij=np.floor((pos[:2]-a['origin_xy'])/float(a['resolution'])).astype(int)
    if 0<=ij[0]<a['labels'].shape[1] and 0<=ij[1]<a['labels'].shape[0]:
        rid=int(a['labels'][ij[1],ij[0]])
        if rid:return floor,rid
    return None


def make_inputs(output:Path, max_views:int=2):
    output.mkdir(parents=True,exist_ok=True)
    room_base=R/'outputs/coverage700_wall_room_preview/wall_structural_rooms'
    labels={i:np.load(room_base/f'L{i}_constrained.npz') for i in (1,2,3)}
    boxes=json.loads((R/'outputs/boxer/coverage700_sam31_20261018/fusion_fixed/boxer_summary.json').read_text())
    objects=defaultdict(list)
    for obj in boxes['objects']:
        f=int(obj['floor']);center=np.asarray(obj['center_xyz'],float)
        matched=room_at(center,labels)
        if matched is None or matched[0]!=f:continue
        key=f'{f}-{matched[1]:02d}'
        # Use geometry identity from the automatic partition, not an obsolete display label.
        objects[key].append({'id':obj['id'],'label':obj['label'],'probability':obj['probability'],
                             'center_xyz_m':obj['center_xyz'],'size_xyz_m':obj['size_xyz'],
                             'room_assignment':'center_inside_auto_region'})
    frames=defaultdict(list)
    source=R/'outputs/boxer/coverage700_measured_20261017/episode'
    detections=defaultdict(int)
    with (R/'outputs/boxer/coverage700_sam31_20261018/detections/episode/boxer_3dbbs.csv').open() as handle:
        for row in csv.DictReader(handle):
            if float(row['prob'])>=.45:detections[int(row['time_ns'])]+=1
    for p in sorted(source.glob('frame_*.npz')):
        with np.load(p) as a:
            pos=a['position'].astype(float);assignment=room_at(pos,labels)
            if assignment is None:continue
            rgb=a['rgb'];orient=a['orientation_xyzw'].astype(float);time_ns=int(a['time_ns'])
            gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
            sharp=float(cv2.Laplacian(gray,cv2.CV_64F).var())
            brightness=float(gray.mean())
            forward=Rotation.from_quat(orient).as_matrix()[:,2]
            frames[assignment].append({'source':p,'position':pos,'forward':forward,'sharpness':sharp,
                                       'brightness':brightness,'detections':detections[time_ns]})
    graph=[];manifest={};frame_meta={}
    for f,a in labels.items():
        grid=a['labels'];res=float(a['resolution']);org=a['origin_xy']
        for rid in np.unique(grid[grid>0]):
            rid=int(rid);mask=grid==rid;y,x=np.where(mask)
            center=[float(org[0]+(x.mean()+.5)*res),float(org[1]+(y.mean()+.5)*res)]
            key=f'{f}-{rid:02d}';numeric=f*100+rid
            room={'id':numeric,'display_id':key,'floor':f,'centroid_xy_m':center,
                  'area_m2':float(mask.sum()*res**2),'space_role':'room',
                  'adjacent_room_ids':[],'objects':objects[key]}
            graph.append(room)
            candidates=frames.get((f,rid),[])
            if not candidates:continue
            # Choose a sharp, usable first view; the second rewards a distinct camera direction/position.
            usable=[v for v in candidates if 25<v['brightness']<235] or candidates
            detected=[v for v in usable if v['detections']>0]
            if detected:usable=detected
            first=max(usable,key=lambda v:3*min(v['detections'],10)+.004*v['sharpness'])
            chosen=[first]
            if max_views>1 and len(usable)>1:
                remaining=[v for v in usable if v is not first]
                second=max(remaining,key=lambda v:
                    min(np.linalg.norm(v['position']-first['position']),2.0)
                    +1.5*(1-float(np.clip(v['forward']@first['forward'],-1,1)))
                    +1.5*np.log1p(v['detections'])+.04*np.log1p(v['sharpness']))
                chosen.append(second)
            paths=[];records=[]
            for index,v in enumerate(chosen,1):
                with np.load(v['source']) as item:rgb=item['rgb'].copy()
                dest=output/'images'/f'{key}_view{index}_{v["source"].stem}.jpg'
                dest.parent.mkdir(exist_ok=True)
                cv2.imwrite(str(dest),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,92])
                paths.append(str(dest.resolve()))
                records.append({'frame':v['source'].name,'position_xyz_m':v['position'].tolist(),
                                'forward_xyz':v['forward'].tolist(),'sharpness':v['sharpness'],
                                'frame_boxer_detections':v['detections'],
                                'image':str(dest.resolve())})
            manifest[str(numeric)]=paths;frame_meta[key]=records
    atomic_json(output/'room_graph_geometry.json',{'rooms':graph,'source':'700s measured auto partition + fused Boxer, no GT',
        'room_geometry_changed':False,'boxer_rerun':False})
    atomic_json(output/'room_image_manifest.json',manifest)
    atomic_json(output/'selected_frames.json',frame_meta)
    return graph,manifest,frame_meta


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',type=Path,default=R/'outputs/stage1_pipeline_flow_20261019/automated_room_semantics')
    ap.add_argument('--room',action='append',help='Display ID such as 2-05; repeat to test more rooms. Default prepares all, runs none.')
    ap.add_argument('--all',action='store_true',help='Classify every room with measured RGB; use with --execute')
    ap.add_argument('--max-rooms-per-run',type=int,default=30,
                    help='Maximum rooms in one run; default covers all 30 automatic room regions')
    ap.add_argument('--execute',action='store_true',help='Send selected measured RGB and Boxer metadata to DashScope Qwen-VL')
    ap.add_argument('--confidence-threshold',type=float,default=.65)
    args=ap.parse_args()
    graph,manifest,meta=make_inputs(args.output)
    if args.all and args.room:raise ValueError('use --all or --room, not both')
    chosen=[r for r in graph if args.all or (args.room and r['display_id'] in args.room)]
    if args.room and len(chosen)!=len(set(args.room)):
        raise ValueError('some requested room IDs do not exist')
    summary={'prepared_rooms':len(graph),'rooms_with_images':len(manifest),
             'rooms_with_objects':sum(bool(r['objects']) for r in graph),'selected_for_execution':[r['display_id'] for r in chosen],
             'execution':'not_requested'}
    if args.execute:
        if not chosen:raise ValueError('--execute requires --all or at least one --room')
        if len(chosen)>args.max_rooms_per_run:
            raise ValueError(f'{len(chosen)} rooms exceed --max-rooms-per-run={args.max_rooms_per_run}')
        missing=[r['display_id'] for r in chosen if str(r['id']) not in manifest]
        if missing:raise ValueError(f'no measured RGB for rooms: {missing}')
        classifier=QwenRoomSemanticClassifier(20.0,args.confidence_threshold)
        decisions,provenance=classifier.classify(chosen,{str(r['id']):manifest[str(r['id'])] for r in chosen})
        atomic_json(args.output/'semantic_results.json',{'decisions':decisions,'provenance':provenance,
            'room_image_manifest':'room_image_manifest.json','room_graph':'room_graph_geometry.json'})
        summary['execution']='completed';summary['results']=decisions
    atomic_json(args.output/'status.json',summary)
    print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
