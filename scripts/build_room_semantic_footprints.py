#!/usr/bin/env python3
"""Persist filled semantic room footprints without changing navigable room masks.

Narrow wall/seam pixels can belong to a room's displayed semantic footprint,
but remain wall barriers in the source segmentation and planning geometry.
"""
from pathlib import Path
import json
import numpy as np
from scipy.ndimage import binary_closing

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'outputs/coverage700_wall_room_preview/wall_structural_rooms'
OUTPUT=ROOT/'outputs/stage1_pipeline_flow_20261019/automated_room_semantics/semantic_footprints'
OUTPUT.mkdir(parents=True,exist_ok=True)
audit={'source':str(SOURCE),'output':str(OUTPUT),'purpose':'semantic footprint only; not free space',
       'max_closed_gap_m':0.20,'floors':{}}
for floor in (1,2,3):
    with np.load(SOURCE/f'L{floor}_constrained.npz') as data:
        labels=data['labels'].copy()
        wall_barrier=data['wall_barrier'].copy()
        measured_wall=data['measured_wall'].copy()
        origin=data['origin_xy'].copy()
        resolution=float(data['resolution'])
    width=int(round(.20/resolution))+1
    if width%2==0:width+=1
    proposed=np.zeros_like(labels)
    claims=np.zeros(labels.shape,dtype=np.uint8)
    for room_id in np.unique(labels[labels>0]):
        claimed=binary_closing(labels==room_id,structure=np.ones((width,width),bool)) & (labels==0)
        proposed[claimed]=room_id
        claims[claimed]+=1
    filled=(claims==1)
    semantic=labels.copy()
    semantic[filled]=proposed[filled]
    assert np.array_equal(semantic[labels>0],labels[labels>0])
    assert set(np.unique(semantic))==set(np.unique(labels))
    assert not np.any(filled & (labels!=0))
    assert not np.any(filled & (claims>1))
    dest=OUTPUT/f'L{floor}_semantic_footprint.npz'
    np.savez_compressed(dest,semantic_footprint_labels=semantic,
                        original_room_labels=labels,filled_seam_cells=filled,
                        wall_barrier=wall_barrier,measured_wall=measured_wall,
                        origin_xy=origin,resolution=resolution)
    audit['floors'][str(floor)]={
        'source_room_count':int(np.unique(labels[labels>0]).size),
        'semantic_room_count':int(np.unique(semantic[semantic>0]).size),
        'filled_cells':int(filled.sum()),
        'filled_area_m2':round(float(filled.sum()*resolution*resolution),4),
        'of_which_wall_barrier':int((filled & wall_barrier).sum()),
        'of_which_measured_wall':int((filled & measured_wall.astype(bool)).sum()),
        'ambiguous_shared_boundary_cells_left_unfilled':int((claims>1).sum()),
        'original_positive_labels_unchanged':True,
        'source_wall_barrier_unchanged':True,
        'kernel_cells':width,
    }
(OUTPUT/'audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(OUTPUT/'audit.json')
