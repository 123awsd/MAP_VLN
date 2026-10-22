#!/usr/bin/env python3
"""Use existing Boxer fusion, preserving camera diversity provenance.

Prediction-to-prediction IoU is only a clustering rule, not ground-truth IoU.
Pose-bin support is a geometric diversity proxy, not an accuracy estimate.
"""
import argparse
import csv
from collections import Counter
import json
import random
import sys
from pathlib import Path
import numpy as np
import torch

root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'third_party/boxer'))
from utils.file_io import read_obb_csv
from utils.fuse_3d_boxes import fuse_obbs_from_csv


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True,type=Path)
    p.add_argument('--episode',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--confidence',type=float,default=.45)
    p.add_argument('--separate-same-frame-proposals',type=Path,help='Optional experimental cannot-link constraints from exactly corresponding 2D rows')
    p.add_argument('--preserve-under-supported-splits',action='store_true',help='Keep an original ambiguous cluster if a constrained child lacks the unchanged minimum support')
    a=p.parse_args()
    if a.preserve_under_supported_splits and not a.separate_same_frame_proposals:p.error('Preservation requires proposal constraints')
    random.seed(7);np.random.seed(7);torch.manual_seed(7)
    frames={}
    for path in a.episode.glob('frame_*.npz'):
        with np.load(path)as f:
            xyz=f['position'];x,y,z,w=f['orientation_xyzw']
            forward=np.array([2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)])
            heading=np.arctan2(forward[1],forward[0])
            position_bin=tuple(np.floor(xyz/.25).astype(int))
            frames[int(f['time_ns'])]={'frame':path.name,'position_bin':position_bin,
                                      'pose_bin':position_bin+(int(np.floor((heading+np.pi)/np.deg2rad(15))),)}
    timed=read_obb_csv(str(a.input));filtered_stamps=[];filtered_labels=[]
    for stamp,obbs in timed.items():
        probs=obbs.prob.detach().cpu().numpy().reshape(-1)
        filtered_stamps.extend([int(stamp)for value in probs if value>=a.confidence])
        filtered_labels.extend([name for name,value in zip(obbs.text_string(),probs) if value>=a.confidence])
    with a.input.open() as stream:
        csv_rows=list(csv.DictReader(stream))
    filtered_row_indices=[i for i,row in enumerate(csv_rows) if float(row['prob'])>=a.confidence]
    if [(int(csv_rows[i]['time_ns']),csv_rows[i]['name']) for i in filtered_row_indices] != list(zip(filtered_stamps,filtered_labels)):
        raise RuntimeError('CSV provenance order does not match fuser input')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    constraint_audit=None
    if a.separate_same_frame_proposals:
        sys.path.insert(0,str(root/'boxer_ext'))
        from fusion_frame_constraints import proposal_cannot_links,constrained_components
        from utils.fuse_3d_boxes import BoundingBox3DFuser
        with a.separate_same_frame_proposals.open() as stream:proposals=list(csv.DictReader(stream))
        if len(proposals)!=len(csv_rows) or any((p['time_ns'],p['name'])!=(r['time_ns'],r['name']) for p,r in zip(proposals,csv_rows)):
            raise RuntimeError('Exact 2D/3D association required for experimental constraints')
        blocked=proposal_cannot_links([proposals[i] for i in filtered_row_indices])
        constraint_audit={'same_frame_proposal_iou_cutoff':.1,'cannot_link_pairs':len(blocked),'rejected_graph_merges':0}
        original_cluster=BoundingBox3DFuser._cluster_detections
        def experimental_cluster(self,matrix,semantic_matrix=None):
            if semantic_matrix is not None:raise RuntimeError('Semantic matrix unsupported in this ablation')
            clusters,rejected=constrained_components(matrix,self.iou_threshold,blocked)
            constraint_audit['rejected_graph_merges']=rejected
            if a.preserve_under_supported_splits:
                original=original_cluster(self,matrix,semantic_matrix)
                owner={i:j for j,group in enumerate(original) for i in group}
                revert={owner[group[0]] for group in clusters if len(group)<self.min_detections and len(original[owner[group[0]]])>=self.min_detections}
                clusters=[group for group in clusters if owner[group[0]] not in revert]+[original[j] for j in sorted(revert)]
                clusters.sort(key=lambda group:min(group))
                constraint_audit['original_clusters_preserved_for_insufficient_child_support']=len(revert)
                constraint_audit['preserved_ambiguous_source_rows']=[original[j] for j in sorted(revert)]
            return clusters
        BoundingBox3DFuser._cluster_detections=experimental_cluster
        try:
            fused=fuse_obbs_from_csv(str(a.input),str(a.output),iou_threshold=.2,min_detections=4,conf_threshold=a.confidence)
        finally:
            BoundingBox3DFuser._cluster_detections=original_cluster
    else:
        fused=fuse_obbs_from_csv(str(a.input),str(a.output),iou_threshold=.2,min_detections=4,conf_threshold=a.confidence)
    instances=[]
    for index,instance in enumerate(fused):
        stamps=[filtered_stamps[i]for i in instance.detection_indices]
        label_counts=Counter(filtered_labels[i] for i in instance.detection_indices)
        missing=[s for s in stamps if s not in frames]
        if missing:raise RuntimeError('Fused support lacks exact acquisition frame')
        position_bins={frames[s]['position_bin']for s in stamps}
        pose_bins={frames[s]['pose_bin']for s in stamps}
        instances.append({'fused_index':index,'name':instance.obb.text_string(),
                          'raw_detection_support':instance.support_count,'distinct_frame_stamps':len(set(stamps)),
                          'camera_position_bins_025m':len(position_bins),
                          'camera_pose_bins_025m_15deg':len(pose_bins),
                          'support_frames':sorted({frames[s]['frame']for s in stamps}),
                          'source_csv_row_indices':[filtered_row_indices[i] for i in instance.detection_indices],
                          'frames_with_multiple_cluster_members':sum(n>1 for n in Counter(stamps).values()),
                          'member_label_counts':dict(sorted(label_counts.items())),
                          'mixed_class_review_required':len(label_counts)>1,
                          'selected_label_member_fraction':label_counts[instance.obb.text_string()]/len(stamps),
                          'repeat_view_review_required':len(pose_bins)<2})
    report={'input':str(a.input),'episode':str(a.episode),'output':str(a.output),
            'fusion_seed':7,'prediction_overlap_threshold':.2,'minimum_detection_count':4,
            'confidence_threshold':a.confidence,'instances':instances,
            'experimental_association_constraints':constraint_audit,
            'full_detection_quality_proven':False,
            'limitations':'Original weighted box averaging and support thresholds unchanged; optional association changes are recorded separately. Repeated frames may inflate detection count. Geometric pose bins do not certify independent measurements, class correctness or object completeness.'}
    a.output.with_suffix('.view_support.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Fused instances:',len(instances),'repeat-view review:',sum(i['repeat_view_review_required']for i in instances))


if __name__=='__main__':main()
