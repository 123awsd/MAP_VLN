#!/usr/bin/env python3
"""Build an evidence-indexed joint pipeline report without claiming completion."""
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
root=Path(__file__).resolve().parents[1]
report={
 'scene':'HM3D 00337 CFVBbU9Rsyb, floors 1-3 only',
 'selected_exploration_baseline': 'outputs/exploration_oct12_restored_trial',
 'candidate_exploration': 'outputs/exploration_shortpath_optimize_full_trial',
 'selected_status':'Oct12 restored algorithm remains selected; short-path optimization is an unselected coverage-priority candidate',
 'requirements':{
  'synchronized_rgb_depth_pose_odom':{'status':'verified for baseline and candidate','evidence':['outputs/exploration_oct12_restored_trial/stream_audit.json','outputs/exploration_shortpath_optimize_full_trial/stream_audit.json']},
  'normal_realtime_planner_yaw_height_cap':{'status':'verified scalar checks; local sloped-roof containment incomplete','evidence':['outputs/exploration_oct12_restored_trial/stream_audit.json','outputs/exploration_shortpath_optimize_full_trial/measured_sloped_roof_audit.json']},
  'exploration_completion':{'status':'not verified; both selected baseline and candidate timeout','evidence':['outputs/exploration_oct12_restored_trial/stream_audit.json','outputs/exploration_shortpath_optimize_full_trial/stream_audit.json']},
  'depth_wall_extraction':{'status':'fresh measured walls and provenance verified; semantic completeness not verified','evidence':['outputs/exploration_oct12_restored_trial/new_wall_extraction_manifest.json','outputs/exploration_shortpath_optimize_full_trial/new_wall_extraction_manifest.json','outputs/exploration_shortpath_optimize_full_trial/independent_wall_geometry_audit.json']},
  'room_segmentation':{'status':'independent topology and candidate comparison available; true room correctness not verified','evidence':['outputs/exploration_oct12_restored_trial/room_region_preservation_trial/metrics.json','outputs/exploration_shortpath_optimize_full_trial/room_joint_audit/metrics.json','outputs/exploration_shortpath_optimize_full_trial/room_joint_audit/independent_topology_audit.json']},
  'boxer_geometry_and_semantics':{'status':'coordinate/depth support and association diagnostics verified; semantic/3D accuracy not verified','evidence':['outputs/boxer/oct12_owl_motion240_20261014/episode/depth_support_full_range_audit.json','outputs/boxer/oct12_owl_motion240_20261014/frame_constraints_multiview_support.json','outputs/boxer/oct12_gdino_strict_motion240_20261014/comparison_findings.json']},
  'visuals_and_reproducibility':{'status':'diagnostic visuals and manifests exist; final three-view video not produced','evidence':['outputs/exploration_oct12_restored_trial/new_geometry_diagnostic.png','outputs/exploration_oct12_restored_trial/room_region_preservation_trial/measured_room_review_white.png','outputs/boxer/oct12_owl_motion240_20261014/frame_constraints_multiview_review.png']}
 },
 'remaining_required_work':['resolve nonempty-frontier short-path/trajectory-timeout loop without degrading collision safety','validate wall/room semantics with independent human or reliable ground truth review','complete multi-view Boxer semantic and 3D accuracy review','produce final three-view video from selected complete run'],
 'full_pipeline_complete':False,
 'scene_truth_used_as_online_planner_input':False
}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print('Wrote',a.output)
