"""Validate full measured-frame coverage and compare output counts, not GT accuracy."""
from pathlib import Path
import json,csv,os,argparse
from collections import Counter
import numpy as np
R=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--run-dir',type=Path,default=R/'outputs/boxer/coverage700_sam31_20261018');args=parser.parse_args()
O=args.run_dir;m=json.loads((O/'manifest.json').read_text());assert m['status']=='completed'and m['frames_count']==350 and len(m['frames'])==350
assert len({v['time_ns']for v in m['frames']})==350
count=0
for row in m['frames']:
 with np.load(O/f"frame_{row['sequence']:06d}_sam.npz")as a:
  assert a['masks'].shape==(row['detections'],480,640);assert a['boxes_xyxy'].shape==(row['detections'],4);assert int(a['time_ns'])==row['time_ns'];assert np.isfinite(a['boxes_xyxy']).all();assert np.all(a['scores']>.5);count+=len(a['scores'])
D=O/'detections/episode';D.mkdir(parents=True,exist_ok=True)
for source,target in [('boxer_3dbbs.csv','boxer_3dbbs.csv'),('sam_2dbbs.csv','owl_2dbbs.csv')]:
 path=D/target
 if not path.exists():path.symlink_to(Path('../..')/source)
raw=list(csv.DictReader((O/'boxer_3dbbs.csv').open()));two=list(csv.DictReader((O/'sam_2dbbs.csv').open()));assert len(two)==count and len(raw)==sum(v['boxer_3d_detections']for v in m['frames'])
assert all(int(v['time_ns'])in{f['time_ns']for f in m['frames']}for v in raw)
stats={'frames':350,'source_episode':m['source_episode'],'all_timestamps_unique':True,'all_mask_shapes_valid':True,'strict_detector_load':m['detector_weights_strict_load'],'detections_2d':count,'detections_3d':len(raw),'peak_cuda_allocated_gb':m['peak_cuda_allocated_gb'],'detector_seconds_total':m['detector_seconds_total'],'detector_seconds_per_frame':m['detector_seconds_total']/350,'zero_detection_frames':sum(v['detections']==0 for v in m['frames']),'video_tracking_tested':False,'gt_accuracy_measured':False}
(O/'checks.json').write_text(json.dumps(stats,indent=2));print(stats)
