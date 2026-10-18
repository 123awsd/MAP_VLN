"""Run existing fusion and preserve per-instance detection support evidence."""
from pathlib import Path
import json,sys,argparse
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'third_party/boxer'))
from utils.fuse_3d_boxes import fuse_obbs_from_csv
parser=argparse.ArgumentParser();parser.add_argument('--run-dir',type=Path,default=R/'outputs/boxer/coverage700_measured_20261017');args=parser.parse_args()
D=args.run_dir/'detections/episode'
instances=fuse_obbs_from_csv(str(D/'boxer_3dbbs.csv'),str(D/'boxer_3dbbs_fused_fixed.csv'),iou_threshold=.2,min_detections=4,conf_threshold=.45)
(D/'fusion_support_fixed.json').write_text(json.dumps([{'fused_row':i,'support_count':int(v.support_count),'raw_detection_count':int(v.raw_detection_count),'filtered_detection_indices':v.detection_indices}for i,v in enumerate(instances)],indent=2))
