#!/usr/bin/env python3
"""Run the existing Boxer CLI with fixed sampling seeds; forward all arguments."""
import random
import runpy
import sys
import os
import json
from pathlib import Path
import numpy as np
import torch

root = Path(__file__).resolve().parents[1]
random.seed(7)
np.random.seed(7)
torch.manual_seed(7)
if torch.cuda.is_available():
    torch.cuda.manual_seed_all(7)
sys.path.insert(0, str(root/'boxer_ext'))
sys.path.insert(0, str(root/'third_party/boxer'))
audit_path = os.environ.get('PRE_MAP_VLN_DETECTOR_AUDIT_PATH')
if audit_path:
    from owl.grounding_dino_wrapper import GroundingDinoWrapper
    original_forward = GroundingDinoWrapper.forward
    target = Path(audit_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise RuntimeError('Detector audit already exists; use a fresh trial path')
    def audited_forward(self, *args, **kwargs):
        result = original_forward(self, *args, **kwargs)
        indices = result[2].detach().cpu().tolist()
        rows = [{'raw_label': detection['label'],
                 'mapped_label': self.prompts[index],
                 'exact_prompt_match': detection['label'] in self.prompts,
                 'score': detection['score'], 'bbox_xyxy': detection['bbox_xyxy']}
                for detection, index in zip(self.last_detections, indices)]
        with target.open('a') as stream:
            stream.write(json.dumps({'detections': rows,
                                     'unmapped_detections': getattr(self, 'last_unmapped_detections', [])})+'\n')
        return result
    GroundingDinoWrapper.forward = audited_forward
sys.argv[0] = str(root/'third_party/boxer/run_boxer.py')
runpy.run_path(sys.argv[0], run_name='__main__')
