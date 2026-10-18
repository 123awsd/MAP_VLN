"""Regression tests for semantic and independent-capture fusion boundaries."""
from pathlib import Path
import sys,unittest,json
import torch
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'third_party/boxer'))
from utils.fuse_3d_boxes import BoundingBox3DFuser,apply_nms_to_fused_instances
from utils.tw.obb import make_obb
from utils.tw.tensor_utils import string2tensor,pad_string

def box(label='bed',x=0.,prob=.9):
 v=make_obb(sz=[1.,1.,1.],position=[x,0.,.5],prob=prob,yaw=0.);v.set_text(string2tensor(pad_string(label,max_len=128)));return v

def fuse(obs,frames,minimum=4):
 return BoundingBox3DFuser(iou_threshold=.2,min_detections=minimum,conf_threshold=.45).fuse(torch.stack(obs),frame_ids=torch.tensor(frames))

class Boundaries(unittest.TestCase):
 def test_incompatible_classes_do_not_merge(self):
  v=fuse([box('bed')]*4+[box('desk')]*4,[1,2,3,4]*2);self.assertEqual(len(v),2);self.assertEqual(sorted(s.support_count for s in v),[4,4])
 def test_same_frame_is_not_four_observations(self):
  self.assertEqual(fuse([box()]*8,[1]*8),[])
 def test_three_frames_with_duplicates_rejected(self):
  self.assertEqual(fuse([box()]*6,[1,1,2,2,3,3]),[])
 def test_duplicate_frame_does_not_bias_geometry(self):
  v=fuse([box()]*4+[box(x=.2,prob=.5)]*6,[1,2,3,4]+[1]*6)[0]
  self.assertEqual(v.support_count,4);self.assertEqual(v.raw_detection_count,10);self.assertAlmostEqual(float(v.obb.T_world_object.t[0]),0.,places=5)
 def test_confidence_filter_keeps_frame_alignment(self):
  self.assertEqual(fuse([box(prob=.3),box(),box(),box()],[1,2,3,4]),[])
 def test_missing_capture_identity_fails_explicitly(self):
  with self.assertRaises(ValueError):BoundingBox3DFuser(min_detections=4).fuse(torch.stack([box()]*4))
 def test_frame_id_length_validation(self):
  with self.assertRaises(ValueError):fuse([box()]*4,[1,2])
 def test_sparse_semantic_boundary(self):
  m=torch.ones((2,2)).to_sparse();f=BoundingBox3DFuser(min_detections=1)
  self.assertEqual(sorted(f._cluster_detections(m,labels=['bed','desk'])),[[0],[1]])
 def test_nms_does_not_remove_other_class(self):
  v=fuse([box('bed')]*4+[box('desk')]*4,[1,2,3,4]*2);self.assertEqual(len(apply_nms_to_fused_instances(v)),2)
 def test_single_detection_confidence_mask(self):
  self.assertEqual(len(fuse([box()],[1],minimum=1)),1)

if __name__=='__main__':
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Boundaries))
 out=R/'outputs/boxer/coverage700_measured_20261017/fusion_fixed';out.mkdir(exist_ok=True)
 (out/'regression.json').write_text(json.dumps({'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),'passed':result.wasSuccessful()},indent=2))
 sys.exit(not result.wasSuccessful())
