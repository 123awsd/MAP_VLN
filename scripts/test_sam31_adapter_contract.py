"""Check Boxer's adapter contract without loading or mocking model weights."""
import sys,unittest
from pathlib import Path
import torch,numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'boxer_ext'))
from sam31_wrapper import Sam31Wrapper
class Processor:
 def __init__(self,empty=False):self.empty=empty
 def set_image(self,image):return {'size':image.size}
 def set_text_prompt(self,prompt,state):
  w,h=state['size'];n=0 if self.empty else (2 if prompt=='bed'else 1)
  return {'boxes':torch.tensor([[10.,20.,100.,80.]]*n).reshape(n,4),'scores':torch.tensor([.9,.6][:n]if prompt=='bed'else [.8]*n),'masks':torch.ones((n,1,h,w),dtype=torch.bool)}
def adapter(empty=False):
 obj=Sam31Wrapper.__new__(Sam31Wrapper);obj.device='cpu';obj.precision='float32';obj.prompts=['bed','chair'];obj.processor=Processor(empty);return obj
class Contract(unittest.TestCase):
 def test_nonsquare_coordinates_and_mask_match(self):
  obj=adapter();b,s,i,m=obj.forward(torch.zeros((1,3,120,200)),resize_to_HW=(960,960));self.assertEqual(b.shape,(1,4));self.assertEqual(b.tolist(),[[10.,100.,20.,80.]]);self.assertEqual(i.tolist(),[0]);self.assertEqual(m.shape,(1,120,200));self.assertEqual(m.dtype,np.bool_);self.assertEqual(obj.last_detections[0]['bbox_xyxy'],[10.,20.,100.,80.])
 def test_empty_output(self):
  b,s,i,m=adapter(True).forward(torch.zeros((1,3,120,200)));self.assertEqual(b.shape,(0,4));self.assertEqual(m.shape,(0,120,200));self.assertEqual(i.dtype,torch.long)
 def test_batch_contract(self):
  with self.assertRaises(ValueError):adapter().forward(torch.zeros((2,3,120,200)))
if __name__=='__main__':unittest.main()
