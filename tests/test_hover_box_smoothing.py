import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('smooth',Path(__file__).resolve().parents[1]/'scripts/smooth_hover_detection_video.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SmoothingTest(unittest.TestCase):
    def test_only_short_interior_gaps(self):
        boxes=np.full((20,4),np.nan)
        boxes[2]=[10,20,30,40]
        boxes[5]=[12,22,32,42]
        boxes[18]=[14,24,34,44]
        result=module.smooth_boxes(boxes,max_gap=3)
        self.assertTrue(np.isfinite(result[2:6]).all())
        self.assertTrue(np.isnan(result[:2]).all())
        self.assertTrue(np.isnan(result[6:18]).all())
        self.assertTrue(np.isnan(result[19:]).all())

    def test_reduces_jitter_without_invalid_boxes(self):
        raw=np.array([[10+i%2*4,20,40+i%2*4,60] for i in range(30)],float)
        out=module.smooth_boxes(raw)
        self.assertLess(np.abs(np.diff(out,axis=0)).mean(),np.abs(np.diff(raw,axis=0)).mean())
        self.assertTrue((out[:,2:] > out[:,:2]).all())

    def test_lock_size_removes_breathing(self):
        raw=np.array([[10,20,40+i,60+2*i] for i in range(20)],float)
        out=module.smooth_boxes(raw,lock_size=True)
        sizes=out[:,2:]-out[:,:2]
        np.testing.assert_allclose(np.std(sizes,axis=0),[0,0],atol=1e-10)
        np.testing.assert_allclose(sizes[0],np.median(raw[:,2:]-raw[:,:2],axis=0))


if __name__=='__main__':
    unittest.main()
