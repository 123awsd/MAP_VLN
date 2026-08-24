from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from baselines.opennav_bridge import OpenNavObservationBridge


class FakeDetector:
    def __init__(self):
        self.closed = False

    def detect(self, path, prompts=None):
        self.path = Path(path)
        self.prompts = prompts
        return {
            "detections": [
                {"label": "chair", "score": 0.91, "bbox_xyxy": [5, 10, 25, 40]},
                {"label": "lamp", "score": 0.72, "bbox_xyxy": [70, 5, 95, 45]},
            ]
        }

    def close(self):
        self.closed = True


class OpenNavBridgeTest(unittest.TestCase):
    def test_observation_is_open_nav_compatible(self):
        detector = FakeDetector()
        with tempfile.TemporaryDirectory() as directory:
            bridge = OpenNavObservationBridge(
                ["chair", "lamp"], detector=detector, runtime_dir=Path(directory)
            )
            observation = bridge.observe_view(
                None,
                3,
                "7",
                {"rgb": Image.fromarray(np.zeros((50, 100, 3), dtype=np.uint8))},
            )
            self.assertIn("Direction Viewpoint ID: 7", observation)
            self.assertIn("chair at image left", observation)
            self.assertIn("lamp at image right", observation)
            self.assertTrue(detector.path.is_file())
            bridge.close()
            self.assertTrue(detector.closed)


if __name__ == "__main__":
    unittest.main()
