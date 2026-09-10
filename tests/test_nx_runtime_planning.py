import importlib.util
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "real_fly/stage2_runtime/scripts/capture_planning_start.py"


def load_capture_module():
    spec = importlib.util.spec_from_file_location("capture_planning_start", CAPTURE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class NxRuntimePlanningTest(unittest.TestCase):
    def test_px4ctrl_relative_height_and_yaw(self):
        module = load_capture_module()
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "px4ctrl.yaml"
            config.write_text(
                "auto_takeoff_land:\n  takeoff_height: 0.8\n",
                encoding="utf-8",
            )
            self.assertAlmostEqual(module.load_takeoff_height(config), 0.8)
        half = math.sqrt(0.5)
        quaternion = SimpleNamespace(x=0.0, y=0.0, z=half, w=half)
        self.assertAlmostEqual(module.yaw_from_quaternion(quaternion), math.pi / 2.0)
if __name__ == "__main__":
    unittest.main()
