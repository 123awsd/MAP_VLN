import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from saved_minco_artifact import read_trajectory


class SavedMincoTests(unittest.TestCase):
    def parse(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.txt"
            path.write_text(text)
            return read_trajectory(path)

    def test_coefficient_order_and_dwell(self):
        text = "FULL_SMOOTH_TRAJECTORY_V1 START 0 0 0 0 PHASE 3 0 0 1 2 1 YAW 0 0 PIECE 1 3 2 1 0 0 0 0 0 END_PHASE END"
        start, phases = self.parse(text)
        self.assertEqual(phases[0]["points"][-1], [1, 0, 0])
        self.assertEqual(phases[0]["dwell"], 3)
        with self.assertRaises(ValueError):
            self.parse(text.replace("YAW 0 0", "YAW nan 0"))
        with self.assertRaises(ValueError):
            self.parse(text + " garbage")


if __name__ == "__main__":
    unittest.main()
