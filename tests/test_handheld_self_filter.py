import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
STAGE1 = ROOT / "real_fly/stage1_exploration"


def launch_param(path: Path, name: str) -> str:
    root = ET.parse(path).getroot()
    values = [node.attrib["value"] for node in root.findall("param") if node.attrib.get("name") == name]
    if len(values) != 1:
        raise AssertionError(f"expected one {name} parameter in {path}, got {values}")
    return values[0]


class HandheldSelfFilterSafetyTest(unittest.TestCase):
    def test_shared_default_is_disabled(self):
        config = yaml.safe_load(
            (STAGE1 / "config/fast_lio_mid360_handheld.yaml").read_text(encoding="utf-8")
        )
        profile = config["preprocess"]["handheld_self_filter"]
        self.assertFalse(profile["enabled"])
        self.assertEqual(profile["min_range_m"], config["preprocess"]["blind"])
        self.assertEqual(profile["max_range_m"], 1.5)
        self.assertEqual(profile["rear_azimuth_deg"], 0.0)
        self.assertEqual(profile["rear_half_angle_deg"], 90.0)

    def test_only_handheld_launch_enables_filter(self):
        parameter = "preprocess/handheld_self_filter/enabled"
        launch = STAGE1 / "ros_ws/src/stage1_fast_lio/launch"
        self.assertEqual(launch_param(launch / "live_mapping.launch", parameter), "true")
        self.assertEqual(launch_param(launch / "localization_only.launch", parameter), "false")

    def test_offline_replay_matches_handheld_mapping(self):
        replay = (STAGE1 / "scripts/run_mapping_from_bag.sh").read_text(encoding="utf-8")
        self.assertIn("rosparam set preprocess/handheld_self_filter/enabled true", replay)

    def test_handheld_start_refuses_unknown_existing_mapper(self):
        start = (STAGE1 / "scripts/start_handheld_mapping_rviz.sh").read_text(encoding="utf-8")
        self.assertIn("Refusing to reuse /laserMapping", start)


if __name__ == "__main__":
    unittest.main()
