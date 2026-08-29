from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from scripts.prepare_multifloor_stage2_scene import flatten_scene


class MultifloorStage2SceneTest(unittest.TestCase):
    def test_ids_and_heights_are_global(self):
        source = {"floors": [], "stairwells": [{"id": "s1", "floor_region_ids": {"1": [1], "2": [1]}}]}
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for floor, height in ((1, 0.0), (2, 3.0)):
                source["floors"].append({
                    "floor_id": floor,
                    "rooms": [{
                        "id": 1, "adjacent_room_ids": [], "objects": [{"id": "boxer_0"}],
                    }],
                })
                (root / f"L{floor}_wall_grid.json").write_text(json.dumps({
                    "floor_z_m": height,
                    "single_floor_policy": {"camera_height_band_m": [height + .5, height + 1.8]},
                }))
            graph = flatten_scene(source, root)
        self.assertEqual([room["id"] for room in graph["rooms"]], ["L1_R1", "L2_R1"])
        self.assertEqual([obj["id"] for obj in graph["objects"]], ["L1_boxer_0", "L2_boxer_0"])
        self.assertEqual(graph["stairwells"][0]["floor_room_ids"], {"1": ["L1_R1"], "2": ["L2_R1"]})


if __name__ == "__main__":
    unittest.main()
