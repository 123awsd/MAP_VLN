import tempfile
import unittest
from pathlib import Path

import numpy as np

from scripts.build_occusg_grid import convex_hull, group_nearby_instances, semantic_ids_for_labels


class Stage1FloorFilterTests(unittest.TestCase):
    def test_hm3d_stair_labels_are_selected_exactly(self):
        with tempfile.TemporaryDirectory() as directory:
            labels = Path(directory) / "scene.semantic.txt"
            labels.write_text(
                '0,FFFFFF,"floor",1\n'
                '11,AAAAAA,"stairs railing",1\n'
                '12,BBBBBB,"stairs",1\n'
                '13,CCCCCC,"chair",1\n',
                encoding="utf-8",
            )
            result = semantic_ids_for_labels(labels, {"stairs", "stairs railing"})
        self.assertEqual(result, {11, 12})

    def test_convex_hull_fills_stair_instance_footprint(self):
        points = np.asarray([[0, 0], [2, 0], [2, 2], [0, 2], [1, 1], [0, 0]])
        self.assertEqual(convex_hull(points).tolist(), [[0, 0], [2, 0], [2, 2], [0, 2]])

    def test_stair_and_nearby_railing_form_one_assembly(self):
        instances = [
            (1, np.asarray([[0.0, 0.0], [1.0, 1.0]])),
            (2, np.asarray([[1.2, 1.0], [2.0, 2.0]])),
            (3, np.asarray([[5.0, 5.0], [6.0, 6.0]])),
        ]
        groups = group_nearby_instances(instances, maximum_gap_m=0.5)
        self.assertEqual([sorted(group["semantic_ids"]) for group in groups], [[1, 2], [3]])


if __name__ == "__main__":
    unittest.main()
