import tempfile
import unittest
from pathlib import Path

import numpy as np

from scripts.build_occusg_grid import (
    convex_hull,
    clear_non_boundary_components,
    group_nearby_instances,
    minimum_area_rectangle,
    points_in_removable_boxes,
    semantic_ids_for_labels,
)


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

    def test_stair_rectangle_fills_suspended_underflight_void(self):
        # Visible stair/railing samples form a triangle.  Its enclosing flight
        # footprint must also contain the otherwise unobserved lower-right void.
        points = np.asarray([[0, 0], [0, 4], [6, 4]])
        rectangle = minimum_area_rectangle(points)
        self.assertEqual(set(map(tuple, rectangle)), {(0, 0), (6, 0), (6, 4), (0, 4)})

    def test_stair_and_nearby_railing_form_one_assembly(self):
        instances = [
            (1, np.asarray([[0.0, 0.0], [1.0, 1.0]])),
            (2, np.asarray([[1.2, 1.0], [2.0, 2.0]])),
            (3, np.asarray([[5.0, 5.0], [6.0, 6.0]])),
        ]
        groups = group_nearby_instances(instances, maximum_gap_m=0.5)
        self.assertEqual([sorted(group["semantic_ids"]) for group in groups], [[1, 2], [3]])

    def test_oriented_box_filter_uses_geometry_not_detector_class_id(self):
        row = {
            "tx_world_object": "1", "ty_world_object": "2", "tz_world_object": "0.5",
            "qw_world_object": "1", "qx_world_object": "0", "qy_world_object": "0", "qz_world_object": "0",
            "scale_x": "2", "scale_y": "1", "scale_z": "1",
        }
        points = np.asarray([[1.0, 2.0, 0.5], [2.2, 2.0, 0.5], [1.0, 2.0, 1.2]])
        mask = points_in_removable_boxes(points, [{"row": row}], expansion=1.0)
        self.assertEqual(mask.tolist(), [True, False, False])

    def test_only_unprotected_occupied_island_is_removed(self):
        grid = np.zeros((8, 8), dtype=np.int8)
        grid[0, :] = -1
        grid[1, 1:4] = 100
        grid[5:7, 5] = 100
        protected = np.zeros_like(grid, dtype=bool)
        protected[1, 1] = True
        result, components, cells = clear_non_boundary_components(grid, protected)
        self.assertTrue(np.all(result[1, 1:4] == 100))
        self.assertTrue(np.all(result[5:7, 5] == 0))
        self.assertEqual((components, cells), (1, 2))


if __name__ == "__main__":
    unittest.main()
