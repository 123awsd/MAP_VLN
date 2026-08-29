import tempfile
import unittest
from pathlib import Path

import numpy as np

from scripts.build_occusg_grid import (
    complete_removed_object_footprint,
    convex_hull,
    clear_non_boundary_components,
    group_nearby_instances,
    minimum_area_rectangle,
    points_in_removable_boxes,
    semantic_ids_for_labels,
)
from scripts.extract_multifloor_wall_grid import low_connected_top


class RemovedObjectFootprintTests(unittest.TestCase):
    def test_unknown_inside_footprint_is_completed(self):
        grid = np.full((11, 11), -1, dtype=np.int8)
        grid[5, 5] = 100
        footprint = np.zeros((11, 11), dtype=bool)
        footprint[2:9, 2:9] = True
        seeds = np.zeros_like(footprint)
        exclusion = np.zeros_like(footprint)

        completed, filled_unknown, cleared_occupied = complete_removed_object_footprint(
            grid, footprint, seeds, exclusion
        )

        self.assertGreater(int(filled_unknown.sum()), 0)
        self.assertEqual(int(cleared_occupied.sum()), 1)
        self.assertEqual(int(completed[5, 5]), 0)
        self.assertEqual(int(completed[0, 0]), -1)

    def test_unknown_boundary_outside_footprint_remains_unknown(self):
        grid = np.full((9, 9), -1, dtype=np.int8)
        footprint = np.zeros((9, 9), dtype=bool)
        footprint[3:6, 3:6] = True
        seeds = np.zeros_like(footprint)
        exclusion = np.zeros_like(footprint)

        completed, _, _ = complete_removed_object_footprint(
            grid, footprint, seeds, exclusion
        )

        self.assertEqual(int(np.count_nonzero(completed == -1)), 81)


class GeometryWallEvidenceTests(unittest.TestCase):
    def test_small_voxel_gaps_still_form_wall_chain(self):
        heights = np.asarray([2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26])
        self.assertAlmostEqual(low_connected_top(heights, 0.1), 2.6, places=5)

    def test_furniture_to_ceiling_gap_stops_vertical_chain(self):
        heights = np.asarray([2, 4, 6, 8, 10, 12, 14, 24, 26])
        self.assertAlmostEqual(low_connected_top(heights, 0.1), 1.4, places=5)


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
