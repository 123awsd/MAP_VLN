import unittest

from stage2.representative_viewpoints import (
    representative_score,
    select_location_representatives,
)


def candidate(identifier, location, x, terminal=0.0, yaw=0.0):
    return {
        "id": identifier,
        "object_id": location,
        "location_hypothesis_id": location,
        "pose": {"x": x, "y": 0.0, "z": 1.0, "yaw": yaw},
        "terminal_cost": terminal,
    }


class RepresentativeViewpointTest(unittest.TestCase):
    def test_keeps_one_dynamic_representative_per_location(self):
        values = {
            "task": [
                candidate("a_far", "bed_a", 5.0),
                candidate("a_near", "bed_a", 1.0),
                candidate("b_best_view", "bed_b", 3.0, terminal=0.0),
                candidate("b_bad_view", "bed_b", 2.5, terminal=2.0),
            ]
        }
        result = select_location_representatives(
            values, [0.0, 0.0, 1.0, 0.0], {"task"}, maximum_per_location=1,
        )
        self.assertEqual({item["id"] for item in result["task"]}, {"a_near", "b_best_view"})

    def test_fallback_limit_keeps_second_ranked_pose(self):
        values = {"task": [
            candidate("best", "bed", 1.0), candidate("fallback", "bed", 2.0),
            candidate("third", "bed", 3.0),
        ]}
        result = select_location_representatives(
            values, [0.0, 0.0, 1.0, 0.0], {"task"}, maximum_per_location=2,
        )
        self.assertEqual([item["id"] for item in result["task"]], ["best", "fallback"])

    def test_yaw_and_terminal_quality_affect_entry_score(self):
        current = [0.0, 0.0, 1.0, 0.0]
        aligned = candidate("aligned", "bed", 1.0, terminal=0.0, yaw=0.0)
        poor = candidate("poor", "bed", 1.0, terminal=1.0, yaw=3.0)
        self.assertLess(representative_score(aligned, current), representative_score(poor, current))


if __name__ == "__main__":
    unittest.main()
