import unittest

from stage2.viewpoint_recovery import ViewpointRecovery


class ViewpointRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.candidates = [{"id": value} for value in ("a", "b", "c", "d")]

    def test_not_found_switches_viewpoint(self):
        recovery = ViewpointRecovery(maximum_attempts=3)
        decision = recovery.decide("task", "a", False, self.candidates)
        self.assertTrue(decision["retry"])
        self.assertNotIn("a", [item["id"] for item in recovery.filtered_candidates({"task": self.candidates})["task"]])

    def test_retry_stops_at_budget(self):
        recovery = ViewpointRecovery(maximum_attempts=2)
        self.assertTrue(recovery.decide("task", "a", False, self.candidates)["retry"])
        self.assertFalse(recovery.decide("task", "b", False, self.candidates)["retry"])

    def test_found_never_retries(self):
        recovery = ViewpointRecovery()
        self.assertFalse(recovery.decide("task", "a", True, self.candidates)["retry"])

    def test_exhausted_location_is_filtered_but_other_location_remains(self):
        candidates = [
            {"id": "a", "object_id": "old"}, {"id": "b", "object_id": "old"},
            {"id": "c", "location_hypothesis_id": "new"},
        ]
        recovery = ViewpointRecovery(maximum_attempts=5, maximum_attempts_per_location=2)
        self.assertTrue(recovery.decide("task", "a", False, candidates)["retry"])
        decision = recovery.decide("task", "b", False, candidates)
        self.assertTrue(decision["location_exhausted"])
        filtered = recovery.filtered_candidates({"task": candidates})["task"]
        self.assertEqual([item["id"] for item in filtered], ["c"])

    def test_failed_viewpoints_continue_in_one_angular_direction(self):
        candidates = [
            {"id": "a", "object_id": "old", "candidate_angle_rad": 0.0},
            {"id": "cw", "object_id": "old", "candidate_angle_rad": 0.4},
            {"id": "ccw", "object_id": "old", "candidate_angle_rad": 2.0 * 3.141592653589793 - 0.4},
            {"id": "far", "object_id": "old", "candidate_angle_rad": 1.2},
        ]
        recovery = ViewpointRecovery(maximum_attempts=4, maximum_attempts_per_location=4)
        first = recovery.decide("task", "a", False, candidates)
        self.assertTrue(first["retry"])
        self.assertIn(first["direction"], (1, -1))
        filtered = recovery.filtered_candidates({"task": candidates})["task"]
        self.assertEqual(len(filtered), 1)
        first_next = filtered[0]["id"]
        self.assertIn(first_next, {"cw", "ccw"})

        second = recovery.decide("task", first_next, False, candidates)
        self.assertTrue(second["retry"])
        filtered_again = recovery.filtered_candidates({"task": candidates})["task"]
        self.assertEqual(len(filtered_again), 1)
        self.assertEqual(filtered_again[0]["id"], "far")

    def test_location_budget_does_not_count_each_viewpoint(self):
        candidates = [
            {"id": "a1", "location_hypothesis_id": "a"},
            {"id": "a2", "location_hypothesis_id": "a"},
            {"id": "a3", "location_hypothesis_id": "a"},
            {"id": "b1", "location_hypothesis_id": "b"},
            {"id": "b2", "location_hypothesis_id": "b"},
            {"id": "b3", "location_hypothesis_id": "b"},
        ]
        recovery = ViewpointRecovery(
            maximum_attempts=3,
            maximum_attempts_per_location=3,
            maximum_locations=2,
        )
        self.assertTrue(recovery.decide("task", "a1", False, candidates)["retry"])
        self.assertTrue(recovery.decide("task", "a2", False, candidates)["retry"])
        third = recovery.decide("task", "a3", False, candidates)
        self.assertTrue(third["retry"])
        self.assertEqual(third["location_attempt_index"], 1)
        self.assertIn("b1", third["remaining_candidate_ids"])

    def test_location_budget_stops_new_location_after_limit(self):
        candidates = [
            {"id": "a", "location_hypothesis_id": "a"},
            {"id": "b", "location_hypothesis_id": "b"},
            {"id": "c", "location_hypothesis_id": "c"},
        ]
        recovery = ViewpointRecovery(
            maximum_attempts=1,
            maximum_attempts_per_location=1,
            maximum_locations=2,
        )
        self.assertTrue(recovery.decide("task", "a", False, candidates)["retry"])
        second = recovery.decide("task", "b", False, candidates)
        self.assertFalse(second["retry"])
        self.assertEqual(second["location_attempt_index"], 2)

    def test_unbounded_mode_exhausts_location_then_releases_global_focus(self):
        candidates = [
            {"id": f"a{index}", "location_hypothesis_id": "bed_a",
             "candidate_angle_rad": index * 0.5}
            for index in range(4)
        ] + [{
            "id": "b1", "location_hypothesis_id": "bed_b",
            "candidate_angle_rad": 0.0,
        }]
        recovery = ViewpointRecovery()
        active = {"task"}

        for candidate_id in ("a0", "a1", "a2"):
            decision = recovery.decide("task", candidate_id, False, candidates)
            self.assertTrue(decision["retry"])
            self.assertEqual(recovery.focused_task_ids(active, {"task": candidates}), {"task"})

        decision = recovery.decide("task", "a3", False, candidates)
        self.assertTrue(decision["retry"])
        self.assertTrue(decision["location_exhausted"])
        self.assertEqual(recovery.focused_task_ids(active, {"task": candidates}), set())
        self.assertEqual(
            [item["id"] for item in recovery.filtered_candidates({"task": candidates})["task"]],
            ["b1"],
        )

        final = recovery.decide("task", "b1", False, candidates)
        self.assertFalse(final["retry"])
        self.assertEqual(final["reason"], "candidate_pool_exhausted")


if __name__ == "__main__":
    unittest.main()
