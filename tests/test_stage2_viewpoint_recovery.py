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


if __name__ == "__main__":
    unittest.main()
