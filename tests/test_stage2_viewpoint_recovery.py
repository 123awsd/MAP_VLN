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


if __name__ == "__main__":
    unittest.main()
