import unittest
import time
from unittest.mock import MagicMock, patch

import numpy as np

from stage2.open_vocab_detector import AsyncOpenVocabularyDetector, LocalOpenVocabularyDetector, apply_class_thresholds, associate_projection, project_detection_to_world, target_found


class OpenVocabularyDetectorTest(unittest.TestCase):
    @patch.object(LocalOpenVocabularyDetector, "_read_prefixed", return_value={"backend": "fake"})
    @patch("stage2.open_vocab_detector.subprocess.Popen")
    def test_worker_can_override_cuda_visibility(self, popen, _ready):
        process = MagicMock()
        process.poll.return_value = 0
        popen.return_value = process
        LocalOpenVocabularyDetector(["chair"], cuda_visible_devices="0")
        environment = popen.call_args.kwargs["env"]
        self.assertEqual(environment["CUDA_VISIBLE_DEVICES"], "0")

    def test_target_found_supports_aliases(self):
        result = {"detections": [{"label": "tv", "score": 0.4}]}
        self.assertTrue(target_found(result, "television", {"television": ["tv"]}))
        self.assertFalse(target_found(result, "chair"))


    def test_project_detection_to_world_at_image_center(self):
        depth = np.full((480, 640), 2.0, dtype=np.float32)
        lifted = project_detection_to_world(
            {"label": "chair", "score": 0.7, "bbox_xyxy": [300, 200, 340, 280]},
            depth,
            [1.0, 2.0, 1.2, 0.0],
        )
        self.assertIsNotNone(lifted)
        self.assertEqual(lifted["center"], [3.0, 2.0, 1.2])
        self.assertEqual(lifted["depth_m"], 2.0)


    def test_projection_rejects_missing_depth(self):
        depth = np.zeros((10, 10), dtype=np.float32)
        self.assertIsNone(project_detection_to_world(
            {"label": "chair", "score": 0.7, "bbox_xyxy": [1, 1, 9, 9]},
            depth,
            [0, 0, 0, 0],
        ))

    def test_association_gates_distant_false_positive(self):
        objects = [{"id": "tv_1", "label": "television", "center_xyz_m": [1.0, 2.0, 1.0]}]
        near = associate_projection(
            {"label": "television", "center": [1.2, 2.0, 1.0]}, objects
        )
        far = associate_projection(
            {"label": "television", "center": [4.0, 2.0, 1.0]}, objects
        )
        self.assertTrue(near["confirmed"])
        self.assertEqual(near["associated_object_id"], "tv_1")
        self.assertFalse(far["confirmed"])

    def test_class_specific_threshold_filters_weak_detection(self):
        result = {"detections": [
            {"label": "television", "score": 0.4},
            {"label": "chair", "score": 0.4},
        ]}
        filtered = apply_class_thresholds(result, 0.2, {"television": 0.45})
        self.assertEqual([item["label"] for item in filtered["detections"]], ["chair"])

    def test_async_detector_drops_request_while_busy(self):
        class FakeBackend:
            def detect(self, image_path):
                time.sleep(0.02)
                return {"image_path": str(image_path), "detections": []}

        detector = AsyncOpenVocabularyDetector(FakeBackend())
        try:
            self.assertTrue(detector.submit("first.jpg", {"frame_index": 1}))
            self.assertFalse(detector.submit("second.jpg", {"frame_index": 2}))
            result = detector.flush()
            self.assertEqual(result["metadata"]["frame_index"], 1)
            self.assertEqual(detector.stats, {"submitted": 1, "dropped_busy": 1})
        finally:
            detector.close()


if __name__ == "__main__":
    unittest.main()
