import unittest
from pathlib import Path

from stage2.vocabulary import load_semantic_aliases


class SemanticVocabularyTests(unittest.TestCase):
    def test_alias_groups_are_symmetric(self):
        aliases = load_semantic_aliases()
        self.assertEqual(aliases["tv"], aliases["television"])
        self.assertIn("kitchen cabinet", aliases["cabinet"])
        self.assertEqual(aliases["mug"], {"cup", "mug"})
        self.assertEqual(aliases["painting"], aliases["picture"])
        self.assertEqual(aliases["shelving"], aliases["shelf"])

    def test_comprehensive_offline_vocabulary_covers_small_objects(self):
        path = Path(__file__).resolve().parents[1] / "config/stage1_indoor_full_v1.txt"
        labels = {
            line.strip() for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        self.assertGreaterEqual(len(labels), 160)
        self.assertLessEqual(len(labels), 220)
        self.assertTrue({"cup", "mobile phone", "remote control", "keys", "shoe"} <= labels)


if __name__ == "__main__":
    unittest.main()
