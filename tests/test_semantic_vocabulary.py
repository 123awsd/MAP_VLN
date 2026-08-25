import unittest

from stage2.vocabulary import load_semantic_aliases


class SemanticVocabularyTests(unittest.TestCase):
    def test_alias_groups_are_symmetric(self):
        aliases = load_semantic_aliases()
        self.assertEqual(aliases["tv"], aliases["television"])
        self.assertIn("kitchen cabinet", aliases["cabinet"])
        self.assertEqual(aliases["mug"], {"cup", "mug"})


if __name__ == "__main__":
    unittest.main()
