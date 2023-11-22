import unittest
from src.perception.utils import match_bboxes_clustering, match_bboxes_points_matching

class TestMatchBboxes(unittest.TestCase):
    def setUp(self):
        # Initialize any necessary data here
        pass

    def test_match_bboxes_clustering(self):
        # Define your inputs for the function
        input1 = None
        input2 = None

        # Call the function with the inputs
        result = match_bboxes_clustering(input1, input2)

        # Assert the expected output
        self.assertEqual(result, expected_output)

    def test_match_bboxes_points_matching(self):
        # Define your inputs for the function
        input1 = None
        input2 = None

        # Call the function with the inputs
        result = match_bboxes_points_matching(input1, input2)

        # Assert the expected output
        self.assertEqual(result, expected_output)

if __name__ == '__main__':
    unittest.main()