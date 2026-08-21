import math
import unittest

from loop_start_policy import select_loop_start


class LoopStartPolicyTest(unittest.TestCase):

    def setUp(self):
        self.targets = (
            {'name': 'A', 'x': 0.0, 'y': 0.0},
            {'name': 'B', 'x': 4.0, 'y': 0.0},
        )

    def test_near_a_starts_first_completed_leg_at_b(self):
        target, anchor, distance_a, distance_b = select_loop_start(
            self.targets, 0.118495, 0.0, 0.05, 0.20
        )
        self.assertEqual(1, target)
        self.assertEqual(0, anchor)
        self.assertAlmostEqual(0.118495, distance_a, places=6)
        self.assertGreater(distance_b, 3.0)

    def test_near_b_starts_first_completed_leg_at_a(self):
        target, anchor, _, distance_b = select_loop_start(
            self.targets, 3.91, 0.0, 0.05, 0.20
        )
        self.assertEqual(0, target)
        self.assertEqual(1, anchor)
        self.assertAlmostEqual(0.09, distance_b, places=6)

    def test_away_from_both_keeps_selected_a_as_first_target(self):
        target, anchor, _, _ = select_loop_start(
            self.targets, 2.0, 1.0, 0.05, 0.20
        )
        self.assertEqual(0, target)
        self.assertIsNone(anchor)

    def test_equal_distance_uses_a_as_anchor_deterministically(self):
        target, anchor, _, _ = select_loop_start(
            self.targets, 2.0, 0.0, 0.05, math.hypot(2.0, 0.0)
        )
        self.assertEqual(1, target)
        self.assertEqual(0, anchor)

    def test_rejects_non_finite_input(self):
        with self.assertRaises(ValueError):
            select_loop_start(
                self.targets, float('nan'), 0.0, 0.05, 0.20
            )

    def test_rejects_stale_localization(self):
        with self.assertRaises(ValueError):
            select_loop_start(
                self.targets, 0.10, 0.0, 0.301, 0.20,
                pose_timeout=0.30
            )


if __name__ == '__main__':
    unittest.main(verbosity=2)
