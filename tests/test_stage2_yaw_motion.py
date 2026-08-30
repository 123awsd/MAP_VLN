import math
import unittest

from stage2.yaw_motion import blend_yaw, facing_yaw, rotation_steps, step_yaw, yaw_delta


class YawMotionTest(unittest.TestCase):
    def test_shortest_delta_crosses_wrap_boundary(self):
        self.assertAlmostEqual(yaw_delta(math.radians(170), math.radians(-170)), math.radians(20))

    def test_step_is_rate_limited(self):
        result = step_yaw(0.0, math.pi, math.radians(12))
        self.assertLessEqual(abs(yaw_delta(0.0, result)), math.radians(12) + 1e-9)

    def test_rotation_steps_include_exact_target_without_large_jump(self):
        maximum = math.radians(10)
        values = rotation_steps(math.radians(170), math.radians(-150), maximum)
        sequence = [math.radians(170)] + values
        self.assertAlmostEqual(yaw_delta(values[-1], math.radians(-150)), 0.0)
        self.assertTrue(all(
            abs(yaw_delta(start, end)) <= maximum + 1e-9
            for start, end in zip(sequence, sequence[1:])
        ))

    def test_blend_uses_shortest_direction(self):
        middle = blend_yaw(math.radians(170), math.radians(-170), 0.5)
        self.assertAlmostEqual(abs(middle), math.pi)

    def test_facing_yaw_points_from_position_to_target(self):
        self.assertAlmostEqual(facing_yaw([1.0, 1.0, 2.0], [1.0, 3.0, 9.0], 0.0), math.pi / 2)

    def test_facing_yaw_keeps_fallback_at_coincident_xy(self):
        fallback = math.radians(35)
        self.assertAlmostEqual(facing_yaw([1.0, 1.0, 2.0], [1.0, 1.0, 9.0], fallback), fallback)


if __name__ == "__main__":
    unittest.main()
