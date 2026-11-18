"""Check motion properties of the continuous path, independently of hardware."""

import unittest
import numpy as np

from scripts.smooth_trajectory import SmoothJointTrajectory


class SmoothTrajectoryChecks(unittest.TestCase):
    def trajectory(self):
        return SmoothJointTrajectory([[0, 1], [0.2, 0.8], [0.5, 0.6], [1, 0.2]],
                                     [[0, 0, 0], [0.5, 0, 0], [1, 0, 0], [1.5, 0, 0]],
                                     joint_speed=0.4, tip_speed=1.)

    def test_rest_at_endpoints_but_no_stop_at_internal_waypoints(self):
        path = self.trajectory()
        np.testing.assert_allclose(path.sample(0)[1], 0, atol=1e-12)
        np.testing.assert_allclose(path.sample(path.duration)[1], 0, atol=1e-12)
        for stamp in path.times[1:-1]:
            self.assertGreater(np.linalg.norm(path.sample(stamp)[1]), 0.01)

    def test_position_and_velocity_are_continuous_at_waypoints(self):
        path = self.trajectory()
        for index, stamp in enumerate(path.times[1:-1], 1):
            q0, v0 = path.sample(stamp - 1e-7)
            q1, v1 = path.sample(stamp + 1e-7)
            np.testing.assert_allclose(q0, q1, atol=1e-6)
            np.testing.assert_allclose(v0, v1, atol=1e-6)
            np.testing.assert_allclose(path.sample(stamp)[0], path.joints[index], atol=1e-10)

    def test_speed_limit_and_no_joint_overshoot_between_points(self):
        path = self.trajectory()
        for index in range(len(path.times) - 1):
            low = np.minimum(path.joints[index], path.joints[index + 1])
            high = np.maximum(path.joints[index], path.joints[index + 1])
            for stamp in np.linspace(path.times[index], path.times[index + 1], 100):
                q, velocity = path.sample(stamp)
                self.assertTrue(np.all(q >= low - 1e-10))
                self.assertTrue(np.all(q <= high + 1e-10))
                self.assertLessEqual(np.max(np.abs(velocity)), 0.4 + 1e-10)

    def test_reversing_joint_stops_at_turn_without_overshoot(self):
        path = SmoothJointTrajectory([[0], [1], [0]], [[0, 0, 0], [1, 0, 0], [2, 0, 0]], 1., 1.)
        self.assertAlmostEqual(path.sample(path.times[1])[1][0], 0)
        for stamp in np.linspace(0, path.duration, 100):
            self.assertTrue(-1e-12 <= path.sample(stamp)[0][0] <= 1 + 1e-12)

    def test_cancelled_or_invalid_trajectory_is_rejected(self):
        for speed in (0, float("nan")):
            with self.assertRaises(ValueError):
                SmoothJointTrajectory([[0], [1]], [[0, 0, 0], [1, 0, 0]], speed, 1.)
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            SmoothJointTrajectory([[0], [1]], [[0, 0, 0], [1, 0, 0]], 1., 1., cancelled=lambda: True)


if __name__ == "__main__":
    unittest.main()
