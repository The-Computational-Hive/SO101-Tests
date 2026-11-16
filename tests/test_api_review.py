"""Hardware-free checks for diagnostic startup and movement pass criteria."""

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from scripts.test_api import SafeStartFollower, validate_endpoint


class ReviewChecks(unittest.TestCase):
    def test_configuration_never_enables_torque(self):
        robot = SimpleNamespace(
            bus=MagicMock(),
            config=SimpleNamespace(position_p_coefficient=16,
                                   position_i_coefficient=0, position_d_coefficient=32),
        )
        robot.bus.motors = {"elbow_flex": None, "gripper": None}
        SafeStartFollower.configure(robot)
        robot.bus.enable_torque.assert_not_called()
        self.assertEqual(robot.bus.disable_torque.call_count, 2)
        robot.bus.write.assert_any_call("P_Coefficient", "elbow_flex", 16)
        robot.bus.write.assert_any_call("Protection_Current", "gripper", 250)

    def test_configuration_failure_leaves_torque_disabled(self):
        robot = SimpleNamespace(bus=MagicMock(), config=SimpleNamespace())
        robot.bus.configure_motors.side_effect = RuntimeError("connection failure")
        with self.assertRaises(RuntimeError):
            SafeStartFollower.configure(robot)
        robot.bus.enable_torque.assert_not_called()
        self.assertEqual(robot.bus.disable_torque.call_count, 2)

    def test_stationary_joint_cannot_pass_two_degree_test(self):
        with self.assertRaises(RuntimeError):
            validate_endpoint({"elbow_flex.pos": 10.0}, {"elbow_flex.pos": 12.0},
                              {"elbow_flex.pos": 10.0}, "elbow_flex.pos", 2.0, True)

    def test_motion_in_wrong_direction_cannot_pass(self):
        with self.assertRaises(RuntimeError):
            validate_endpoint({"elbow_flex.pos": 9.5}, {"elbow_flex.pos": 12.0},
                              {"elbow_flex.pos": 10.0}, "elbow_flex.pos", 2.0, True)

    def test_reached_target_and_return_pass(self):
        validate_endpoint({"elbow_flex.pos": 11.5}, {"elbow_flex.pos": 12.0},
                          {"elbow_flex.pos": 10.0}, "elbow_flex.pos", 2.0, True)
        validate_endpoint({"elbow_flex.pos": 10.5}, {"elbow_flex.pos": 10.0},
                          {"elbow_flex.pos": 12.0}, "elbow_flex.pos", 2.0, False)


if __name__ == "__main__":
    unittest.main()
