"""Hardware-free checks for slider limits, ramping, and shutdown behavior."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from scripts.joint_sliders import (
    JOINTS, ArmController, DemoRobot, bounded_targets, calibrated_limits, ramp_targets,
)


class SliderChecks(unittest.TestCase):
    def make_controller(self):
        robot = DemoRobot()
        limits = {f"{joint}.pos": (0, 100) if joint == "gripper" else (-90, 90) for joint in JOINTS}
        return ArmController(robot, limits, {"shoulder_lift": 32, "elbow_flex": 36})

    def test_limits_follow_degree_normalization_with_margin(self):
        calibration = {joint: SimpleNamespace(range_min=100, range_max=2147) for joint in JOINTS}
        limits = calibrated_limits(calibration)
        self.assertAlmostEqual(limits["elbow_flex.pos"][1], 2047 * 180 / 4095 - 2)
        self.assertEqual(limits["gripper.pos"], (0, 100))

    def test_closed_gripper_pose_can_be_held_without_repositioning(self):
        controller = self.make_controller()
        controller.robot.positions["gripper.pos"] = 1.65
        actual = controller.enable()
        self.assertEqual(actual["gripper.pos"], 1.65)
        self.assertEqual(controller.robot.goals["gripper.pos"], 1.65)
        controller.stop()

    def test_targets_are_clamped_and_nonfinite_targets_rejected(self):
        limits = {"elbow_flex.pos": (-80, 80)}
        self.assertEqual(bounded_targets({"elbow_flex.pos": 200}, limits), {"elbow_flex.pos": 80})
        with self.assertRaises(ValueError):
            bounded_targets({"elbow_flex.pos": float("nan")}, limits)

    def test_delayed_tick_still_has_small_step_and_no_overshoot(self):
        result = ramp_targets({"elbow_flex.pos": 0, "gripper.pos": 50},
                              {"elbow_flex.pos": 90, "gripper.pos": 49.9}, 10, 1, 2)
        self.assertAlmostEqual(result["elbow_flex.pos"], 0.1)
        self.assertAlmostEqual(result["gripper.pos"], 49.9)

    def test_enable_holds_current_pose_and_stop_restores_gains(self):
        controller = self.make_controller()
        controller.robot.positions["elbow_flex.pos"] = -54
        actual = controller.enable()
        self.assertEqual(controller.commanded, actual)
        self.assertEqual(controller.robot.goals, actual)
        self.assertEqual(controller.robot.gains["elbow_flex"], 36)
        controller.health()
        controller.stop()
        self.assertEqual(controller.robot.torque, 0)
        self.assertEqual(controller.robot.gains["elbow_flex"], 16)
        self.assertEqual(controller.robot.gains["shoulder_lift"], 16)
        self.assertFalse(controller.enabled)

    def test_enable_rejects_pose_outside_slider_limits(self):
        controller = self.make_controller()
        controller.robot.positions["elbow_flex.pos"] = 91
        with self.assertRaises(RuntimeError):
            controller.enable()
        self.assertEqual(controller.robot.torque, 0)

    def test_lag_allows_bounded_lead_then_stops_without_progress(self):
        controller = self.make_controller()
        controller.enable()
        desired = controller.commanded.copy()
        desired["elbow_flex.pos"] = 50
        controller.commanded["elbow_flex.pos"] = 2
        with patch.object(controller.robot, "send_action", side_effect=lambda action: action.copy()):
            controller.tick(desired, 0.1, 1, 2, now=1)
            self.assertAlmostEqual(controller.commanded["elbow_flex.pos"], 2.1)
            with self.assertRaisesRegex(RuntimeError, "not tracking"):
                controller.tick(desired, 0.1, 1, 2, now=5)
        controller.stop()
        self.assertEqual(controller.robot.torque, 0)

    def test_lead_limit_does_not_accumulate_commands_on_stuck_joint(self):
        controller = self.make_controller()
        controller.enable()
        desired = controller.commanded.copy()
        desired["elbow_flex.pos"] = 50
        controller.commanded["elbow_flex.pos"] = 2.49
        with patch.object(controller.robot, "send_action", side_effect=lambda action: action.copy()):
            controller.tick(desired, 0.1, 1, 2, now=1)
        self.assertEqual(controller.commanded["elbow_flex.pos"], 2.49)
        controller.stop()

    def test_experimental_lead_only_changes_selected_joint(self):
        base = self.make_controller()
        controller = ArmController(base.robot, base.limits, base.gains, {"shoulder_lift.pos": 3.0})
        controller.enable()
        desired = controller.commanded.copy()
        for joint in ("shoulder_lift", "elbow_flex"):
            desired[f"{joint}.pos"] = 50
            controller.commanded[f"{joint}.pos"] = 2.49
        with patch.object(controller.robot, "send_action", side_effect=lambda action: action.copy()):
            controller.tick(desired, 0.1, 1, 2, now=1)
        self.assertAlmostEqual(controller.commanded["shoulder_lift.pos"], 2.59)
        self.assertEqual(controller.commanded["elbow_flex.pos"], 2.49)
        self.assertEqual(controller.max_lead["gripper.pos"], 2.5)
        controller.stop()

    def test_experimental_lead_remains_bounded_and_stall_detection_active(self):
        base = self.make_controller()
        controller = ArmController(base.robot, base.limits, base.gains, {"shoulder_lift.pos": 3.0})
        controller.enable()
        desired = controller.commanded.copy()
        desired["shoulder_lift.pos"] = 50
        controller.commanded["shoulder_lift.pos"] = 2.99
        with patch.object(controller.robot, "send_action", side_effect=lambda action: action.copy()):
            controller.tick(desired, 0.1, 1, 2, now=1)
            self.assertEqual(controller.commanded["shoulder_lift.pos"], 2.99)
            with self.assertRaisesRegex(RuntimeError, "not tracking"):
                controller.tick(desired, 0.1, 1, 2, now=5)
        controller.stop()

    def test_unknown_or_unbounded_lead_overrides_are_rejected(self):
        base = self.make_controller()
        for leads in ({"shoulder_lift.pos": 5}, {"gripper.pos": 3}, {"unknown.pos": 3}):
            with self.assertRaises(ValueError):
                ArmController(base.robot, base.limits, base.gains, leads)

    def test_moving_joint_with_offset_is_not_reported_as_stuck(self):
        controller = self.make_controller()
        controller.enable()
        desired = controller.commanded.copy()
        desired["elbow_flex.pos"] = 50
        controller.commanded["elbow_flex.pos"] = 3
        with patch.object(controller.robot, "send_action", side_effect=lambda action: action.copy()):
            for index in range(5):
                controller.robot.positions["elbow_flex.pos"] = 1 + index * 0.3
                controller.tick(desired, 0.1, 1, 2, now=1 + index * 2)
        self.assertTrue(controller.enabled)
        controller.stop()

    def test_sag_cannot_cause_command_to_jump_toward_actual_pose(self):
        controller = self.make_controller()
        controller.enable()
        desired = controller.commanded.copy()
        desired["elbow_flex.pos"] = -50
        controller.robot.positions["elbow_flex.pos"] = 4
        with patch.object(controller.robot, "send_action", side_effect=lambda action: action.copy()):
            controller.tick(desired, 0.1, 1, 2, now=1)
        self.assertEqual(controller.commanded["elbow_flex.pos"], 0)
        controller.stop()

    def test_goal_mismatch_is_detected(self):
        controller = self.make_controller()
        controller.enable()
        controller.robot.goals["elbow_flex.pos"] += 1
        with self.assertRaisesRegex(RuntimeError, "goal mismatch"):
            controller.health()
        controller.stop()

    def test_voltage_fault_is_identified_and_not_ignored(self):
        controller = self.make_controller()
        controller.enable()
        original_read = controller.robot.sync_read

        def read_with_voltage_fault(register, normalize=True):
            values = original_read(register, normalize=normalize)
            if register == "Status":
                values["gripper"] = 1
            return values

        with patch.object(controller.robot, "sync_read", side_effect=read_with_voltage_fault):
            with self.assertRaisesRegex(RuntimeError, "gripper: 1.*input voltage error"):
                controller.health()
        controller.stop()

    def test_failed_torque_off_does_not_restore_gains_under_torque(self):
        controller = self.make_controller()
        controller.enable()
        with patch.object(controller.robot, "disable_torque", side_effect=RuntimeError("serial error")):
            with self.assertRaisesRegex(RuntimeError, "Torque-off failed"):
                controller.stop()
        self.assertFalse(controller.enabled)
        self.assertEqual(controller.robot.gains["elbow_flex"], 36)
        with self.assertRaisesRegex(RuntimeError, "Previous torque-off failed"):
            controller.enable()
        controller.stop()
        self.assertEqual(controller.robot.gains["elbow_flex"], 16)

    def test_torque_off_failure_still_attempts_later_motors_and_locks_out_enable(self):
        controller = self.make_controller()
        controller.enable()
        disable = controller.robot.disable_torque

        def failing_wrist(motors=None, num_retry=0):
            if motors == "wrist_flex":
                raise RuntimeError("Incorrect status packet")
            disable(motors, num_retry=num_retry)

        with patch.object(controller.robot, "disable_torque", side_effect=failing_wrist) as attempts:
            with self.assertRaisesRegex(RuntimeError, "wrist_flex"):
                controller.stop()
            self.assertEqual([call.args[0] for call in attempts.call_args_list], list(JOINTS))
        self.assertEqual(controller.robot.torques["wrist_flex"], 1)
        self.assertTrue(all(controller.robot.torques[joint] == 0 for joint in JOINTS if joint != "wrist_flex"))
        self.assertEqual(controller.robot.gains["elbow_flex"], 36)
        controller.stop()  # A later successful cleanup can restore gains.
        self.assertEqual(controller.robot.torque, 0)
        with self.assertRaisesRegex(RuntimeError, "Previous torque-off failed"):
            controller.enable()  # Re-enabling still requires restarting after inspection.

    def test_lost_torque_off_ack_is_allowed_only_after_confirmed_zero_readback(self):
        controller = self.make_controller()
        controller.enable()
        disable = controller.robot.disable_torque

        def lost_ack(motors=None, num_retry=0):
            disable(motors, num_retry=num_retry)
            if motors == "wrist_flex":
                raise RuntimeError("lost acknowledgement")

        with patch.object(controller.robot, "disable_torque", side_effect=lost_ack):
            controller.stop()
        self.assertEqual(controller.robot.torque, 0)
        self.assertFalse(controller.shutdown_failed)
        self.assertEqual(controller.robot.gains["elbow_flex"], 16)

    def test_failed_torque_readback_cannot_be_reported_as_safe_shutdown(self):
        controller = self.make_controller()
        controller.enable()
        read = controller.robot.read

        def failed_readback(register, joint, **kwargs):
            if register == "Torque_Enable" and joint == "gripper":
                raise RuntimeError("communication timeout")
            return read(register, joint, **kwargs)

        with patch.object(controller.robot, "read", side_effect=failed_readback):
            with self.assertRaisesRegex(RuntimeError, "could not confirm OFF on gripper"):
                controller.stop()
        self.assertTrue(controller.shutdown_failed)
        self.assertFalse(controller.enabled)
        controller.stop()


if __name__ == "__main__":
    unittest.main()
