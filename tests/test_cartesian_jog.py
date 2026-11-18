"""Offline kinematics and Cartesian controller checks; no serial ports or windows."""

import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from scripts.cartesian_jog import CartesianWorker, body_vector
from scripts.so101_kinematics import MODEL_PATH, SO101Kinematics


class KinematicsChecks(unittest.TestCase):
    def setUp(self):
        self.kin = SO101Kinematics()
        self.q = np.array([0., -30., 30., 30., -90.])

    def test_pan_rotation_agrees_with_base_axis_and_millimetres(self):
        origin = self.kin.xyz(np.zeros(5))
        rotated = self.kin.xyz([90, 0, 0, 0, 0])
        pivot = np.array([38.8353, -0.00000897657])
        delta = origin[:2] - pivot
        expected = pivot + [delta[1], -delta[0]]
        np.testing.assert_allclose(rotated[:2], expected, atol=0.01)
        self.assertAlmostEqual(rotated[2], origin[2], places=2)
        self.assertTrue(100 < origin[0] < 500)

    def test_tool_offset_is_local_and_in_millimetres(self):
        extended = SO101Kinematics(tool_offset_mm=[0, 0, 50])
        transform = self.kin.transform(self.q)
        np.testing.assert_allclose(extended.xyz(self.q) - self.kin.xyz(self.q), transform[:3, 2] * 50)

    def test_local_inverse_kinematics_reaches_generated_targets_within_bounds(self):
        random = np.random.default_rng(17)
        for _ in range(12):
            seed = random.uniform([-50, -40, -50, -40, -100], [50, 40, 50, 40, 100])
            nearby = seed + random.uniform(-0.5, 0.5, 5)
            target = self.kin.xyz(nearby)
            result = self.kin.solve(target, seed, self.kin.limits)
            self.assertLessEqual(np.linalg.norm(self.kin.xyz(result) - target), 0.15)
            self.assertTrue(np.all(result >= self.kin.limits[:, 0]))
            self.assertTrue(np.all(result <= self.kin.limits[:, 1]))

    def test_preview_has_small_continuous_steps_and_can_preserve_direction(self):
        target = self.kin.xyz(self.q) + [0, 0, 1]
        plan = self.kin.plan(self.q, target, self.kin.limits, hold_direction=True)
        self.assertEqual(len(plan.waypoints), 2)
        joints = np.array([self.q, *plan.waypoints])
        self.assertLessEqual(np.max(np.abs(np.diff(joints, axis=0))), 2)
        for point, angles in zip(plan.xyz_mm, plan.waypoints):
            self.assertLessEqual(np.linalg.norm(self.kin.xyz(angles) - point), 0.15)
            direction = self.kin.transform(angles)[:3, 2]
            self.assertGreaterEqual(direction @ self.kin.transform(self.q)[:3, 2], np.cos(np.deg2rad(0.5)))

    def test_large_nonfinite_unreachable_and_cancelled_targets_are_rejected(self):
        origin = self.kin.xyz(self.q)
        for target in (origin + [11, 0, 0], [float("nan"), 0, 0]):
            with self.assertRaises(ValueError):
                self.kin.plan(self.q, target, self.kin.limits)
        with self.assertRaises(ValueError):
            self.kin.solve([900, 0, 0], self.q, self.kin.limits)
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            self.kin.plan(self.q, origin + [1, 0, 0], self.kin.limits, cancelled=lambda: True)

    def test_calibration_limits_only_narrow_model_limits(self):
        calibrated = {f"{name}.pos": (-20, 20) for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")}
        np.testing.assert_allclose(self.kin.intersect_limits(calibrated), np.tile([-20, 20], (5, 1)))
        calibrated["elbow_flex.pos"] = (150, 160)
        with self.assertRaises(ValueError):
            self.kin.intersect_limits(calibrated)


class CartesianWorkerChecks(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path_patch = patch("scripts.cartesian_jog.Path", side_effect=lambda path: Path(self.directory.name) / path)
        self.path_patch.start()
        args = SimpleNamespace(model=MODEL_PATH, tool_offset_mm=(0, 0, 0), demo=True, shoulder_p=48, elbow_p=36,
                               tip_speed=1., joint_speed=2 / 3)
        self.worker = CartesianWorker(args)
        self.worker.connect()  # Demo only: the worker thread is not started.

    def tearDown(self):
        if self.worker.is_alive():
            self.worker.close_event.set()
            self.worker.join(timeout=3)
            self.assertFalse(self.worker.is_alive())
        self.worker.stop()
        self.worker.robot.disconnect()
        if self.worker.log:
            self.worker.log.close()
        self.path_patch.stop()
        self.directory.cleanup()

    def target(self):
        return self.worker.kinematics.xyz(body_vector(self.worker.controller.pose())) + [1, 0, 0]

    def test_preview_with_torque_off_sends_no_actions_and_cannot_move(self):
        with patch.object(self.worker.robot, "send_action", wraps=self.worker.robot.send_action) as send:
            self.worker.preview(self.target(), False, 1)
            send.assert_not_called()
            with self.assertRaises(ValueError):
                self.worker.start_motion(self.worker.pending[0])
        self.assertEqual(self.worker.robot.torque, 0)

    def test_jog_click_starts_validated_motion_without_second_move_request(self):
        self.worker.handle("enable", None)
        origin = self.worker.kinematics.xyz(body_vector(self.worker.controller.commanded))
        self.worker.handle("jog", (0, 1., False, 1))
        self.assertIsNotNone(self.worker.motion)
        self.assertIsNone(self.worker.pending)
        np.testing.assert_allclose(self.worker.motion["plan"].target_mm, origin + [1, 0, 0])

    def test_absolute_xyz_click_starts_motion_and_rejects_invalid_targets(self):
        self.worker.handle("enable", None)
        self.worker.handle("goto", (self.target(), False, 1))
        self.assertIsNotNone(self.worker.motion)
        self.worker.handle("hold", None)
        reference = self.worker.controller.commanded.copy()
        with patch.object(self.worker.robot, "send_action", wraps=self.worker.robot.send_action) as send:
            with self.assertRaises(ValueError):
                self.worker.handle("goto", (self.target() + [20, 0, 0], False, 2))
            send.assert_not_called()
        self.assertIsNone(self.worker.motion)
        self.assertEqual(self.worker.controller.commanded, reference)

    def test_direct_clicks_cannot_enable_torque_implicitly(self):
        for request, payload in (("jog", (0, 1., False, 1)), ("goto", (self.target(), False, 1))):
            with self.assertRaisesRegex(ValueError, "Enable holding"):
                self.worker.handle(request, payload)
        self.assertEqual(self.worker.robot.torque, 0)

    def test_extra_click_during_motion_does_not_queue_or_replace_target(self):
        self.worker.handle("enable", None)
        self.worker.handle("jog", (0, 1., False, 1))
        motion = self.worker.motion
        with self.assertRaises(ValueError):
            self.worker.handle("jog", (1, 1., False, 2))
        self.assertIs(self.worker.motion, motion)

    def test_execution_follows_preview_preserves_gripper_and_finishes_holding(self):
        self.worker.handle("enable", None)
        gripper = self.worker.controller.commanded["gripper.pos"]
        target = self.target()
        self.worker.preview(target, False, 1)
        self.worker.start_motion(self.worker.pending[0])
        previous = body_vector(self.worker.controller.commanded)
        for index in range(500):
            now = index * 0.05
            self.worker.control_tick(0.05, now)
            current = body_vector(self.worker.controller.commanded)
            self.assertLessEqual(np.max(np.abs(current - previous)), (2 / 3) * 0.05 + 1e-9)
            self.assertEqual(self.worker.controller.commanded["gripper.pos"], gripper)
            previous = current
            if self.worker.motion is None:
                break
        self.assertIsNone(self.worker.motion)
        self.assertTrue(self.worker.controller.enabled)
        self.assertLess(np.linalg.norm(self.worker.kinematics.xyz(body_vector(self.worker.controller.pose())) - target), 0.2)

    def test_motion_controller_receives_final_destination_not_only_intermediate_setpoint(self):
        self.worker.handle("enable", None)
        self.worker.handle("jog", (0, 2., False, 1))
        expected = self.worker.motion["plan"].waypoints[-1].copy()
        with patch.object(self.worker.controller, "tick", wraps=self.worker.controller.tick) as tick:
            self.worker.control_tick(0.05, now=1)
        np.testing.assert_allclose(body_vector(tick.call_args.kwargs["endpoint_target"]), expected)
        self.assertGreater(np.max(np.abs(body_vector(tick.call_args.args[0]) - expected)), 0.001)

    def test_drift_invalidates_preview(self):
        self.worker.handle("enable", None)
        self.worker.preview(self.target(), False, 1)
        self.worker.robot.positions["elbow_flex.pos"] += 2
        with self.assertRaisesRegex(ValueError, "drifted"):
            self.worker.start_motion(self.worker.pending[0])
        self.assertIsNone(self.worker.motion)

    def test_cancel_holds_previous_command_without_retargeting_sagged_pose(self):
        self.worker.handle("enable", None)
        reference = self.worker.controller.commanded.copy()
        self.worker.preview(self.target(), False, 1)
        self.worker.start_motion(self.worker.pending[0])
        self.worker.robot.positions["elbow_flex.pos"] += 0.5
        self.worker.handle("hold", None)
        self.assertEqual(self.worker.controller.commanded, reference)
        self.assertIsNone(self.worker.motion)
        self.assertTrue(self.worker.controller.enabled)

    def test_worker_heartbeat_loss_disables_torque_and_restores_gains(self):
        self.worker.handle("enable", None)
        self.worker.heartbeat = time.monotonic() - 2
        self.worker.start()
        deadline = time.monotonic() + 2
        stopped = False
        while time.monotonic() < deadline:
            while not self.worker.updates.empty():
                kind, value = self.worker.updates.get_nowait()
                if kind == "status" and "heartbeat lost" in value:
                    stopped = True
            if stopped:
                break
            time.sleep(0.01)
        self.assertTrue(stopped)
        self.assertFalse(self.worker.controller.enabled)
        self.assertEqual(self.worker.robot.torque, 0)
        self.assertEqual(self.worker.robot.gains["shoulder_lift"], 16)

    def test_invalid_preview_revision_is_carried_to_ui_for_stale_result_rejection(self):
        self.worker.preview(self.target(), False, 42)
        previews = []
        while not self.worker.updates.empty():
            kind, payload = self.worker.updates.get_nowait()
            if kind == "preview":
                previews.append(payload)
        self.assertEqual(previews[0]["revision"], 42)


if __name__ == "__main__":
    unittest.main()
