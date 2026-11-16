"""Small joint trajectory using LeRobot's observation and action APIs."""

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


class SafeStartFollower(SO101Follower):
    """Apply LeRobot 0.6.1's configuration without enabling stale targets."""

    def configure(self):
        # Upstream configure() exits torque_disabled(), which enables all motors.
        # Leave torque off until a current-position action has been confirmed.
        self.bus.disable_torque()
        try:
            self.bus.configure_motors()
            for motor in self.bus.motors:
                for register, value in (
                    ("Operating_Mode", 0),
                    ("P_Coefficient", self.config.position_p_coefficient),
                    ("I_Coefficient", self.config.position_i_coefficient),
                    ("D_Coefficient", self.config.position_d_coefficient),
                ):
                    self.bus.write(register, motor, value)
                if motor == "gripper":
                    for register, value in (("Max_Torque_Limit", 500),
                                            ("Protection_Current", 250),
                                            ("Overload_Torque", 25)):
                        self.bus.write(register, motor, value)
        finally:
            self.bus.disable_torque()


def validate_endpoint(actual, target, origin, selected, amount, outward):
    for key in target:
        tolerance = min(1.0, amount / 2) if key == selected else 2.0
        if abs(actual[key] - target[key]) > tolerance:
            raise RuntimeError(
                f"Target not reached on {key}: target={target[key]:.3f}, "
                f"actual={actual[key]:.3f}, tolerance={tolerance:.3f}"
            )
    if outward:
        direction = 1 if target[selected] > origin[selected] else -1
        progress = (actual[selected] - origin[selected]) * direction
        if progress < amount * 0.5:
            raise RuntimeError("Less than half the requested movement was measured; test did not pass.")


def read_pose(robot):
    observation = robot.get_observation()
    pose = {key: float(observation[key]) for key in robot.action_features}
    if not all(math.isfinite(value) for value in pose.values()):
        raise RuntimeError("Invalid joint position reading.")
    return pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument(
        "--joint", choices=("shoulder_pan", "shoulder_lift", "elbow_flex",
                            "wrist_flex", "wrist_roll", "gripper"), default="elbow_flex",
    )
    amount_group = parser.add_mutually_exclusive_group()
    amount_group.add_argument("--degrees", type=float, help="Body-joint movement in degrees (default: 2).")
    amount_group.add_argument("--amount", type=float, help="Movement size: degrees for body joints, 0-100 units for gripper (default: 2).")
    parser.add_argument(
        "--speed", type=float, default=2.0 / 3.0,
        help="Trajectory units/second: degrees for body joints, 0-100 units for gripper (default: 0.667; base maximum: 5).",
    )
    parser.add_argument(
        "--direction", choices=("toward-zero", "away-from-zero"),
        default="toward-zero", help="Initial direction of the joint test (may cross zero).",
    )
    parser.add_argument(
        "--diagnostics", action="store_true",
        help="Read and log the selected joint's raw voltage, current, load, and status.",
    )
    gain_group = parser.add_mutually_exclusive_group()
    gain_group.add_argument(
        "--joint-p", type=int, choices=(16, 20, 32, 36, 40, 48),
        help="Temporary P gain for the selected shoulder_lift or elbow_flex joint; restored at shutdown.",
    )
    gain_group.add_argument(
        "--elbow-p", type=int, choices=(16, 20, 32, 36, 40, 48),
        help="Legacy elbow-only alias for --joint-p; restored at shutdown.",
    )
    args = parser.parse_args()
    if args.joint == "gripper" and args.degrees is not None:
        parser.error("For gripper use --amount, since gripper positions are on a 0-100 scale, not degrees")
    amount = args.amount if args.amount is not None else args.degrees
    args.degrees = 2.0 if amount is None else amount
    units = "gripper units" if args.joint == "gripper" else "degrees"
    if args.elbow_p is not None and args.joint != "elbow_flex":
        parser.error("--elbow-p is only valid with --joint elbow_flex")
    if args.elbow_p is not None:
        args.joint_p = args.elbow_p
    if args.joint_p is not None and args.joint not in ("shoulder_lift", "elbow_flex"):
        parser.error("--joint-p is only supported for shoulder_lift and elbow_flex")
    maximum = 10.0 if args.joint == "shoulder_pan" else 2.0
    if not math.isfinite(args.degrees) or not 0 < args.degrees <= maximum:
        parser.error(f"Movement size must be greater than zero and at most {maximum:g} for {args.joint}")
    maximum_speed = 5.0 if args.joint == "shoulder_pan" else 2.0 / 3.0
    if not math.isfinite(args.speed) or not 0 < args.speed <= maximum_speed:
        parser.error(f"--speed must be greater than zero and at most {maximum_speed:g} for {args.joint}")
    movement_duration = args.degrees / args.speed
    robot = SafeStartFollower(SO101FollowerConfig(
        port=args.port, id=args.robot_id, use_degrees=True,
        max_relative_target=3.0,
    ))
    if not robot.calibration:
        raise RuntimeError("No saved calibration found. Check --robot-id.")

    original_joint_p = None
    log_path = Path("logs") / f"api_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
    try:
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            raise RuntimeError("Saved calibration does not match the motors.")
        if args.diagnostics:
            print(f"Read-only {args.joint} diagnostics; values are raw register counts.")
            for register in ("Min_Voltage_Limit", "Max_Voltage_Limit", "Protection_Current"):
                value = robot.bus.read(register, args.joint, normalize=False)
                print(f"{register}: {value}")
        print("Support the arm during startup; secure the base and clear the path.")
        direction_label = args.direction.replace("-", " ")
        print(f"All joints hold; only {args.joint} moves {args.degrees:g} {units} initially {direction_label} and back.")
        print(f"Commanded speed: {args.speed:g} {units}/s; each movement takes approximately {movement_duration:g} seconds.")
        print("Ctrl+C/errors disable torque. This trajectory is not collision checked.")
        if args.joint_p is not None:
            print(f"{args.joint}-only P gain experiment: {args.joint_p}; I/D and other joints unchanged.")
        if input("Type MOVE to continue: ").strip() != "MOVE":
            return

        if args.joint_p is not None:
            # The public robot configuration only supports a shared gain.
            # This isolated configuration change does not command movement.
            robot.bus.disable_torque()
            original_joint_p = robot.bus.read("P_Coefficient", args.joint, normalize=False)
            robot.bus.write("P_Coefficient", args.joint, args.joint_p, normalize=False)
            stored_p = robot.bus.read("P_Coefficient", args.joint, normalize=False)
            if stored_p != args.joint_p:
                raise RuntimeError("Joint P gain readback mismatch.")
            print(f"{args.joint} P: {original_joint_p} -> {stored_p}; restored on normal/error shutdown.")

        start = read_pose(robot)
        end = dict(start)
        joint_key = f"{args.joint}.pos"
        delta = -args.degrees if start[joint_key] >= 0 else args.degrees
        if args.direction == "away-from-zero":
            delta = -delta
        end[joint_key] += delta
        calibration = robot.calibration[args.joint]
        half_range_degrees = (calibration.range_max - calibration.range_min) * 180 / 4095
        outside_range = (
            not 0 <= end[joint_key] <= 100 if args.joint == "gripper"
            else abs(end[joint_key]) > half_range_degrees
        )
        if outside_range:
            raise RuntimeError("Joint target is outside its calibrated range; stopping.")
        print(f"{args.joint}: {start[joint_key]:.2f} -> {end[joint_key]:.2f} -> {start[joint_key]:.2f} {units}")
        # send_action's return value is a logical/clipped target, not motor acknowledgement.
        robot.send_action(start)
        for name in robot.bus.motors:
            stored = robot.bus.read("Goal_Position", name)
            if abs(stored - start[f"{name}.pos"]) > 0.2:
                raise RuntimeError(f"Starting target readback failed on {name}")
            torque = robot.bus.read("Torque_Enable", name, normalize=False)
            if torque == 0:
                robot.bus.enable_torque(name)
            elif torque != 1:
                raise RuntimeError(f"Unexpected torque state on {name}: {torque}")
        log_path.parent.mkdir(exist_ok=True)
        began = time.monotonic()
        with log_path.open("x", encoding="utf-8", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(["elapsed_s", "phase", "joint", "requested", "sent", "actual", "error",
                             "tested_joint_voltage_raw", "tested_joint_current_raw", "tested_joint_load_raw", "tested_joint_status_raw",
                             "tested_joint_p_override", "tested_joint_stored_goal", "tested_joint_torque",
                             "tested_joint_temperature_raw"])

            def run_phase(label, first, last, duration):
                phase_start = time.monotonic()
                next_tick = phase_start
                next_print = phase_start
                while True:
                    now = time.monotonic()
                    fraction = min(1.0, (now - phase_start) / duration)
                    requested = {key: first[key] + (last[key] - first[key]) * fraction for key in first}
                    sent = robot.send_action(requested)
                    actual = read_pose(robot)
                    stored_goal = robot.bus.read("Goal_Position", args.joint)
                    torque = robot.bus.read("Torque_Enable", args.joint, normalize=False)
                    temperature = robot.bus.read("Present_Temperature", args.joint, normalize=False)
                    telemetry = ["", "", "", ""]
                    if args.diagnostics:
                        # These are diagnostic reads only; actions still use send_action().
                        telemetry = [robot.bus.read(register, args.joint, normalize=False)
                                     for register in ("Present_Voltage", "Present_Current", "Present_Load", "Status")]
                    for key in requested:
                        writer.writerow([round(now - began, 3), label, key, requested[key],
                                         sent[key], actual[key], actual[key] - requested[key], *telemetry,
                                         args.joint_p if args.joint_p is not None else "",
                                         stored_goal, torque, temperature])
                    output.flush()
                    if now >= next_print:
                        diagnostics = (
                            f", voltage_raw={telemetry[0]}, current_raw={telemetry[1]}, "
                            f"load_raw={telemetry[2]}, status_raw={telemetry[3]}"
                            if args.diagnostics else ""
                        )
                        print(f"{label}: target={requested[joint_key]:.2f}, sent={sent[joint_key]:.2f}, "
                              f"actual={actual[joint_key]:.2f}, error={actual[joint_key] - requested[joint_key]:+.3f}"
                              f", stored={stored_goal:.2f}, torque={torque}, temperature_raw={temperature}"
                              f"{diagnostics}",
                              flush=True)
                        next_print = now + 0.5
                    if abs(stored_goal - sent[joint_key]) > 0.2 or torque != 1:
                        raise RuntimeError("Stored target or motor torque does not match the action.")
                    if args.diagnostics and telemetry[3] != 0:
                        raise RuntimeError(f"{args.joint} reports status fault {telemetry[3]}; stopping.")
                    if any(abs(actual[k] - requested[k]) > 5.0 for k in requested):
                        raise RuntimeError(f"{label}: tracking error exceeded 5 units; stopping.")
                    if any(abs(sent[k] - requested[k]) > 0.2 for k in requested):
                        raise RuntimeError(f"{label}: LeRobot clipped the requested target; stopping.")
                    if fraction >= 1.0:
                        return actual
                    next_tick += 0.05
                    # Do not burst through missed ticks if communication is slow.
                    next_tick = max(next_tick, time.monotonic())
                    time.sleep(max(0.0, next_tick - time.monotonic()))

            run_phase("initial hold", start, start, 3.0)
            input("If holding steadily, clear your hands and press Enter to move: ")
            # Measure after removing support, then use that actual pose as the
            # origin. This separates release sag from response to our test move.
            current = run_phase("unsupported baseline", start, start, 1.0)
            if any(abs(current[k] - start[k]) > 2.0 for k in start):
                raise RuntimeError("Starting pose drifted more than 2 units; stopping.")
            start = current
            end = dict(start)
            end[joint_key] += delta
            if (not 0 <= end[joint_key] <= 100 if args.joint == "gripper"
                    else abs(end[joint_key]) > half_range_degrees):
                raise RuntimeError("Rebased target is outside the calibrated range.")
            print(f"Unsupported baseline: {start[joint_key]:.3f}; outward target: {end[joint_key]:.3f}")
            run_phase("outward", start, end, movement_duration)
            actual = run_phase("outward hold", end, end, 2.0)
            validate_endpoint(actual, end, start, joint_key, args.degrees, outward=True)
            run_phase("return", end, start, movement_duration)
            actual = run_phase("return hold", start, start, 2.0)
            validate_endpoint(actual, start, end, joint_key, args.degrees, outward=False)
            print("API movement and return PASSED.")
        input("Support the arm, then press Enter to disable torque: ")
    except KeyboardInterrupt:
        print("\nInterrupted; no automatic return.")
    finally:
        try:
            if robot.is_connected:
                try:
                    if original_joint_p is not None:
                        robot.bus.disable_torque()
                        robot.bus.write("P_Coefficient", args.joint, original_joint_p, normalize=False)
                        restored_p = robot.bus.read("P_Coefficient", args.joint, normalize=False)
                        if restored_p != original_joint_p:
                            raise RuntimeError("Could not verify restoration of joint P gain.")
                        print(f"Restored {args.joint} P to {restored_p}.")
                finally:
                    robot.disconnect()
                    print("Disconnected; torque disabled.")
        finally:
            if log_path.exists():
                print(f"Diagnostic log: {log_path.resolve()}")


if __name__ == "__main__":
    main()
