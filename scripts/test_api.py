"""Small joint trajectory using LeRobot's observation and action APIs."""

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


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
    parser.add_argument("--joint", choices=("elbow_flex", "shoulder_pan"), default="elbow_flex")
    parser.add_argument("--degrees", type=float, default=2.0, help="Movement size; base supports up to 10 degrees.")
    parser.add_argument(
        "--speed", type=float, default=2.0 / 3.0,
        help="Commanded trajectory speed in degrees/second (default: 0.667; base maximum: 5).",
    )
    parser.add_argument(
        "--direction", choices=("toward-zero", "away-from-zero"),
        default="toward-zero", help="Initial direction of the joint test (may cross zero).",
    )
    parser.add_argument(
        "--diagnostics", action="store_true",
        help="Read and log the selected joint's raw voltage, current, load, and status.",
    )
    parser.add_argument(
        "--elbow-p", type=int, choices=(16, 20),
        help="Optional elbow-only proportional gain experiment; restored at shutdown.",
    )
    args = parser.parse_args()
    if args.elbow_p is not None and args.joint != "elbow_flex":
        parser.error("--elbow-p is only valid with --joint elbow_flex")
    maximum = 10.0 if args.joint == "shoulder_pan" else 2.0
    if not math.isfinite(args.degrees) or not 0 < args.degrees <= maximum:
        parser.error(f"--degrees must be greater than zero and at most {maximum:g} for {args.joint}")
    maximum_speed = 5.0 if args.joint == "shoulder_pan" else 2.0 / 3.0
    if not math.isfinite(args.speed) or not 0 < args.speed <= maximum_speed:
        parser.error(f"--speed must be greater than zero and at most {maximum_speed:g} for {args.joint}")
    movement_duration = args.degrees / args.speed
    robot = SO101Follower(SO101FollowerConfig(
        port=args.port, id=args.robot_id, use_degrees=True,
        max_relative_target=3.0,
    ))
    if not robot.calibration:
        raise RuntimeError("No saved calibration found. Check --robot-id.")

    original_elbow_p = None
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
        print(f"All joints hold; only {args.joint} moves {args.degrees:g} degrees initially {direction_label} and back.")
        print(f"Commanded speed: {args.speed:g} degrees/s; each movement takes approximately {movement_duration:g} seconds.")
        print("Ctrl+C/errors disable torque. This trajectory is not collision checked.")
        if args.elbow_p is not None:
            print(f"Elbow-only P gain experiment: {args.elbow_p}; I/D and other joints unchanged.")
        if input("Type MOVE to continue: ").strip() != "MOVE":
            return

        if args.elbow_p is not None:
            # The public robot configuration only supports a shared gain.
            # This isolated configuration change does not command movement.
            robot.bus.disable_torque()
            original_elbow_p = robot.bus.read("P_Coefficient", "elbow_flex", normalize=False)
            robot.bus.write("P_Coefficient", "elbow_flex", args.elbow_p, normalize=False)
            stored_p = robot.bus.read("P_Coefficient", "elbow_flex", normalize=False)
            if stored_p != args.elbow_p:
                raise RuntimeError("Elbow P gain readback mismatch.")
            print(f"Elbow P: {original_elbow_p} -> {stored_p}; restored on normal/error shutdown.")

        start = read_pose(robot)
        end = dict(start)
        joint_key = f"{args.joint}.pos"
        delta = -args.degrees if start[joint_key] >= 0 else args.degrees
        if args.direction == "away-from-zero":
            delta = -delta
        end[joint_key] += delta
        calibration = robot.calibration[args.joint]
        half_range_degrees = (calibration.range_max - calibration.range_min) * 180 / 4095
        if abs(end[joint_key]) > half_range_degrees:
            raise RuntimeError("Joint target is outside its calibrated range; stopping.")
        print(f"{args.joint}: {start[joint_key]:.2f} -> {end[joint_key]:.2f} -> {start[joint_key]:.2f} degrees")
        log_path.parent.mkdir(exist_ok=True)
        began = time.monotonic()
        with log_path.open("x", encoding="utf-8", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(["elapsed_s", "phase", "joint", "requested", "sent", "actual", "error",
                             "tested_joint_voltage_raw", "tested_joint_current_raw", "tested_joint_load_raw", "tested_joint_status_raw",
                             "elbow_p_override"])

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
                    telemetry = ["", "", "", ""]
                    if args.diagnostics:
                        # These are diagnostic reads only; actions still use send_action().
                        telemetry = [robot.bus.read(register, args.joint, normalize=False)
                                     for register in ("Present_Voltage", "Present_Current", "Present_Load", "Status")]
                    for key in requested:
                        writer.writerow([round(now - began, 3), label, key, requested[key],
                                         sent[key], actual[key], actual[key] - requested[key], *telemetry,
                                         args.elbow_p if args.elbow_p is not None else ""])
                    output.flush()
                    if now >= next_print:
                        diagnostics = (
                            f", voltage_raw={telemetry[0]}, current_raw={telemetry[1]}, "
                            f"load_raw={telemetry[2]}, status_raw={telemetry[3]}"
                            if args.diagnostics else ""
                        )
                        print(f"{label}: target={requested[joint_key]:.2f}, sent={sent[joint_key]:.2f}, "
                              f"actual={actual[joint_key]:.2f}, error={actual[joint_key] - requested[joint_key]:+.3f}"
                              f"{diagnostics}",
                              flush=True)
                        next_print = now + 0.5
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
            current = read_pose(robot)
            if any(abs(current[k] - start[k]) > 2.0 for k in start):
                raise RuntimeError("Starting pose drifted more than 2 units; stopping.")
            run_phase("outward", start, end, movement_duration)
            actual = run_phase("outward hold", end, end, 2.0)
            if any(abs(actual[k] - end[k]) > 2.0 for k in end):
                raise RuntimeError("Outward target not reached within 2-unit tolerance.")
            run_phase("return", end, start, movement_duration)
            actual = run_phase("return hold", start, start, 2.0)
            if any(abs(actual[k] - start[k]) > 2.0 for k in start):
                raise RuntimeError("Return target not reached within 2-unit tolerance.")
            print("API movement and return PASSED.")
        input("Support the arm, then press Enter to disable torque: ")
    except KeyboardInterrupt:
        print("\nInterrupted; no automatic return.")
    finally:
        try:
            if robot.is_connected:
                try:
                    if original_elbow_p is not None:
                        robot.bus.disable_torque()
                        robot.bus.write("P_Coefficient", "elbow_flex", original_elbow_p, normalize=False)
                        restored_p = robot.bus.read("P_Coefficient", "elbow_flex", normalize=False)
                        if restored_p != original_elbow_p:
                            raise RuntimeError("Could not verify restoration of elbow P gain.")
                        print(f"Restored elbow P to {restored_p}.")
                finally:
                    robot.disconnect()
                    print("Disconnected; torque disabled.")
        finally:
            if log_path.exists():
                print(f"Diagnostic log: {log_path.resolve()}")


if __name__ == "__main__":
    main()
