"""Move gradually to the midpoint of each joint's calibrated range."""

import argparse
import math
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


def positions(robot):
    return robot.bus.sync_read("Present_Position")


def engage_hold(robot):
    goals = {}
    # Each target write can enable torque: sample immediately before each write.
    for name in robot.bus.motors:
        current = robot.bus.read("Present_Position", name)
        if not math.isfinite(current):
            raise RuntimeError(f"Invalid position for {name}")
        robot.bus.write("Goal_Position", name, current)
        stored = robot.bus.read("Goal_Position", name)
        if not math.isfinite(stored) or abs(stored - current) > 0.2:
            raise RuntimeError(f"Holding target readback failed for {name}")
        torque = robot.bus.read("Torque_Enable", name, normalize=False)
        if torque == 0:
            robot.bus.enable_torque(name)
        elif torque != 1:
            raise RuntimeError(f"Unexpected torque state for {name}: {torque}")
        goals[name] = current
        wait_for_step(robot, goals, f"Engaging {name}")
        print(f"Holding {name} at {current:.2f}")
    return goals


def verify_hold(robot, goals):
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        actual = positions(robot)
        stored = robot.bus.sync_read("Goal_Position")
        torque = robot.bus.sync_read("Torque_Enable", normalize=False)
        for name, target in goals.items():
            if not all(math.isfinite(v) for v in (actual[name], stored[name])):
                raise RuntimeError(f"Invalid feedback for {name}")
            if abs(actual[name] - target) > 2.0:
                raise RuntimeError(f"{name} moved during hold: target={target:.2f}, actual={actual[name]:.2f}")
            if abs(stored[name] - target) > 0.2 or torque[name] != 1:
                raise RuntimeError(f"Hold target or torque changed for {name}")
        time.sleep(0.1)
    print("All-joint hold check PASSED.")


def wait_for_step(robot, goals, label):
    deadline = time.monotonic() + 3.0
    stable = 0
    # Body-joint tolerance matches the verified hold test. Gripper uses its
    # normalized 0-100 scale, so retain a separate one-unit tolerance.
    tolerances = {n: 1.0 if n == "gripper" else 2.0 for n in goals}
    while time.monotonic() < deadline:
        actual = positions(robot)
        if not all(math.isfinite(v) for v in actual.values()):
            raise RuntimeError(f"{label}: invalid position reading")
        errors = {n: actual[n] - goals[n] for n in goals}
        details = "; ".join(
            f"{n}: target={goals[n]:.2f}, actual={actual[n]:.2f}, error={errors[n]:+.2f}"
            for n in goals if abs(errors[n]) > tolerances[n]
        )
        if any(abs(error) > 5.0 for error in errors.values()):
            raise RuntimeError(f"{label}: excessive tracking error: {details}")
        stable = stable + 1 if all(abs(errors[n]) <= tolerances[n] for n in goals) else 0
        if stable >= 5:
            return actual
        time.sleep(0.1)
    raise RuntimeError(f"{label}: did not settle within 3 seconds: {details}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--robot-id", required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--hold-only", action="store_true", help="Hold current positions only (default).")
    mode.add_argument("--move-to-middle", action="store_true", help="Move after the all-joint hold passes.")
    args = parser.parse_args()
    robot = SO101Follower(SO101FollowerConfig(
        port=args.port, id=args.robot_id, use_degrees=True,
    ))
    if not robot.calibration:
        raise RuntimeError("No saved calibration found. Check --robot-id.")

    try:
        robot.connect(calibrate=False)
        robot.bus.disable_torque()
        if not robot.is_calibrated:
            raise RuntimeError("Saved calibration does not match the motors.")

        preview = positions(robot)
        targets = {name: 50.0 if name == "gripper" else 0.0 for name in preview}
        print("Body joints: degrees; gripper: calibrated 0-100 scale.")
        for name, value in preview.items():
            if args.move_to_middle:
                print(f"{name:16s}: {value:8.2f} -> {targets[name]:8.2f}")
            else:
                print(f"{name:16s}: current={value:8.2f}")
        print("All six motors will be powered. Secure the base and clear the path.")
        if args.move_to_middle:
            print("The calibrated midpoint pose/path is not checked for collisions.")
        else:
            print("HOLD ONLY: no move toward the midpoint will be requested.")
        confirmation = "MOVE" if args.move_to_middle else "HOLD"
        if input(f"Type {confirmation} to begin: ").strip() != confirmation:
            return

        start = engage_hold(robot)
        verify_hold(robot, start)
        if not args.move_to_middle:
            input("Support the arm, then press Enter to disable torque: ")
            return
        steps = max(1, math.ceil(max(abs(targets[n] - start[n]) for n in start) / 0.5))
        print(f"Moving in {steps} steps, at most 0.5 degrees per body joint per step.")
        for step in range(1, steps + 1):
            goals = {n: start[n] + (targets[n] - start[n]) * step / steps for n in start}
            robot.bus.sync_write("Goal_Position", goals)
            actual = wait_for_step(robot, goals, f"Step {step}/{steps}")
            print(f"Step {step}/{steps} verified")

        print("Middle pose verified:", {n: round(v, 2) for n, v in actual.items()})
        input("Torque remains ON. Support the arm, then press Enter to disable torque: ")
    except KeyboardInterrupt:
        print("\nInterrupted; disabling torque without further movement.")
    finally:
        if robot.bus.is_connected:
            robot.disconnect()
            print("Disconnected; torque disabled.")


if __name__ == "__main__":
    main()
