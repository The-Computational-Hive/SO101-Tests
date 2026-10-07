"""Diagnose shoulder-lift torque engagement without requesting movement."""

import argparse
import math
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


def snapshot(robot, label):
    bus = robot.bus
    motor = "shoulder_lift"
    actual = bus.read("Present_Position", motor)
    goal = bus.read("Goal_Position", motor)
    torque = bus.read("Torque_Enable", motor, normalize=False)
    voltage = bus.read("Present_Voltage", motor, normalize=False)
    print(
        f"{label}: actual={actual:.2f} deg, target={goal:.2f} deg, "
        f"torque={torque}, voltage_register={voltage}", flush=True,
    )
    return actual, goal, torque


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--robot-id", required=True)
    args = parser.parse_args()
    robot = SO101Follower(SO101FollowerConfig(
        port=args.port, id=args.robot_id, use_degrees=True,
    ))
    if not robot.calibration:
        raise RuntimeError("No calibration found. Check --robot-id.")

    try:
        robot.connect(calibrate=False)
        robot.bus.disable_torque()
        _, _, torque = snapshot(robot, "Immediately after disabling")
        if torque != 0:
            raise RuntimeError("Shoulder-lift torque did not disable; no target was written.")
        if not robot.is_calibrated:
            raise RuntimeError("Calibration does not match the motors.")
        print("Support the upper arm and forearm. Only shoulder_lift will be enabled for 3 seconds.")
        print("Writing a hold target may engage torque on this firmware. Ctrl+C stops.")
        if input("Type HOLD to continue: ").strip() != "HOLD":
            return

        _, _, torque = snapshot(robot, "Before writing hold target")
        if torque != 0:
            raise RuntimeError("Torque became enabled before target write; stopping.")
        start = robot.bus.read("Present_Position", "shoulder_lift")
        if not math.isfinite(start):
            raise RuntimeError("Invalid starting position.")
        robot.bus.write("Goal_Position", "shoulder_lift", start)
        actual, goal, torque = snapshot(robot, "After writing hold target")
        if abs(goal - start) > 0.2 or abs(actual - start) > 1:
            raise RuntimeError("Starting target/position check failed; stopping.")

        if torque == 0:
            robot.bus.enable_torque("shoulder_lift")
        elif torque == 1:
            print("Goal write enabled torque; skipping the explicit enable call.")
        else:
            raise RuntimeError(f"Unexpected torque state: {torque}")

        for name in robot.bus.motors:
            if name != "shoulder_lift" and robot.bus.read("Torque_Enable", name, normalize=False) != 0:
                raise RuntimeError(f"Unexpected torque enabled on {name}; stopping.")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            actual, goal, torque = snapshot(robot, "Holding")
            if not all(math.isfinite(v) for v in (actual, goal)):
                raise RuntimeError("Invalid feedback.")
            if abs(goal - start) > 0.2:
                raise RuntimeError("Goal changed after torque engagement.")
            if abs(actual - start) > 2.0:
                raise RuntimeError("Shoulder lift moved more than 2 degrees during hold.")
            if torque != 1:
                raise RuntimeError("Motor torque is not enabled.")
            time.sleep(0.1)
        print("Hold check passed.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if robot.bus.is_connected:
            robot.disconnect()
            print("Disconnected; torque disabled.")


if __name__ == "__main__":
    main()
