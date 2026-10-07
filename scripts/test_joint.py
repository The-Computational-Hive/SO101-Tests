import argparse
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


def wait_for_position(robot, target, tolerance=1.0, timeout=3.0):
    deadline = time.monotonic() + timeout
    stable_reads = 0
    actual = robot.bus.read("Present_Position", "shoulder_pan")

    while time.monotonic() < deadline:
        actual = robot.bus.read("Present_Position", "shoulder_pan")
        if abs(actual - target) <= tolerance:
            stable_reads += 1
            if stable_reads >= 5:
                return actual
        else:
            stable_reads = 0
        time.sleep(0.1)

    raise RuntimeError(
        f"Target {target:.2f} degrees not reached within {timeout:.1f}s; "
        f"last position {actual:.2f} degrees."
    )


def move_slowly(robot, start, end):
    for step in range(1, 41):
        target = start + (end - start) * step / 40
        robot.bus.write("Goal_Position", "shoulder_pan", target)
        actual = robot.bus.read("Present_Position", "shoulder_pan")
        if abs(actual - target) > 5:
            raise RuntimeError("Position differs from target by more than 5 degrees.")
        time.sleep(0.05)

    return wait_for_position(robot, end)


def main():
    parser = argparse.ArgumentParser(description="Small SO101 base rotation test.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--robot-id", required=True)
    args = parser.parse_args()

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.robot_id,
            use_degrees=True,
        )
    )

    if not robot.calibration:
        raise RuntimeError("No saved calibration found. Check --robot-id.")

    try:
        robot.connect(calibrate=False)
        robot.bus.disable_torque()

        if not robot.is_calibrated:
            raise RuntimeError("Calibration does not match the motors.")

        print("Only shoulder_pan will be powered.")
        if input("Clear the workspace, then type MOVE to continue: ").strip() != "MOVE":
            return

        start = robot.bus.read("Present_Position", "shoulder_pan")
        # Move toward zero to avoid crossing the angular wrap boundary.
        target = start - 2 if start >= 0 else start + 2

        # Replace any old target before enabling torque.
        robot.bus.write("Goal_Position", "shoulder_pan", start)
        robot.bus.enable_torque("shoulder_pan")

        print(f"Moving from {start:.2f}° to {target:.2f}°")
        actual = move_slowly(robot, start, target)
        print(f"Outward target verified: {actual:.2f} degrees")

        final = move_slowly(robot, actual, start)
        print(
            f"Return verified: {final:.2f} degrees "
            f"(error: {abs(final - start):.2f} degrees)"
        )

    except KeyboardInterrupt:
        print("\nInterrupted; disabling torque without returning.")
    finally:
        if robot.bus.is_connected:
            robot.disconnect()
            print("Disconnected; torque disabled.")


if __name__ == "__main__":
    main()
