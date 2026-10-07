import argparse
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


def main():
    parser = argparse.ArgumentParser(description="Read SO101 joint positions.")
    parser.add_argument("--port", required=True, help="Controller port, e.g. COM5")
    parser.add_argument("--robot-id", required=True, help="ID used during calibration")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    if args.samples < 1 or args.interval <= 0:
        parser.error("--samples must be >= 1 and --interval must be > 0")

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.robot_id,
            use_degrees=True,
        )
    )

    # Check before connecting to avoid using an incorrect calibration ID.
    if not robot.calibration:
        raise RuntimeError(
            f"No saved calibration found for '{args.robot_id}'. "
            "Use the same robot ID you used during calibration."
        )

    try:
        robot.connect(calibrate=False)

        if not robot.is_calibrated:
            raise RuntimeError("Saved calibration does not match the motors.")

        print("Connected. Calibration matches.")
        print("Joint positions are degrees; gripper is normalized to 0–100.")

        for sample in range(args.samples):
            observation = robot.get_observation()
            positions = " | ".join(
                f"{name.removesuffix('.pos')}: {value:.2f}"
                for name, value in observation.items()
                if name.endswith(".pos")
            )
            print(f"{sample + 1:03d} | {positions}")

            if sample + 1 < args.samples:
                time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        # Also clean up if connect() failed after opening the motor bus.
        if robot.bus.is_connected:
            robot.disconnect()
            print("Disconnected.")


if __name__ == "__main__":
    main()