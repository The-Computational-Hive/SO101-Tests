"""Log a small elbow move while the remaining joints hold their positions."""

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from move_to_middle import engage_hold, positions, verify_hold


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

    log_path = Path("logs") / f"elbow_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
    try:
        robot.connect(calibrate=False)
        robot.bus.disable_torque()
        if not robot.is_calibrated:
            raise RuntimeError("Calibration does not match the motors.")
        print("Secure the base; clear room for a 2-degree elbow move and return.")
        print("Support the arm during startup. All joints will hold under power.")
        if input("Type MOVE to continue: ").strip() != "MOVE":
            return
        goals = engage_hold(robot)
        verify_hold(robot, goals)
        input("If the arm holds steadily, clear your hands and press Enter to test: ")
        start = goals["elbow_flex"]
        delta = -2.0 if start >= 0 else 2.0
        log_path.parent.mkdir(exist_ok=True)
        began = time.monotonic()
        with log_path.open("x", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(["elapsed_s", "phase", "joint", "command", "stored_goal",
                             "actual", "error", "elbow_voltage_register", "torque"])

            def sample(phase):
                actual = positions(robot)
                stored = robot.bus.sync_read("Goal_Position")
                torque = robot.bus.sync_read("Torque_Enable", normalize=False)
                voltage = robot.bus.read("Present_Voltage", "elbow_flex", normalize=False)
                for name, target in goals.items():
                    writer.writerow([round(time.monotonic() - began, 3), phase, name,
                                     target, stored[name], actual[name], actual[name] - target,
                                     voltage, torque[name]])
                output.flush()
                print(f"{phase}: elbow target={goals['elbow_flex']:.2f}, "
                      f"actual={actual['elbow_flex']:.2f}, "
                      f"error={actual['elbow_flex'] - goals['elbow_flex']:+.3f}, "
                      f"voltage_raw={voltage}", flush=True)
                for name, target in goals.items():
                    if not all(math.isfinite(v) for v in (stored[name], actual[name])):
                        raise RuntimeError(f"Invalid feedback for {name}")
                    if torque[name] != 1 or abs(stored[name] - target) > 0.2:
                        raise RuntimeError(f"Target/torque mismatch on {name}")
                    if abs(actual[name] - target) > 5.0:
                        raise RuntimeError(f"Tracking error exceeds 5 on {name}")
                return actual

            for phase, offsets in [
                ("outward", [delta * step / 4 for step in range(1, 5)]),
                ("return", [delta * (1 - step / 4) for step in range(1, 5)]),
            ]:
                for offset in offsets:
                    goals["elbow_flex"] = start + offset
                    robot.bus.write("Goal_Position", "elbow_flex", goals["elbow_flex"])
                    # Keep the target fixed for 1 second and record its response.
                    for _ in range(10):
                        time.sleep(0.1)
                        actual = sample(phase)
                if any(abs(actual[n] - goals[n]) > 2.0 for n in goals):
                    raise RuntimeError(f"{phase}: target not reached within 2-unit tolerance")
            print("Elbow movement and return PASSED.")
        input("Support the arm, then press Enter to disable torque: ")
    except KeyboardInterrupt:
        print("\nInterrupted; no automatic return.")
    finally:
        try:
            if robot.bus.is_connected:
                robot.disconnect()
                print("Disconnected; torque disabled.")
        finally:
            if log_path.exists():
                print(f"Diagnostic log: {log_path.resolve()}")


if __name__ == "__main__":
    main()
