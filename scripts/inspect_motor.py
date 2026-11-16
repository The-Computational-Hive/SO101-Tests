"""Read STS3215 settings without configuring motors or enabling torque."""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import scservo_sdk as scs
from lerobot.motors.feetech.tables import STS_SMS_SERIES_CONTROL_TABLE


FIELDS = (
    "Model_Number", "Firmware_Major_Version", "Firmware_Minor_Version", "Operating_Mode",
    "Torque_Enable", "P_Coefficient", "I_Coefficient", "D_Coefficient",
    "CW_Dead_Zone", "CCW_Dead_Zone", "Minimum_Startup_Force",
    "Max_Torque_Limit", "Torque_Limit", "Protection_Current",
    "Protective_Torque", "Overload_Torque", "Protection_Time",
    "Over_Current_Protection_Time", "Unloading_Condition", "Phase", "Lock",
    "Goal_Time", "Goal_Velocity", "Acceleration",
    "Goal_Position", "Present_Position", "Moving",
    "Present_Voltage", "Min_Voltage_Limit", "Max_Voltage_Limit",
    "Present_Temperature", "Present_Load", "Present_Current", "Status",
)

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def read_motor(packet, port, motor_id):
    """Use only register reads; retain packet error flags alongside valid data."""
    registers = {}
    for name in FIELDS:
        address, size = STS_SMS_SERIES_CONTROL_TABLE[name]
        reader = packet.read1ByteTxRx if size == 1 else packet.read2ByteTxRx
        value, result, error = reader(port, motor_id, address)
        registers[name] = {
            "raw": value if result == scs.COMM_SUCCESS else None,
            "communication": "OK" if result == scs.COMM_SUCCESS else packet.getTxRxResult(result),
            "packet_error": error if result == scs.COMM_SUCCESS else None,
            "packet_status": (packet.getRxPacketError(error) if error else "OK")
                             if result == scs.COMM_SUCCESS else "unavailable",
        }
    return registers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--motor-id", type=int, default=3, choices=range(1, 7))
    selection.add_argument("--all", action="store_true", help="Audit all six motors without configuring them.")
    parser.add_argument("--output", type=Path, help="Save full raw JSON and print a concise summary.")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()
    port = scs.PortHandler(args.port)
    packet = scs.PacketHandler(0)
    if not port.openPort():
        raise RuntimeError(f"Cannot open {args.port}")
    try:
        if not port.setBaudRate(args.baudrate):
            raise RuntimeError("Cannot set serial baud rate")
        report = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "port": args.port,
                  "baudrate": args.baudrate, "units": "raw register counts; positions are not calibrated degrees",
                  "motors": {}}
        for motor_id in (range(1, 7) if args.all else [args.motor_id]):
            registers = read_motor(packet, port, motor_id)
            joint = JOINTS[motor_id - 1]
            report["motors"][joint] = {"id": motor_id, "registers": registers}
            if args.output is None:
                print(f"\nMotor ID {motor_id}: {joint}; read-only settings; all values are raw.")
                for name, entry in registers.items():
                    print(f"{name}: {entry['raw']}; comm={entry['communication']}; status={entry['packet_status']}")
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print("Read-only audit. No configuration, goal, or torque writes were sent.")
            print("All values below are raw register counts.")
            for joint, motor in report["motors"].items():
                registers = motor["registers"]
                names = ("Torque_Enable", "P_Coefficient", "I_Coefficient", "D_Coefficient",
                         "Max_Torque_Limit", "Torque_Limit", "Protection_Current",
                         "Protective_Torque", "Overload_Torque", "Protection_Time",
                         "Present_Voltage", "Present_Current", "Present_Load", "Status")
                print(f"\nID {motor['id']}: {joint}")
                print("  " + ", ".join(f"{name}={registers[name]['raw']}" for name in names))
                for name, entry in registers.items():
                    if entry["communication"] != "OK" or entry["packet_error"]:
                        print(f"  {name}: comm={entry['communication']}; status={entry['packet_status']}")
            print(f"\nFull audit: {args.output.resolve()}")
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
