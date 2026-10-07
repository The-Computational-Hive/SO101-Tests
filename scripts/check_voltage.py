"""Read STS3215 voltage registers without changing motor settings or torque."""

import argparse

import scservo_sdk as scs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()
    port = scs.PortHandler(args.port)
    packet = scs.PacketHandler(0)
    if not port.openPort():
        raise RuntimeError(f"Cannot open {args.port}")
    try:
        if not port.setBaudRate(args.baudrate):
            raise RuntimeError("Cannot set serial baud rate")
        print("Read-only diagnostic. Values are raw register counts.")
        names = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                 "wrist_flex", "wrist_roll", "gripper"]
        for motor_id, name in enumerate(names, 1):
            print(f"\nID {motor_id}: {name}")
            for label, address in [("Present_Voltage", 62),
                                   ("Min_Voltage_Limit", 15),
                                   ("Max_Voltage_Limit", 14),
                                   ("Torque_Enable", 40)]:
                value, result, error = packet.read1ByteTxRx(port, motor_id, address)
                if result != scs.COMM_SUCCESS:
                    print(f"  {label}: communication failed: {packet.getTxRxResult(result)}")
                    continue
                status = packet.getRxPacketError(error) if error else "OK"
                print(f"  {label}: {value}; status={status}")
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
