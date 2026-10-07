"""Read STS3215 settings without configuring motors or enabling torque."""

import argparse

import scservo_sdk as scs
from lerobot.motors.feetech.tables import STS_SMS_SERIES_CONTROL_TABLE


FIELDS = (
    "Firmware_Major_Version", "Firmware_Minor_Version", "Operating_Mode",
    "Torque_Enable", "P_Coefficient", "I_Coefficient", "D_Coefficient",
    "CW_Dead_Zone", "CCW_Dead_Zone", "Minimum_Startup_Force",
    "Max_Torque_Limit", "Torque_Limit", "Protection_Current",
    "Protective_Torque", "Overload_Torque", "Protection_Time",
    "Goal_Time", "Goal_Velocity", "Acceleration",
    "Present_Voltage", "Min_Voltage_Limit", "Max_Voltage_Limit",
    "Present_Temperature", "Present_Load", "Present_Current", "Status",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--motor-id", type=int, default=3, choices=range(1, 7))
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()
    port = scs.PortHandler(args.port)
    packet = scs.PacketHandler(0)
    if not port.openPort():
        raise RuntimeError(f"Cannot open {args.port}")
    try:
        if not port.setBaudRate(args.baudrate):
            raise RuntimeError("Cannot set serial baud rate")
        print(f"Motor ID {args.motor_id}: read-only settings; all values are raw.")
        for name in FIELDS:
            address, size = STS_SMS_SERIES_CONTROL_TABLE[name]
            reader = packet.read1ByteTxRx if size == 1 else packet.read2ByteTxRx
            value, result, error = reader(port, args.motor_id, address)
            if result != scs.COMM_SUCCESS:
                print(f"{name}: communication failed: {packet.getTxRxResult(result)}")
                continue
            status = packet.getRxPacketError(error) if error else "OK"
            print(f"{name}: {value}; status={status}")
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
