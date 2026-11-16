"""Ensure the torque audit remains read-only and retains fault information."""

import unittest
from unittest.mock import MagicMock

import scservo_sdk as scs

from scripts.inspect_motor import FIELDS, read_motor
from lerobot.motors.feetech.tables import STS_SMS_SERIES_CONTROL_TABLE


class MotorAuditChecks(unittest.TestCase):
    def test_register_audit_uses_only_reads_of_correct_width(self):
        packet = MagicMock(spec=["read1ByteTxRx", "read2ByteTxRx", "getTxRxResult", "getRxPacketError"])
        packet.read1ByteTxRx.return_value = (54, scs.COMM_SUCCESS, 0)
        packet.read2ByteTxRx.return_value = (1000, scs.COMM_SUCCESS, 0)
        port = object()
        data = read_motor(packet, port, 3)
        self.assertEqual(set(data), set(FIELDS))
        self.assertEqual(packet.read1ByteTxRx.call_count + packet.read2ByteTxRx.call_count, len(FIELDS))
        for name in FIELDS:
            address, width = STS_SMS_SERIES_CONTROL_TABLE[name]
            reader = packet.read1ByteTxRx if width == 1 else packet.read2ByteTxRx
            reader.assert_any_call(port, 3, address)
        self.assertEqual(data["Torque_Limit"]["raw"], 1000)

    def test_faulted_data_and_failed_communications_remain_distinct(self):
        packet = MagicMock()
        packet.read1ByteTxRx.return_value = (54, scs.COMM_SUCCESS, 1)
        packet.read2ByteTxRx.return_value = (0, -1, 0)
        packet.getRxPacketError.return_value = "input voltage error"
        packet.getTxRxResult.return_value = "communication failed"
        data = read_motor(packet, object(), 6)
        self.assertEqual(data["Present_Voltage"]["raw"], 54)
        self.assertEqual(data["Present_Voltage"]["packet_error"], 1)
        self.assertIsNone(data["Torque_Limit"]["raw"])
        self.assertEqual(data["Torque_Limit"]["communication"], "communication failed")


if __name__ == "__main__":
    unittest.main()
