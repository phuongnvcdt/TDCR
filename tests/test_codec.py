"""Unit tests for protocol codec."""
import pytest
from odri_can.protocol.codec import (
    float_to_q24, q24_to_float,
    pack_two_q24, unpack_two_q24,
    pack_command, unpack_two_int32,
    parse_status_byte, parse_encoder_index,
)
from odri_can.protocol.messages import Command, ErrorCode


class TestQ24:
    def test_zero(self):
        assert float_to_q24(0.0) == 0
        assert q24_to_float(0) == 0.0

    def test_one(self):
        assert float_to_q24(1.0) == 1 << 24

    def test_roundtrip(self):
        for v in [0.0, 0.5, -0.5, 1.5, -2.7, 10.0, -10.0]:
            roundtrip = q24_to_float(float_to_q24(v))
            assert abs(roundtrip - v) < 1e-6, f"roundtrip failed for {v}"

    def test_clamping(self):
        # Very large value should clamp, not overflow
        huge = 1e10
        result = float_to_q24(huge)
        assert result == 2**31 - 1


class TestPackTwoQ24:
    def test_size(self):
        assert len(pack_two_q24(0.0, 0.0)) == 8

    def test_roundtrip(self):
        data = pack_two_q24(1.5, -2.25)
        assert len(data) == 8
        u_a, u_b = unpack_two_q24(data)
        assert abs(u_a - 1.5) < 1e-6
        assert abs(u_b - -2.25) < 1e-6

    def test_invalid_length(self):
        with pytest.raises(ValueError):
            unpack_two_q24(b"\x00" * 4)


class TestPackCommand:
    def test_enable_sys(self):
        # ENABLE_SYS = 0x01, value = 1
        # MDL = 1 (int32 LE), MDH = 0x01 (int32 LE)
        data = pack_command(int(Command.ENABLE_SYS), 1)
        assert len(data) == 8
        value, code = unpack_two_int32(data)
        assert value == 1
        assert code == 0x01

    def test_set_timeout(self):
        # SET_CAN_RECV_TIMEOUT = 0x1E, value = 100
        data = pack_command(int(Command.SET_CAN_RECV_TIMEOUT), 100)
        value, code = unpack_two_int32(data)
        assert value == 100
        assert code == 0x1E


class TestStatusByte:
    def test_all_zero(self):
        flags = parse_status_byte(0)
        assert flags["sys_enabled"] is False
        assert flags["mtr1_enabled"] is False
        assert flags["mtr1_ready"] is False
        assert flags["mtr2_enabled"] is False
        assert flags["mtr2_ready"] is False
        assert flags["error_code"] == ErrorCode.NO_ERROR

    def test_all_enabled(self):
        # bits 0..4 set, error code 0
        byte = 0b00011111
        flags = parse_status_byte(byte)
        assert all([flags["sys_enabled"], flags["mtr1_enabled"], flags["mtr1_ready"],
                    flags["mtr2_enabled"], flags["mtr2_ready"]])
        assert flags["error_code"] == ErrorCode.NO_ERROR

    def test_error_code(self):
        # error code 2 (CAN timeout) in bits 5-7
        byte = (2 << 5) | 0b00000001  # also sys enabled
        flags = parse_status_byte(byte)
        assert flags["error_code"] == ErrorCode.CAN_RECV_TIMEOUT
        assert flags["sys_enabled"] is True


class TestEncoderIndex:
    def test_basic(self):
        import struct
        # Position = 0.5 rev → Q24 raw, then 1 byte motor_id
        raw_pos = float_to_q24(0.5)
        data = struct.pack("<i", raw_pos) + bytes([1])
        pos, motor_id = parse_encoder_index(data)
        assert abs(pos - 0.5) < 1e-6
        assert motor_id == 1
