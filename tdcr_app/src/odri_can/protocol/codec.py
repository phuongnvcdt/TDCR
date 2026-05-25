"""
Codec utilities for ODRI CAN protocol.

Q24 fixed-point format:
    int_value = round(float_value * 2^24)
    float_value = int_value / 2^24

Frame data layout (8 bytes total):
    bytes[0:4] = MDL (low 4 bytes, big-endian int32)
    bytes[4:8] = MDH (high 4 bytes, big-endian int32)
"""
import struct
from .messages import Q24_SCALE, StatusBit, ErrorCode


def float_to_q24(value: float) -> int:
    """Convert float → Q24 signed int32. Clamp to int32 range."""
    raw = round(value * Q24_SCALE)
    return max(-2**31, min(2**31 - 1, raw))


def q24_to_float(raw: int) -> float:
    """Convert Q24 signed int32 → float."""
    return raw / Q24_SCALE


def pack_two_q24(value_mdl: float, value_mdh: float) -> bytes:
    """Pack two floats as Q24 into 8-byte CAN payload (MDL, MDH).

    Uses big-endian signed int32 — confirmed by ODRI's TI C2000 firmware
    convention. Verify on hardware if behavior seems off.
    """
    return struct.pack(">ii", float_to_q24(value_mdl), float_to_q24(value_mdh))


def unpack_two_q24(data: bytes) -> tuple[float, float]:
    """Unpack 8-byte CAN payload → (MDL_float, MDH_float)."""
    if len(data) != 8:
        raise ValueError(f"Expected 8 bytes, got {len(data)}")
    mdl, mdh = struct.unpack(">ii", data)
    return q24_to_float(mdl), q24_to_float(mdh)


def pack_command(command_code: int, value: int) -> bytes:
    """Pack a command frame: MDL=value (int32), MDH=command_code (int32)."""
    return struct.pack(">ii", value, command_code)


def unpack_two_int32(data: bytes) -> tuple[int, int]:
    """Unpack 8-byte payload → (MDL_int, MDH_int) as raw int32."""
    if len(data) != 8:
        raise ValueError(f"Expected 8 bytes, got {len(data)}")
    return struct.unpack(">ii", data)


# ---- Status byte parsing ----
def parse_status_byte(byte: int) -> dict:
    """Parse status byte into a dict of flags and error code."""
    return {
        "sys_enabled": bool(byte & (1 << StatusBit.SYS_ENABLED)),
        "mtr1_enabled": bool(byte & (1 << StatusBit.MTR1_ENABLED)),
        "mtr1_ready": bool(byte & (1 << StatusBit.MTR1_READY)),
        "mtr2_enabled": bool(byte & (1 << StatusBit.MTR2_ENABLED)),
        "mtr2_ready": bool(byte & (1 << StatusBit.MTR2_READY)),
        "error_code": ErrorCode((byte >> 5) & 0x07),
    }


def parse_encoder_index(data: bytes) -> tuple[float, int]:
    """Parse encoder index frame: MDL(4B)=position at index, MDH first byte=motor_id."""
    if len(data) < 5:
        raise ValueError(f"Expected at least 5 bytes, got {len(data)}")
    pos_raw = struct.unpack(">i", data[0:4])[0]
    motor_id = data[4]
    return q24_to_float(pos_raw), motor_id
