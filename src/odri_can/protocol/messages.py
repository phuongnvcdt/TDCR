"""
ODRI mw_dual_motor_vel_pos_ctrl CAN Protocol Constants.

Reference: canapi.h in mw_dual_motor_vel_pos_ctrl firmware.
- 11-bit standard CAN ID
- 1 Mbit/s, sampling point 86.7%
- 8-byte data split into MDL (low 4 bytes) and MDH (high 4 bytes)
- Motor data uses Q24 fixed-point: float = int_value / 2^24
"""
from enum import IntEnum


# ---- Messages từ board → PC (telemetry) ----
class RxId(IntEnum):
    STATUS = 0x010          # 1 byte: status flags + error code
    CURRENT_IQ = 0x020      # MDL=Iq M1, MDH=Iq M2 (Q24, A)
    POSITION = 0x030        # MDL=Pos M1, MDH=Pos M2 (Q24, motor revs)
    VELOCITY = 0x040        # MDL=Vel M1, MDH=Vel M2 (Q24, krpm)
    ADC6 = 0x050            # MDL=ADC A6, MDH=ADC B6
    ENCODER_INDEX = 0x060   # MDL=position at index, MDH(1B)=motor ID


# ---- Messages từ PC → board (control) ----
class TxId(IntEnum):
    COMMANDS = 0x000        # MDL=value, MDH=command code
    IQ_REF = 0x005          # MDL=IqRef M1, MDH=IqRef M2 (Q24, A)
    VEL_POS_REF = 0x006     # MDL=Ref M1, MDH=Ref M2 (Q24, krpm or rev)


# ---- Control modes (used with SET_CTRL_MODE commands) ----
class CtrlMode(IntEnum):
    TORQUE = 0      # IqRef from mailbox 0x005 (original behaviour)
    VELOCITY = 1    # Velocity PID — SpeedRef from mailbox 0x006 (Q24, krpm)
    POSITION = 2    # Position PID — PosRef from mailbox 0x006 (Q24, rev)


# ---- Command codes (gửi trong MDH của frame ID 0x000) ----
class Command(IntEnum):
    ENABLE_SYS = 0x01
    ENABLE_MTR1 = 0x02
    ENABLE_MTR2 = 0x03
    ENABLE_VSPRING1 = 0x04
    ENABLE_VSPRING2 = 0x05
    SEND_CURRENT = 0x0C
    SEND_POSITION = 0x0D
    SEND_VELOCITY = 0x0E
    SEND_ADC6 = 0x0F
    SEND_ENC_INDEX = 0x10   # 16
    SEND_ALL = 0x14
    SET_CAN_RECV_TIMEOUT = 0x1E   # value: uint32 ms, 0=disable
    ENABLE_POS_ROLLOVER_ERROR = 0x1F
    # --- Control mode (value = CtrlMode enum) ---
    SET_CTRL_MODE_MTR1 = 40
    SET_CTRL_MODE_MTR2 = 41
    # --- Velocity PID gains (value = Q24 float) ---
    SET_VEL_KP_MTR1 = 50
    SET_VEL_KI_MTR1 = 51
    SET_VEL_KD_MTR1 = 52
    SET_VEL_KP_MTR2 = 53
    SET_VEL_KI_MTR2 = 54
    SET_VEL_KD_MTR2 = 55
    # --- Position PID gains (value = Q24 float) ---
    SET_POS_KP_MTR1 = 60
    SET_POS_KI_MTR1 = 61
    SET_POS_KD_MTR1 = 62
    SET_POS_KP_MTR2 = 63
    SET_POS_KI_MTR2 = 64
    SET_POS_KD_MTR2 = 65


# ---- Error codes (bit 5-7 của status byte) ----
class ErrorCode(IntEnum):
    NO_ERROR = 0
    ENCODER = 1
    CAN_RECV_TIMEOUT = 2
    CRITICAL_MOTOR_TEMP = 3
    POSITION_CONVERTER = 4
    POSITION_ROLLOVER = 5
    OTHER = 7


ERROR_DESCRIPTIONS = {
    ErrorCode.NO_ERROR: "No error",
    ErrorCode.ENCODER: "Encoder error",
    ErrorCode.CAN_RECV_TIMEOUT: "CAN receive timeout",
    ErrorCode.CRITICAL_MOTOR_TEMP: "Critical motor temperature",
    ErrorCode.POSITION_CONVERTER: "Position converter error",
    ErrorCode.POSITION_ROLLOVER: "Position rollover",
    ErrorCode.OTHER: "Other error",
}


# ---- Status byte bit positions ----
class StatusBit:
    SYS_ENABLED = 0
    MTR1_ENABLED = 1
    MTR1_READY = 2
    MTR2_ENABLED = 3
    MTR2_READY = 4
    # Bits 5-7: error code


# ---- Q24 fixed-point scale ----
Q24_SCALE = 1 << 24  # 16777216
