"""State models for motors and system."""
from dataclasses import dataclass, field
from .. protocol.messages import ErrorCode


@dataclass
class MotorState:
    """State của 1 motor."""
    motor_id: int
    enabled: bool = False
    ready: bool = False
    iq_actual: float = 0.0      # A
    iq_ref: float = 0.0         # A
    position: float = 0.0       # motor revolutions
    velocity: float = 0.0       # krpm
    index_position: float | None = None  # last detected index pulse position
    timestamp: float = 0.0      # monotonic time of last position update
    ctrl_mode: int = 0          # 0=TORQUE, 1=VELOCITY, 2=POSITION (CtrlMode enum)


@dataclass
class SystemState:
    """State toàn bộ hệ thống."""
    sys_enabled: bool = False
    error_code: ErrorCode = ErrorCode.NO_ERROR
    motor1: MotorState = field(default_factory=lambda: MotorState(motor_id=1))
    motor2: MotorState = field(default_factory=lambda: MotorState(motor_id=2))
    adc_a6: float = 0.0
    adc_b6: float = 0.0
    last_status_time: float = 0.0     # monotonic; updated by STATUS (0x010)
    last_telemetry_time: float = 0.0  # monotonic; updated by position/velocity/current frames
    is_connected: bool = False

    @property
    def has_error(self) -> bool:
        return self.error_code != ErrorCode.NO_ERROR

    @property
    def both_motors_ready(self) -> bool:
        return self.motor1.ready and self.motor2.ready
