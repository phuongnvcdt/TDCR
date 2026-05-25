"""Mock CAN backend simulating ODRI mw_dual_motor_torque_ctrl firmware.

Allows testing the full app stack without hardware. Simulates:
- Echo of commands (status updates when ENABLE_* is sent)
- Periodic telemetry (status @ 100Hz, current/position/velocity @ 1kHz when SEND_ALL=1)
- Simple motor model: velocity integrates from Iq, position integrates from velocity
"""
import threading
import time
import logging
import math

from .interface import CanBackend, CanFrame
from ..protocol.messages import RxId, TxId, Command, StatusBit, ErrorCode
from ..protocol.codec import (
    pack_two_q24, unpack_two_q24, unpack_two_int32,
)

logger = logging.getLogger(__name__)


class MockBackend(CanBackend):
    """In-process mock of the ODRI firmware. No hardware required."""

    def __init__(self):
        super().__init__()
        self._open = False
        self._stop_event = threading.Event()
        self._tx_thread: threading.Thread | None = None

        # Simulated firmware state
        self._sys_enabled = False
        self._mtr1_enabled = False
        self._mtr2_enabled = False
        self._send_all = False
        self._send_current = False
        self._send_position = False
        self._send_velocity = False
        self._iq_ref_1 = 0.0
        self._iq_ref_2 = 0.0
        # Simulated motor state
        self._pos_1 = 0.0
        self._pos_2 = 0.0
        self._vel_1 = 0.0
        self._vel_2 = 0.0
        # Time of motor enable (for ready delay simulation)
        self._mtr1_enable_time: float | None = None
        self._mtr2_enable_time: float | None = None
        self._ready_delay = 1.5  # s, simulate alignment time
        self._error_code = ErrorCode.NO_ERROR

    @property
    def is_open(self) -> bool:
        return self._open

    def open(self, **kwargs) -> None:
        if self._open:
            raise RuntimeError("Mock already open")
        self._open = True
        self._stop_event.clear()
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()
        logger.info("MockBackend started")

    def close(self) -> None:
        self._stop_event.set()
        if self._tx_thread is not None:
            self._tx_thread.join(timeout=1.0)
        self._open = False
        logger.info("MockBackend closed")

    def send(self, frame: CanFrame) -> None:
        """Receive a command from the 'PC' and update simulated state."""
        if not self._open:
            raise RuntimeError("Mock not open")
        if frame.log:
            logger.info("CAN TX 0x%03X %s", frame.arbitration_id, frame.hex_data())
        try:
            self._handle_pc_to_board(frame)
        except Exception as e:
            logger.exception("Mock handle error")
            if self._on_error:
                self._on_error(e)

    def _handle_pc_to_board(self, frame: CanFrame) -> None:
        if frame.arbitration_id == TxId.COMMANDS:
            if len(frame.data) != 8:
                return
            value, code = unpack_two_int32(frame.data)
            self._apply_command(code, value)
        elif frame.arbitration_id == TxId.IQ_REF:
            iq1, iq2 = unpack_two_q24(frame.data)
            self._iq_ref_1 = iq1
            self._iq_ref_2 = iq2

    def _apply_command(self, code: int, value: int) -> None:
        if code == Command.ENABLE_SYS:
            self._sys_enabled = bool(value)
        elif code == Command.ENABLE_MTR1:
            self._mtr1_enabled = bool(value)
            self._mtr1_enable_time = time.monotonic() if value else None
        elif code == Command.ENABLE_MTR2:
            self._mtr2_enabled = bool(value)
            self._mtr2_enable_time = time.monotonic() if value else None
        elif code == Command.SEND_ALL:
            self._send_all = bool(value)
        elif code == Command.SEND_CURRENT:
            self._send_current = bool(value)
        elif code == Command.SEND_POSITION:
            self._send_position = bool(value)
        elif code == Command.SEND_VELOCITY:
            self._send_velocity = bool(value)
        # Other commands ignored in mock

    def _is_ready(self, enable_time: float | None) -> bool:
        if enable_time is None:
            return False
        return (time.monotonic() - enable_time) > self._ready_delay

    def _tx_loop(self) -> None:
        """Periodically send telemetry frames to the 'PC'."""
        last_status = 0.0
        last_telem = 0.0
        last_sim = time.monotonic()

        while not self._stop_event.is_set():
            now = time.monotonic()
            dt = now - last_sim
            last_sim = now

            # Simple motor sim: vel += k * iq * dt, pos += vel * dt
            # k chosen so 1A produces ~10 rev/s² accel, with light damping
            k = 10.0
            damping = 2.0
            if self._mtr1_enabled and self._sys_enabled and self._is_ready(self._mtr1_enable_time):
                self._vel_1 += (k * self._iq_ref_1 - damping * self._vel_1) * dt
                self._pos_1 += self._vel_1 * dt
            if self._mtr2_enabled and self._sys_enabled and self._is_ready(self._mtr2_enable_time):
                self._vel_2 += (k * self._iq_ref_2 - damping * self._vel_2) * dt
                self._pos_2 += self._vel_2 * dt

            # Status @ 100Hz
            if now - last_status > 0.01:
                self._emit_status()
                last_status = now

            # Telemetry @ 200Hz (slow enough for slcan; firmware actually does 1kHz)
            if (self._send_all or self._send_current or self._send_position or self._send_velocity) and (now - last_telem > 0.005):
                if self._send_all or self._send_current:
                    # Mock: assume Iq actual ≈ Iq ref (perfect FOC)
                    self._emit_frame(RxId.CURRENT_IQ, pack_two_q24(self._iq_ref_1, self._iq_ref_2))
                if self._send_all or self._send_position:
                    self._emit_frame(RxId.POSITION, pack_two_q24(self._pos_1, self._pos_2))
                if self._send_all or self._send_velocity:
                    self._emit_frame(RxId.VELOCITY, pack_two_q24(self._vel_1, self._vel_2))
                last_telem = now

            time.sleep(0.001)

    def _emit_status(self) -> None:
        byte = 0
        if self._sys_enabled:
            byte |= 1 << StatusBit.SYS_ENABLED
        if self._mtr1_enabled:
            byte |= 1 << StatusBit.MTR1_ENABLED
        if self._is_ready(self._mtr1_enable_time):
            byte |= 1 << StatusBit.MTR1_READY
        if self._mtr2_enabled:
            byte |= 1 << StatusBit.MTR2_ENABLED
        if self._is_ready(self._mtr2_enable_time):
            byte |= 1 << StatusBit.MTR2_READY
        byte |= (int(self._error_code) & 0x07) << 5
        self._emit_frame(RxId.STATUS, bytes([byte]))

    def _emit_frame(self, can_id: int, data: bytes) -> None:
        if self._on_receive is None:
            return
        logger.info("CAN RX 0x%03X %s", can_id, data.hex(' ').upper())
        frame = CanFrame(arbitration_id=int(can_id), data=data)
        try:
            self._on_receive(frame)
        except Exception:
            logger.exception("Receive callback error in mock")
