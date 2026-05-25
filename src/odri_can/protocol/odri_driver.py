"""High-level driver for ODRI mw_dual_motor_torque_ctrl firmware.

Wraps a CanBackend and provides domain-level methods:
- enable_system(), enable_motors(), set_iq_ref(), e_stop()
- Parses incoming frames and updates SystemState
- Emits Qt-style signals (using a simple callback list, decoupled from PySide6)
"""
import copy
import logging
import threading
import time
from typing import Callable

from ..can.interface import CanBackend, CanFrame
from ..models.motor_state import SystemState
from .messages import RxId, TxId, Command, CtrlMode, ErrorCode, ERROR_DESCRIPTIONS
from .codec import (
    pack_two_q24, pack_command, unpack_two_q24,
    float_to_q24, parse_status_byte, parse_encoder_index,
)

logger = logging.getLogger(__name__)

StateCallback = Callable[[SystemState], None]


class OdriDriver:
    """High-level driver for the ODRI dual motor torque control firmware."""

    # Software safety limits — applied before sending IqRef to firmware
    DEFAULT_IQ_LIMIT = 5.0  # A; conservative default, raise after testing

    def __init__(self, backend: CanBackend):
        self._backend = backend
        self._backend.set_receive_callback(self._on_frame)
        self._backend.set_error_callback(self._on_backend_error)

        self._state = SystemState()
        self._state_lock = threading.Lock()
        self._state_listeners: list[StateCallback] = []
        self._last_notify = 0.0  # monotonic; throttle UI callbacks to ~50 Hz

        self._iq_limit = self.DEFAULT_IQ_LIMIT

        # Watchdog: continuously resend refs so firmware doesn't trigger CAN_RECV_TIMEOUT
        self._iq_ref_1 = 0.0
        self._iq_ref_2 = 0.0
        self._iq_ref_lock = threading.Lock()
        self._watchdog_thread: threading.Thread | None = None
        self._watchdog_stop = threading.Event()
        self._watchdog_period = 0.005  # 200 Hz

        # Velocity / Position reference (mailbox 0x006, Q24 krpm or rev)
        self._velpos_ref_1 = 0.0
        self._velpos_ref_2 = 0.0
        self._velpos_ref_lock = threading.Lock()

        # Per-motor control mode (default: TORQUE)
        self._ctrl_mode: dict[int, CtrlMode] = {1: CtrlMode.TORQUE, 2: CtrlMode.TORQUE}

        # Position move: cancellation events, accumulated target, and thread refs per motor
        self._move_stop: dict[int, threading.Event] = {1: threading.Event(), 2: threading.Event()}
        self._move_target: dict[int, float | None] = {1: None, 2: None}  # None = no move active
        self._move_target_lock = threading.Lock()
        self._move_threads: dict[int, threading.Thread | None] = {1: None, 2: None}

        # +1 or -1 per motor: flips Iq sign in position controller
        # Set to -1 if positive Iq makes the encoder decrease for that motor.
        self._motor_direction: dict[int, float] = {1: 1.0, 2: 1.0}

        # Position unwrapping: Q24 signed int32 overflows at ±128 rev.
        # Track accumulated offsets so continuous rotation > 128 rev works correctly.
        self._raw_pos: dict[int, float] = {1: 0.0, 2: 0.0}
        self._pos_offset: dict[int, float] = {1: 0.0, 2: 0.0}
        _WRAP_RANGE = 256.0   # full Q24 range (128 − (−128))
        self._WRAP_HALF = _WRAP_RANGE / 2   # 128 — jump larger than this = overflow


    # ------------------ State access ------------------

    @property
    def state(self) -> SystemState:
        with self._state_lock:
            return self._state

    def add_state_listener(self, cb: StateCallback) -> None:
        self._state_listeners.append(cb)

    def remove_state_listener(self, cb: StateCallback) -> None:
        if cb in self._state_listeners:
            self._state_listeners.remove(cb)

    def _notify_state(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last_notify < 0.02:  # throttle: tối đa 50 Hz
            return
        self._last_notify = now
        with self._state_lock:
            snapshot = copy.deepcopy(self._state)  # bản sao bất biến cho UI thread
        for cb in self._state_listeners:
            try:
                cb(snapshot)
            except Exception:
                logger.exception("State listener error")

    # ------------------ Lifecycle ------------------

    def start_watchdog(self) -> None:
        """Start resending IqRef in a background thread (safety + watchdog)."""
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            return
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()

    def stop_watchdog(self) -> None:
        self._watchdog_stop.set()
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=1.0)

    def shutdown(self) -> None:
        """Safe shutdown: zero references, reset to torque mode, disable motors/system."""
        try:
            for ev in self._move_stop.values():
                ev.set()
            for t in self._move_threads.values():
                if t and t.is_alive():
                    t.join(timeout=0.15)
            self.set_iq_ref(0.0, 0.0)
            with self._velpos_ref_lock:
                self._velpos_ref_1 = 0.0
                self._velpos_ref_2 = 0.0
            self._send_velpos_ref_now(0.0, 0.0, log=False)
            self.set_ctrl_mode_both(CtrlMode.TORQUE)
            time.sleep(0.02)
            self.disable_motors()
            time.sleep(0.02)
            self.disable_system()
        except Exception:
            logger.exception("Error during shutdown")
        finally:
            self.stop_watchdog()

    # ------------------ Position unwrapping ------------------

    def _unwrap_pos(self, motor_id: int, raw: float) -> float:
        delta = raw - self._raw_pos[motor_id]
        if delta > self._WRAP_HALF:
            self._pos_offset[motor_id] -= 256.0
        elif delta < -self._WRAP_HALF:
            self._pos_offset[motor_id] += 256.0
        self._raw_pos[motor_id] = raw
        return raw + self._pos_offset[motor_id]

    # ------------------ Commands ------------------

    def _send_command(self, code: Command, value: int, log: bool = True) -> None:
        frame = CanFrame(arbitration_id=int(TxId.COMMANDS),
                         data=pack_command(int(code), value),
                         log=log)
        self._backend.send(frame)

    def enable_system(self) -> None:
        self._send_command(Command.ENABLE_SYS, 1)
        with self._state_lock:
            self._state.sys_enabled = True
        self._notify_state(force=True)

    def disable_system(self) -> None:
        self._send_command(Command.ENABLE_SYS, 0)
        with self._state_lock:
            self._state.sys_enabled = False
        self._notify_state(force=True)

    def enable_motor1(self) -> None:
        self._send_command(Command.ENABLE_MTR1, 1)
        with self._state_lock:
            self._state.motor1.enabled = True
            self._state.motor1.ready = False  # aligning
        self._notify_state(force=True)

    def enable_motor2(self) -> None:
        self._send_command(Command.ENABLE_MTR2, 1)
        with self._state_lock:
            self._state.motor2.enabled = True
            self._state.motor2.ready = False  # aligning
        self._notify_state(force=True)

    def disable_motor1(self) -> None:
        self._send_command(Command.ENABLE_MTR1, 0)
        with self._state_lock:
            self._state.motor1.enabled = False
            self._state.motor1.ready = False
        self._notify_state(force=True)

    def disable_motor2(self) -> None:
        self._send_command(Command.ENABLE_MTR2, 0)
        with self._state_lock:
            self._state.motor2.enabled = False
            self._state.motor2.ready = False
        self._notify_state(force=True)

    def enable_motors(self) -> None:
        """Enable both motors. CALLER MUST first set iq_ref=0 and enable_system."""
        self.enable_motor1()
        self.enable_motor2()

    def disable_motors(self) -> None:
        self.disable_motor1()
        self.disable_motor2()

    def request_telemetry(self, log: bool = False) -> None:
        """One-shot telemetry request: SEND_ALL=1 then immediately SEND_ALL=0.

        Firmware starts persistent streaming on SEND_ALL=1, so we stop it
        immediately with SEND_ALL=0. The firmware still sends one burst of
        4 frames (current/position/velocity/ADC) before stopping.
        """
        self._send_command(Command.SEND_ALL, 1, log=log)
        self._send_command(Command.SEND_ALL, 0, log=log)

    def enable_telemetry(self, enable: bool = True, log: bool = True) -> None:
        """Enable/disable continuous telemetry streaming. Prefer request_telemetry()."""
        self._send_command(Command.SEND_ALL, 1 if enable else 0, log=log)

    def set_can_recv_timeout(self, timeout_ms: int) -> None:
        """Set CAN watchdog timeout. 0 = disabled."""
        self._send_command(Command.SET_CAN_RECV_TIMEOUT, timeout_ms)

    def set_motor_direction(self, motor_id: int, direction: float) -> None:
        """Set position-controller Iq sign for motor_id. Use -1.0 if positive Iq
        causes the encoder to decrease (motor wired/mounted in reverse)."""
        if motor_id not in (1, 2):
            raise ValueError("motor_id must be 1 or 2")
        self._motor_direction[motor_id] = 1.0 if direction >= 0 else -1.0
        logger.info("Motor %d direction set to %+.0f", motor_id,
                    self._motor_direction[motor_id])

    def set_iq_limit(self, limit_amperes: float) -> None:
        if limit_amperes < 0:
            raise ValueError("iq_limit must be >= 0")
        self._iq_limit = limit_amperes

    def set_iq_ref(self, iq_motor1: float, iq_motor2: float, log: bool = True) -> None:
        """Set Iq reference for both motors (clamped to ±iq_limit)."""
        iq1 = max(-self._iq_limit, min(self._iq_limit, iq_motor1))
        iq2 = max(-self._iq_limit, min(self._iq_limit, iq_motor2))
        with self._iq_ref_lock:
            self._iq_ref_1 = iq1
            self._iq_ref_2 = iq2
        self._send_iq_ref_now(iq1, iq2, log=log)

    def _send_iq_ref_now(self, iq1: float, iq2: float, log: bool = True) -> None:
        frame = CanFrame(arbitration_id=int(TxId.IQ_REF),
                         data=pack_two_q24(iq1, iq2),
                         log=log)
        try:
            self._backend.send(frame)
        except Exception:
            logger.exception("Failed to send IqRef")

    # ------------------ Control mode ------------------

    def set_ctrl_mode(self, motor_id: int, mode: CtrlMode) -> None:
        """Set control mode for one motor (TORQUE / VELOCITY / POSITION)."""
        if motor_id not in (1, 2):
            raise ValueError("motor_id must be 1 or 2")
        cmd = Command.SET_CTRL_MODE_MTR1 if motor_id == 1 else Command.SET_CTRL_MODE_MTR2
        self._send_command(cmd, int(mode))
        self._ctrl_mode[motor_id] = mode
        with self._state_lock:
            motor = self._state.motor1 if motor_id == 1 else self._state.motor2
            motor.ctrl_mode = int(mode)
        self._notify_state(force=True)
        logger.info("Motor %d ctrl_mode → %s", motor_id, mode.name)

    def set_ctrl_mode_both(self, mode: CtrlMode) -> None:
        """Set the same control mode for both motors."""
        self.set_ctrl_mode(1, mode)
        self.set_ctrl_mode(2, mode)

    # ------------------ Velocity / Position reference ------------------

    def set_vel_ref(self, vel1: float, vel2: float, log: bool = True) -> None:
        """Send velocity setpoint for both motors (Q24, krpm). Motor must be in VELOCITY mode."""
        with self._velpos_ref_lock:
            self._velpos_ref_1 = vel1
            self._velpos_ref_2 = vel2
        self._send_velpos_ref_now(vel1, vel2, log=log)

    def set_hw_pos_ref(self, pos1: float, pos2: float, log: bool = True) -> None:
        """Send position setpoint for both motors (Q24, rev). Motor must be in POSITION mode."""
        with self._velpos_ref_lock:
            self._velpos_ref_1 = pos1
            self._velpos_ref_2 = pos2
        self._send_velpos_ref_now(pos1, pos2, log=log)

    def _send_velpos_ref_now(self, ref1: float, ref2: float, log: bool = True) -> None:
        frame = CanFrame(arbitration_id=int(TxId.VEL_POS_REF),
                         data=pack_two_q24(ref1, ref2),
                         log=log)
        try:
            self._backend.send(frame)
        except Exception:
            logger.exception("Failed to send VelPosRef")

    # ------------------ PID gain tuning ------------------

    def set_vel_pid(self, motor_id: int, kp: float, ki: float, kd: float) -> None:
        """Send velocity PID gains to firmware (Q24 encoded). Applied to one motor."""
        if motor_id == 1:
            self._send_command(Command.SET_VEL_KP_MTR1, float_to_q24(kp))
            self._send_command(Command.SET_VEL_KI_MTR1, float_to_q24(ki))
            self._send_command(Command.SET_VEL_KD_MTR1, float_to_q24(kd))
        else:
            self._send_command(Command.SET_VEL_KP_MTR2, float_to_q24(kp))
            self._send_command(Command.SET_VEL_KI_MTR2, float_to_q24(ki))
            self._send_command(Command.SET_VEL_KD_MTR2, float_to_q24(kd))
        logger.info("Motor %d vel PID: kp=%.3f ki=%.3f kd=%.3f", motor_id, kp, ki, kd)

    def set_pos_pid(self, motor_id: int, kp: float, ki: float, kd: float) -> None:
        """Send position PID gains to firmware (Q24 encoded). Applied to one motor."""
        if motor_id == 1:
            self._send_command(Command.SET_POS_KP_MTR1, float_to_q24(kp))
            self._send_command(Command.SET_POS_KI_MTR1, float_to_q24(ki))
            self._send_command(Command.SET_POS_KD_MTR1, float_to_q24(kd))
        else:
            self._send_command(Command.SET_POS_KP_MTR2, float_to_q24(kp))
            self._send_command(Command.SET_POS_KI_MTR2, float_to_q24(ki))
            self._send_command(Command.SET_POS_KD_MTR2, float_to_q24(kd))
        logger.info("Motor %d pos PID: kp=%.3f ki=%.3f kd=%.3f", motor_id, kp, ki, kd)

    def rotate_motor(self, motor_id: int, revolutions: float,
                     kp: float = 0.5, kd: float = 0.15, timeout: float = 10.0) -> None:
        """Non-blocking: drive motor_id by `revolutions` turns using software PD control.

        Pressing multiple times while a move is in progress accumulates the target:
        e.g. pressing +1 rev twice → motor moves to current_pos + 2 rev total.

        Args:
            motor_id: 1 or 2
            revolutions: signed number of revolutions (+ = forward, - = backward)
            kp: proportional gain [1/s]
            kd: velocity gain [A·s/rev]
            timeout: max seconds before giving up
        """
        if motor_id not in (1, 2):
            raise ValueError("motor_id must be 1 or 2")

        with self._move_target_lock:
            current_target = self._move_target[motor_id]
            if current_target is None:
                # No move in progress: base target on current position
                state = self.state
                current_pos = state.motor1.position if motor_id == 1 else state.motor2.position
                new_target = current_pos + revolutions
            else:
                # Move in progress: accumulate onto existing target
                new_target = current_target + revolutions
            self._move_target[motor_id] = new_target

        # If already moving, the running thread will pick up the new target automatically.
        # Only start a new thread if none is running.
        if current_target is None:
            ev = threading.Event()
            self._move_stop[motor_id] = ev
            t = threading.Thread(
                target=self._position_move,
                args=(motor_id, kp, 0.0, kd, 1.0, timeout, ev),
                daemon=True,
            )
            self._move_threads[motor_id] = t
            t.start()
        logger.info("Motor %d: target set to %.3f rev", motor_id, new_target)

    def move_to_abs(self, motor_id: int, target_pos: float,
                    kp: float = 1.0, ki: float = 0.1, kd: float = 0.3,
                    max_i: float = 0.5, timeout: float = 30.0) -> None:
        """Non-blocking: move motor_id to absolute position target_pos [rev].

        Cascade PI+P controller:
            kp:      outer position proportional gain [1/s]
            ki:      outer position integral gain [1/s²] — eliminates static error under load
            kd:      inner velocity proportional gain [A·s/rev]
            max_i:   current cap [A]
        """
        if motor_id not in (1, 2):
            raise ValueError("motor_id must be 1 or 2")

        with self._move_target_lock:
            current_target = self._move_target[motor_id]
            self._move_target[motor_id] = target_pos

        if current_target is None:
            ev = threading.Event()
            self._move_stop[motor_id] = ev
            t = threading.Thread(
                target=self._position_move,
                args=(motor_id, kp, ki, kd, max_i, timeout, ev),
                daemon=True,
            )
            self._move_threads[motor_id] = t
            t.start()
        logger.info("Motor %d: move_to_abs %.3f rev (max_i=%.2f A)", motor_id, target_pos, max_i)

    def _position_move(self, motor_id: int,
                       kp: float, ki: float, kd: float, max_i: float,
                       timeout: float, stop_event: threading.Event) -> None:
        _FAIL_SAFE_T  = 0.1    # zero iq if no new telemetry for this long [s]
        _DT_DEFAULT   = 0.02   # dt assumed on first valid frame
        _DT_FLOOR     = 0.001  # minimum dt
        _DT_CAP       = 0.1    # dt above this → stale/gap, reset integral and skip
        _INTEGRAL_LIM = 1.0
        _VEL_DB       = 0.01   # velocity deadband [krpm]
        _HOLD_ERR     = 0.02   # [rev] threshold for soft-hold zone
        _HOLD_LIM     = 0.1    # [A] iq limit inside soft-hold zone

        last_ts  = None
        deadline = time.monotonic() + timeout
        integral = 0.0

        with self._move_target_lock:
            target = self._move_target[motor_id]
        logger.info("Motor %d: move started → %.3f rev", motor_id, target)

        while not stop_event.is_set():
            # Re-read target (user may update it while playing)
            with self._move_target_lock:
                new_target = self._move_target[motor_id]
            if abs(new_target - target) > 1e-6:   # point 2: float-safe compare
                target   = new_target
                deadline = time.monotonic() + timeout
                integral = 0.0

            # Point 1: poll timestamp — no event race condition
            # spin at 1 ms until motor.timestamp changes or fail-safe triggers
            wait_start = time.monotonic()
            while not stop_event.is_set():
                state = self.state
                motor = state.motor1 if motor_id == 1 else state.motor2
                ts    = motor.timestamp
                if ts and ts != last_ts:
                    break                           # new POSITION frame processed
                if time.monotonic() - wait_start > _FAIL_SAFE_T:
                    break                           # no frame for 100 ms
                stop_event.wait(0.002)              # yield; interruptible by stop

            if stop_event.is_set():
                break

            # Fail-safe: no new frame arrived within _FAIL_SAFE_T
            if not ts or ts == last_ts:
                logger.warning("Motor %d: telemetry timeout, zeroing iq", motor_id)
                with self._iq_ref_lock:
                    iq1, iq2 = self._iq_ref_1, self._iq_ref_2
                if motor_id == 1:
                    self.set_iq_ref(0.0, iq2)
                else:
                    self.set_iq_ref(iq1, 0.0)
                continue

            raw_dt = (ts - last_ts) if last_ts is not None else _DT_DEFAULT
            last_ts = ts

            if raw_dt > _DT_CAP:
                integral = 0.0
                continue
            
            dt = max(_DT_FLOOR, raw_dt)

            pos, vel = motor.position, motor.velocity

            # Point 4: velocity deadband — suppress noise-driven hunting
            if abs(vel) < _VEL_DB:
                vel = 0.0

            if time.monotonic() > deadline:
                logger.warning("Motor %d: move timeout at %.3f rev", motor_id, pos)
                break

            err = target - pos

            # Point 3: soft hold — PD still active near target, just at lower limit
            iq_lim   = _HOLD_LIM if abs(err) < _HOLD_ERR else max_i

            if abs(err) < _HOLD_ERR:
                integral = 0.0
            else:
                integral = max(-_INTEGRAL_LIM, min(_INTEGRAL_LIM, integral + err * dt))
                
            iq = kp * err + ki * integral - kd * vel
            iq = max(-iq_lim, min(iq_lim, iq))

            logger.debug("M%d pos=%.3f tgt=%.3f err=%.3f vel=%.3f iq=%.3f", motor_id, pos, target, err, vel, iq)

            with self._iq_ref_lock:
                iq1, iq2 = self._iq_ref_1, self._iq_ref_2
            if motor_id == 1:
                self.set_iq_ref(iq, iq2)
            else:
                self.set_iq_ref(iq1, iq)

        # stop_motor_move / e_stop already zero iq on the stop_event path
        with self._move_target_lock:
            self._move_target[motor_id] = None
        self._move_threads[motor_id] = None
        logger.info("Motor %d: move ended", motor_id)

    def stop_motor_move(self, motor_id: int) -> None:
        """Cancel in-progress position move for one motor and zero its Iq."""
        if motor_id not in (1, 2):
            raise ValueError("motor_id must be 1 or 2")
        self._move_stop[motor_id].set()
        with self._move_target_lock:
            self._move_target[motor_id] = None
        with self._iq_ref_lock:
            iq1, iq2 = self._iq_ref_1, self._iq_ref_2
        if motor_id == 1:
            self.set_iq_ref(0.0, iq2)
        else:
            self.set_iq_ref(iq1, 0.0)
        logger.info("Motor %d: move paused", motor_id)

    def stop_moves(self) -> None:
        """Cancel any in-progress position moves and zero Iq. Does NOT disable motors/system."""
        for ev in self._move_stop.values():
            ev.set()
        with self._move_target_lock:
            self._move_target = {1: None, 2: None}
        self.set_iq_ref(0.0, 0.0)
        logger.info("Position moves cancelled")

    def e_stop(self) -> None:
        """Emergency stop: zero refs + disable motors immediately. NON-BLOCKING."""
        logger.warning("E-STOP triggered")
        for ev in self._move_stop.values():
            ev.set()
        with self._move_target_lock:
            self._move_target = {1: None, 2: None}
        try:
            self.set_iq_ref(0.0, 0.0)
            with self._velpos_ref_lock:
                self._velpos_ref_1 = 0.0
                self._velpos_ref_2 = 0.0
            self._send_velpos_ref_now(0.0, 0.0, log=False)
            self.disable_motors()
            self.disable_system()
        except Exception:
            logger.exception("E-stop send error (continuing)")

    def _watchdog_loop(self) -> None:
        """Resend refs at 200 Hz; re-request telemetry every 20 ms (50 Hz).

        Refs are sent only after alignment completes (motor.ready=True).
        During alignment, only telemetry is requested so the UI can show progress.
        tick starts at _telemetry_every so the first SEND_ALL fires immediately.
        """
        _telemetry_every = 4  # 4 × 5 ms = 20 ms → 50 Hz
        tick = _telemetry_every
        while not self._watchdog_stop.is_set():
            with self._state_lock:
                sys_on    = self._state.sys_enabled
                any_ready = self._state.motor1.ready or self._state.motor2.ready

            # Only send control refs after alignment is done (motor.ready).
            # During alignment the firmware handles current internally — sending
            # IqRef=0 can interfere and keep the motor stuck in the ALIGN state.
            if sys_on and any_ready:
                mode1, mode2 = self._ctrl_mode[1], self._ctrl_mode[2]

                if (mode1 == CtrlMode.TORQUE or mode2 == CtrlMode.TORQUE):
                    with self._iq_ref_lock:
                        iq1, iq2 = self._iq_ref_1, self._iq_ref_2
                    try:
                        self._send_iq_ref_now(iq1, iq2, log=False)
                    except Exception:
                        pass

                if (mode1 in (CtrlMode.VELOCITY, CtrlMode.POSITION) or
                        mode2 in (CtrlMode.VELOCITY, CtrlMode.POSITION)):
                    with self._velpos_ref_lock:
                        r1, r2 = self._velpos_ref_1, self._velpos_ref_2
                    try:
                        self._send_velpos_ref_now(r1, r2, log=False)
                    except Exception:
                        pass

            # Request telemetry only when system is on (includes alignment phase).
            if sys_on:
                tick += 1
                if tick >= _telemetry_every:
                    tick = 0
                    try:
                        self.request_telemetry()
                    except Exception:
                        pass

            time.sleep(self._watchdog_period)

    # ------------------ Receive parsing ------------------

    def _on_frame(self, frame: CanFrame) -> None:
        try:
            with self._state_lock:
                if frame.arbitration_id == RxId.STATUS:
                    self._parse_status(frame)
                elif frame.arbitration_id == RxId.CURRENT_IQ:
                    iq1, iq2 = unpack_two_q24(frame.data)
                    self._state.motor1.iq_actual = iq1
                    self._state.motor2.iq_actual = iq2
                    self._state.last_telemetry_time = time.monotonic()
                elif frame.arbitration_id == RxId.POSITION:
                    p1, p2 = unpack_two_q24(frame.data)
                    ts = time.monotonic()
                    self._state.motor1.position = self._unwrap_pos(1, p1)
                    self._state.motor2.position = self._unwrap_pos(2, p2)
                    self._state.motor1.timestamp = ts
                    self._state.motor2.timestamp = ts
                    self._state.last_telemetry_time = ts
                elif frame.arbitration_id == RxId.VELOCITY:
                    v1, v2 = unpack_two_q24(frame.data)
                    self._state.motor1.velocity = v1
                    self._state.motor2.velocity = v2
                    self._state.last_telemetry_time = time.monotonic()
                elif frame.arbitration_id == RxId.ADC6:
                    a6, b6 = unpack_two_q24(frame.data)
                    self._state.adc_a6 = a6
                    self._state.adc_b6 = b6
                elif frame.arbitration_id == RxId.ENCODER_INDEX:
                    pos, motor_id = parse_encoder_index(frame.data)
                    if motor_id == 1:
                        self._state.motor1.index_position = pos
                    elif motor_id == 2:
                        self._state.motor2.index_position = pos
                else:
                    return  # Unknown ID, ignore
            self._notify_state()
        except Exception:
            logger.exception("Frame parse error: %s", frame)

    def _parse_status(self, frame: CanFrame) -> None:
        if not frame.data:
            return
        flags = parse_status_byte(frame.data[0])
        s = self._state

        # sys_enabled / motor.enabled are tracked by software commands — STATUS cannot
        # override them. Only motor.ready (alignment done) and error_code come from STATUS.
        # Exception: if firmware reports an error, trust the full STATUS state for safety.
        has_error = flags["error_code"] != ErrorCode.NO_ERROR
        if has_error:
            s.sys_enabled = flags["sys_enabled"]
            s.motor1.enabled = flags["mtr1_enabled"]
            s.motor2.enabled = flags["mtr2_enabled"]

        s.motor1.ready = flags["mtr1_ready"]
        s.motor2.ready = flags["mtr2_ready"]

        prev_err = s.error_code
        s.error_code = flags["error_code"]
        s.last_status_time = time.monotonic()
        s.is_connected = True
        if s.error_code != ErrorCode.NO_ERROR and s.error_code != prev_err:
            logger.error("Firmware error: %s (%s)",
                         s.error_code.name, ERROR_DESCRIPTIONS.get(s.error_code, "?"))

    def _on_backend_error(self, exc: Exception) -> None:
        logger.error("CAN backend error: %s", exc)
