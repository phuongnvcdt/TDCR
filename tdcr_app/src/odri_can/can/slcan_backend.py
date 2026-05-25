"""Slcan backend wrapping python-can.

CANable v2.0 with slcan firmware appears as a virtual COM port.
python-can's 'slcan' interface handles the ASCII protocol (S8\\r, O\\r, t.../T..., C\\r).
"""
import sys
import threading
import logging
from typing import Optional

# When main.py is run as a script, Python inserts src/odri_can/ into sys.path[0],
# causing `import can` to resolve to the local can/ subpackage instead of python-can.
# Remove that entry before importing.
sys.path = [p for p in sys.path if not p.replace("\\", "/").rstrip("/").endswith("/odri_can")]
sys.modules.pop("can", None)

import can  # python-can

from .interface import CanBackend, CanFrame

logger = logging.getLogger(__name__)


# Map our bitrate enum (matching slcan S0-S8) to python-can numeric bitrates
_SLCAN_BITRATES = {
    0: 10_000,
    1: 20_000,
    2: 50_000,
    3: 100_000,
    4: 125_000,
    5: 250_000,
    6: 500_000,
    7: 800_000,
    8: 1_000_000,
}


class SlcanBackend(CanBackend):
    """Slcan-based CAN backend (CANable v2.0 with slcan firmware)."""

    def __init__(self):
        super().__init__()
        self._bus: Optional[can.BusABC] = None
        self._notifier: Optional[can.Notifier] = None
        self._lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    def open(
        self,
        port: str = "COM3",
        bitrate_code: int = 8,
        receive_own_messages: bool = False,
    ) -> None:
        """Open slcan bus.

        Args:
            port: COM port (Windows: 'COM3', Linux: '/dev/ttyACM0')
            bitrate_code: 0-8 corresponding to S0-S8 (8 = 1 Mbps, ODRI default)
        """
        if self._bus is not None:
            raise RuntimeError("Backend already open")

        bitrate = _SLCAN_BITRATES.get(bitrate_code)
        if bitrate is None:
            raise ValueError(f"Invalid bitrate_code {bitrate_code}, must be 0-8")

        logger.info("Opening slcan on %s at %d bps", port, bitrate)
        self._bus = can.Bus(
            interface="slcan",
            channel=port,
            bitrate=bitrate,
            receive_own_messages=receive_own_messages,
        )

        # Notifier runs a thread that calls our listener for each incoming frame
        self._notifier = can.Notifier(self._bus, [self._on_msg])

    def close(self) -> None:
        with self._lock:
            if self._notifier is not None:
                self._notifier.stop()
                self._notifier = None
            if self._bus is not None:
                try:
                    self._bus.shutdown()
                except Exception:
                    logger.exception("Error during bus shutdown")
                self._bus = None
        logger.info("Slcan backend closed")

    def send(self, frame: CanFrame) -> None:
        with self._lock:
            if self._bus is None:
                raise RuntimeError("Backend not open")
            if frame.log:
                logger.info("CAN TX 0x%03X %s", frame.arbitration_id, frame.hex_data())
            else:
                logger.debug("CAN TX 0x%03X %s", frame.arbitration_id, frame.hex_data())
            msg = can.Message(
                arbitration_id=frame.arbitration_id,
                data=frame.data,
                is_extended_id=False,
            )
            try:
                self._bus.send(msg, timeout=0.1)
            except can.CanError as e:
                if self._bus is None:
                    # Backend is closing/closed; ignore transient send error during disconnect.
                    return
                logger.error("CAN send failed: %s", e)
                if self._on_error:
                    self._on_error(e)
                raise

    def _on_msg(self, msg: can.Message) -> None:
        """Called by python-can Notifier on each received frame."""
        if msg.is_error_frame or msg.is_remote_frame:
            return
        data = bytes(msg.data)
        logger.debug("CAN RX 0x%03X %s", msg.arbitration_id, data.hex(' ').upper())
        frame = CanFrame(
            arbitration_id=msg.arbitration_id,
            data=data,
            timestamp=msg.timestamp,
        )
        if self._on_receive:
            try:
                self._on_receive(frame)
            except Exception as e:
                logger.exception("Receive callback error")
                if self._on_error:
                    self._on_error(e)
