"""CAN backend abstraction.

We wrap python-can so we can swap real slcan ↔ mock backend for offline testing.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable
import time


@dataclass
class CanFrame:
    """A single CAN 2.0A frame."""
    arbitration_id: int          # 11-bit ID
    data: bytes                  # 0-8 bytes
    timestamp: float = 0.0       # monotonic time
    log: bool = True             # whether the frame should be logged on transmit

    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.monotonic()

    def hex_data(self) -> str:
        return self.data.hex(' ').upper()

    def __repr__(self) -> str:
        return f"CanFrame(id=0x{self.arbitration_id:03X}, data={self.hex_data()})"


class CanBackend(ABC):
    """Abstract CAN backend."""

    def __init__(self):
        self._on_receive: Callable[[CanFrame], None] | None = None
        self._on_error: Callable[[Exception], None] | None = None

    def set_receive_callback(self, cb: Callable[[CanFrame], None]) -> None:
        self._on_receive = cb

    def set_error_callback(self, cb: Callable[[Exception], None]) -> None:
        self._on_error = cb

    @property
    @abstractmethod
    def is_open(self) -> bool: ...

    @abstractmethod
    def open(self, **kwargs) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def send(self, frame: CanFrame) -> None: ...
