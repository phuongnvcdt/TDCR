"""Realtime telemetry plots using pyqtgraph."""
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QWidget, QVBoxLayout


class TelemetryPlots(QWidget):
    """3 stacked plots: Iq, position, velocity. Last ~10 seconds rolling window."""

    WINDOW_SECONDS = 10.0
    UPDATE_HZ = 30

    def __init__(self):
        super().__init__()
        self._t0 = time.monotonic()
        self._buf_size = 2000
        self._times = deque(maxlen=self._buf_size)
        self._iq1 = deque(maxlen=self._buf_size)
        self._iq2 = deque(maxlen=self._buf_size)
        self._pos1 = deque(maxlen=self._buf_size)
        self._pos2 = deque(maxlen=self._buf_size)
        self._vel1 = deque(maxlen=self._buf_size)
        self._vel2 = deque(maxlen=self._buf_size)

        self._build()

        # Decouple plot redraw from data arrival rate
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._redraw)
        self._timer.start(int(1000 / self.UPDATE_HZ))

    def _build(self):
        pg.setConfigOptions(antialias=True, background="w", foreground="k")
        layout = QVBoxLayout(self)

        self.plot_iq = pg.PlotWidget(title="Iq Current [A]")
        self.plot_vel = pg.PlotWidget(title="Velocity [rpm]")
        self.plot_pos = pg.PlotWidget(title="Position [rev]")

        for p in (self.plot_iq, self.plot_vel, self.plot_pos):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("bottom", "Time", units="s")
            p.addLegend()

        self.curve_iq1 = self.plot_iq.plot([], [], pen=pg.mkPen("#00c", width=2), name="M1")
        self.curve_iq2 = self.plot_iq.plot([], [], pen=pg.mkPen("#c00", width=2), name="M2")
        self.curve_vel1 = self.plot_vel.plot([], [], pen=pg.mkPen("#00c", width=2), name="M1")
        self.curve_vel2 = self.plot_vel.plot([], [], pen=pg.mkPen("#c00", width=2), name="M2")
        self.curve_pos1 = self.plot_pos.plot([], [], pen=pg.mkPen("#00c", width=2), name="M1")
        self.curve_pos2 = self.plot_pos.plot([], [], pen=pg.mkPen("#c00", width=2), name="M2")

        layout.addWidget(self.plot_iq)
        layout.addWidget(self.plot_vel)
        layout.addWidget(self.plot_pos)

    def push_state(self, state):
        """Append latest state to buffers (called on each state update)."""
        t = time.monotonic() - self._t0
        self._times.append(t)
        self._iq1.append(state.motor1.iq_actual)
        self._iq2.append(state.motor2.iq_actual)
        self._pos1.append(state.motor1.position)
        self._pos2.append(state.motor2.position)
        self._vel1.append(state.motor1.velocity * 1000)
        self._vel2.append(state.motor2.velocity * 1000)

    def _redraw(self):
        if not self._times:
            return
        t = np.fromiter(self._times, dtype=float)
        # Rolling window
        t_max = t[-1]
        t_min = max(t[0], t_max - self.WINDOW_SECONDS)
        mask = t >= t_min
        t = t[mask]

        self.curve_iq1.setData(t, np.fromiter(self._iq1, dtype=float)[mask])
        self.curve_iq2.setData(t, np.fromiter(self._iq2, dtype=float)[mask])
        self.curve_pos1.setData(t, np.fromiter(self._pos1, dtype=float)[mask])
        self.curve_pos2.setData(t, np.fromiter(self._pos2, dtype=float)[mask])
        self.curve_vel1.setData(t, np.fromiter(self._vel1, dtype=float)[mask])
        self.curve_vel2.setData(t, np.fromiter(self._vel2, dtype=float)[mask])

        for p in (self.plot_iq, self.plot_pos, self.plot_vel):
            p.setXRange(t_min, t_max, padding=0)

    def clear(self):
        for buf in (self._times, self._iq1, self._iq2,
                    self._pos1, self._pos2, self._vel1, self._vel2):
            buf.clear()
        self._t0 = time.monotonic()
