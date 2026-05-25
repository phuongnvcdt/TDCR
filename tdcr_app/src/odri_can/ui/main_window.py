"""Main application window."""
import logging

from PySide6.QtCore import Qt, Signal, QObject
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QSplitter, QMessageBox,
)

from ..can.slcan_backend import SlcanBackend
from ..can.mock_backend import MockBackend
from ..protocol.odri_driver import OdriDriver
from ..protocol.messages import CtrlMode
from ..models.motor_state import SystemState

from .connection_panel import ConnectionPanel
from .control_panel import ControlPanel
from .telemetry_plots import TelemetryPlots
from .log_panel import LogPanel

logger = logging.getLogger(__name__)


class _StateBridge(QObject):
    """Bridges background-thread state callbacks → Qt signal on UI thread."""
    state_arrived = Signal(object)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tendon-Driven Continuum Robot Controller")
        self.resize(1400, 1280)

        self._backend = None
        self._driver: OdriDriver | None = None
        self._bridge = _StateBridge()
        self._bridge.state_arrived.connect(self._on_state_ui)

        self._build_ui()
        self._install_shortcuts()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)

        self.connection_panel = ConnectionPanel()
        self.connection_panel.connect_requested.connect(self._handle_connect)
        self.connection_panel.disconnect_requested.connect(self._handle_disconnect)
        outer.addWidget(self.connection_panel)

        # Splitter: left controls, right plots
        splitter = QSplitter(Qt.Orientation.Horizontal)
        outer.addWidget(splitter, stretch=1)

        # Left: control + log
        left = QSplitter(Qt.Orientation.Vertical)
        self.control_panel = ControlPanel()
        self.control_panel.enable_system_clicked.connect(self._on_enable_system)
        self.control_panel.disable_system_clicked.connect(self._on_disable_system)
        self.control_panel.enable_motors_clicked.connect(self._on_enable_motors)
        self.control_panel.disable_motors_clicked.connect(self._on_disable_motors)
        self.control_panel.iq_ref_changed.connect(self._on_iq_ref)
        self.control_panel.iq_limit_changed.connect(self._on_iq_limit)
        self.control_panel.vel_ref_changed.connect(self._on_vel_ref)
        self.control_panel.fw_pos_ref_changed.connect(self._on_fw_pos_ref)
        self.control_panel.fw_pid_changed.connect(self._on_fw_pid)
        self.control_panel.set_home_clicked.connect(self._on_set_home)
        self.control_panel.mode_changed.connect(self._on_mode_changed)
        left.addWidget(self.control_panel)

        self.log_panel = LogPanel()
        left.addWidget(self.log_panel)
        left.setSizes([425, 425])
        left.setStretchFactor(0, 0)
        left.setStretchFactor(1, 1)

        splitter.addWidget(left)

        # Right: plots
        self.plots = TelemetryPlots()
        splitter.addWidget(self.plots)
        splitter.setSizes([550, 850])

    def _install_shortcuts(self):
        # Spacebar = E-Stop
        sc = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        sc.activated.connect(self._on_estop)

    # ------------------ Connection handling ------------------

    def _handle_connect(self, port: str, bitrate: int, use_mock: bool):
        try:
            if use_mock:
                self._backend = MockBackend()
                self._backend.open()
                info = "(mock)"
            else:
                if not port:
                    QMessageBox.warning(self, "No port", "Please select a COM port.")
                    return
                self._backend = SlcanBackend()
                self._backend.open(port=port, bitrate_code=bitrate)
                info = f"(slcan {port})"

            self._driver = OdriDriver(self._backend)
            self._driver.add_state_listener(self._on_state_bg)
            self._driver.set_iq_limit(self.control_panel.iq_limit_spin.value())

            # Start watchdog (50 Hz SEND_ALL), then request telemetry immediately
            self._driver.start_watchdog()
            self._driver.request_telemetry()

            self.connection_panel.set_connected(True, info)
            self.control_panel.set_connected(True)
            self.plots.clear()
            logger.info("Connected: %s", info)
        except Exception as e:
            logger.exception("Connect failed")
            QMessageBox.critical(self, "Connection failed", str(e))
            self._cleanup_backend()

    def _handle_disconnect(self):
        try:
            if self._driver:
                self._driver.shutdown()
        except Exception:
            logger.exception("Shutdown error")
        finally:
            self._cleanup_backend()
            self.connection_panel.set_connected(False)
            self.control_panel.set_connected(False)
            logger.info("Disconnected")

    def _cleanup_backend(self):
        if self._backend:
            try:
                self._backend.close()
            except Exception:
                logger.exception("Backend close error")
        self._backend = None
        self._driver = None

    # ------------------ Command handlers ------------------

    def _on_enable_system(self):
        if self._driver:
            self._driver.enable_system()
            self._driver.request_telemetry()
            logger.info("ENABLE_SYS=1")

    def _on_disable_system(self):
        if self._driver:
            self._driver.disable_system()
            self._driver.request_telemetry()
            logger.info("ENABLE_SYS=0")

    def _on_enable_motors(self):
        if not self._driver:
            return
        # Safe sequence: zero IqRef first
        self.control_panel.zero_iq()
        self._driver.set_iq_ref(0.0, 0.0)
        self._driver.enable_motor1()
        self._driver.enable_motor2()
        self._driver.request_telemetry()
        logger.info("ENABLE_MTR1=1, ENABLE_MTR2=1")

    def _on_disable_motors(self):
        if self._driver:
            self._driver.disable_motors()
            self._driver.request_telemetry()
            logger.info("ENABLE_MTR1=0, ENABLE_MTR2=0")

    def _on_estop(self):
        if self._driver:
            self._driver.e_stop()
            self.control_panel.zero_iq()
            logger.warning("E-STOP activated")

    def _on_iq_ref(self, iq1: float, iq2: float):
        if self._driver:
            self._driver.set_iq_ref(iq1, iq2)

    def _on_iq_limit(self, limit: float):
        if self._driver:
            self._driver.set_iq_limit(limit)
            logger.info("Iq limit set to %.2f A", limit)

    def _on_vel_ref(self, vel1: float, vel2: float):
        if self._driver:
            self._driver.set_vel_ref(vel1, vel2)

    def _on_fw_pos_ref(self, pos1: float, pos2: float):
        if self._driver:
            self._driver.set_hw_pos_ref(pos1, pos2)

    def _on_fw_pid(self, pid_type: str, motor_id: int,
                   kp: float, ki: float, kd: float):
        if not self._driver:
            return
        if pid_type == "vel":
            self._driver.set_vel_pid(motor_id, kp, ki, kd)
        else:
            self._driver.set_pos_pid(motor_id, kp, ki, kd)

    def _on_set_home(self):
        if not self._driver:
            return
        state = self._driver.state
        pos1 = state.motor1.position
        pos2 = state.motor2.position
        self.control_panel.update_home(pos1, pos2)
        logger.info("Home set: M1=%.3f rev, M2=%.3f rev", pos1, pos2)

    def _on_mode_changed(self, mode: str):
        if not self._driver:
            return
        if mode == "iq":
            self._driver.stop_moves()
            self._driver.set_ctrl_mode_both(CtrlMode.TORQUE)
            logger.info("Ctrl mode → TORQUE")
        elif mode == "vel":
            self._driver.set_vel_ref(0.0, 0.0)
            self._driver.set_ctrl_mode_both(CtrlMode.VELOCITY)
            logger.info("Ctrl mode → VELOCITY")
        elif mode == "pos":
            state = self._driver.state
            # Hold current position when entering position mode
            p1, p2 = state.motor1.position, state.motor2.position
            self._driver.set_hw_pos_ref(p1, p2)
            self._driver.set_ctrl_mode_both(CtrlMode.POSITION)
            logger.info("Ctrl mode → POSITION (hold %.3f, %.3f rev)", p1, p2)

    # ------------------ State updates ------------------

    def _on_state_bg(self, state: SystemState):
        """Called from CAN backend thread — marshal to UI thread via signal."""
        self._bridge.state_arrived.emit(state)

    def _on_state_ui(self, state: SystemState):
        """Runs on UI thread."""
        self.control_panel.update_state(state)
        self.plots.push_state(state)

    # ------------------ Window close ------------------

    def closeEvent(self, event):
        try:
            if self._driver:
                self._driver.shutdown()
            self._cleanup_backend()
        except Exception:
            logger.exception("Close error")
        super().closeEvent(event)
