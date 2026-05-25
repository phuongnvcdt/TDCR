"""Control panel: system controls, 3-mode toggle (Iq/Vel/Pos), status, and refs."""
from PySide6.QtCore import Signal, Qt, QSettings
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider, QGroupBox,
    QDoubleSpinBox, QGridLayout, QFrame, QAbstractSpinBox, QMessageBox, QCheckBox,
)

_LABEL_W    = 58  # col 0: row labels
_SPIN_W     = 120  # col 1: spinboxes
_PID_LBL_W  = 26  # Kp / Ki / Kd labels

_SETTINGS_ORG = "Mestrostic"
_SETTINGS_APP = "OdriCAN"


class ControlPanel(QWidget):
    # System
    enable_system_clicked = Signal()
    disable_system_clicked = Signal()
    enable_motors_clicked = Signal()
    disable_motors_clicked = Signal()
    # Current (Iq) control
    iq_ref_changed = Signal(float, float)       # iq1, iq2
    iq_limit_changed = Signal(float)
    # Velocity control — both motors' setpoints (Q24 krpm), firmware VELOCITY mode
    vel_ref_changed = Signal(float, float)      # vel1, vel2 [krpm]
    vel_limit_changed = Signal(float)
    # Position control — absolute positions (Q24 rev), firmware POSITION mode
    fw_pos_ref_changed = Signal(float, float)   # abs_pos1, abs_pos2 [rev]
    # Firmware PID gains — ('vel'|'pos', motor_id, kp, ki, kd)
    fw_pid_changed = Signal(str, int, float, float, float)
    # Origin
    set_home_clicked = Signal()
    # Mode: "iq" | "vel" | "pos"
    mode_changed = Signal(str)

    def __init__(self):
        super().__init__()
        self._mode = ""
        self._home = [0.0, 0.0]
        self._pos_playing = False
        self._build()
        self._load_settings()
        self._connect_settings_save()
        self.set_connected(False)

    # ── helpers ───────────────────────────────────────────────────

    @staticmethod
    def _to_slider(v: float, vmin: float, vmax: float) -> int:
        span = vmax - vmin
        if span == 0:
            return 0
        return round((v - vmin) / span * 2000 - 1000)

    @staticmethod
    def _from_slider(s: int, vmin: float, vmax: float) -> float:
        return vmin + (s + 1000) / 2000 * (vmax - vmin)

    def _make_spin(self, vmin: float, vmax: float,
                   decimals: int, step: float, width: int = _SPIN_W) -> QDoubleSpinBox:
        sp = QDoubleSpinBox()
        sp.setRange(vmin, vmax)
        sp.setDecimals(decimals)
        sp.setSingleStep(step)
        sp.setValue(0.0)
        sp.setFixedWidth(width)
        return sp

    def _make_slider(self) -> QSlider:
        sl = QSlider(Qt.Orientation.Horizontal)
        sl.setRange(-1000, 1000)
        sl.setValue(0)
        return sl

    def _make_pid_spin(self) -> QDoubleSpinBox:
        sp = QDoubleSpinBox()
        sp.setRange(0.0, 200.0)
        sp.setDecimals(4)
        sp.setSingleStep(0.1)
        sp.setValue(0.0)
        sp.setFixedWidth(90)
        sp.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        return sp

    # ── build ─────────────────────────────────────────────────────

    def _build(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        # ── System controls ──────────────────────────────────────
        sys_group = QGroupBox("System")
        sys_grid = QGridLayout(sys_group)
        sys_grid.setSpacing(4)
        self.enable_sys_btn = QPushButton("Enable System")
        self.disable_sys_btn = QPushButton("Disable System")
        self.enable_mtr_btn = QPushButton("Enable Motors")
        self.disable_mtr_btn = QPushButton("Disable Motors")
        self.enable_sys_btn.clicked.connect(self.enable_system_clicked)
        self.disable_sys_btn.clicked.connect(self.disable_system_clicked)
        self.enable_mtr_btn.clicked.connect(self.enable_motors_clicked)
        self.disable_mtr_btn.clicked.connect(self.disable_motors_clicked)
        sys_grid.addWidget(self.enable_sys_btn, 0, 0)
        sys_grid.addWidget(self.disable_sys_btn, 0, 1)
        sys_grid.addWidget(self.enable_mtr_btn, 1, 0)
        sys_grid.addWidget(self.disable_mtr_btn, 1, 1)
        layout.addWidget(sys_group)

        # ── Mode toggle (3 modes) ────────────────────────────────
        mode_group = QGroupBox("Control Mode")
        mode_layout = QHBoxLayout(mode_group)
        self.iq_mode_btn  = QPushButton("⚡ Current (Iq)")
        self.vel_mode_btn = QPushButton("💨 Velocity")
        self.pos_mode_btn = QPushButton("📍 Position")
        for btn in (self.iq_mode_btn, self.vel_mode_btn, self.pos_mode_btn):
            btn.setCheckable(True)
            btn.setMinimumHeight(32)
        self.iq_mode_btn.clicked.connect(lambda: self._set_mode("iq"))
        self.vel_mode_btn.clicked.connect(lambda: self._set_mode("vel"))
        self.pos_mode_btn.clicked.connect(lambda: self._set_mode("pos"))
        mode_layout.addWidget(self.iq_mode_btn)
        mode_layout.addWidget(self.vel_mode_btn)
        mode_layout.addWidget(self.pos_mode_btn)
        layout.addWidget(mode_group)

        # ── System Status ────────────────────────────────────────
        status_group = QGroupBox("System Status")
        status_grid = QGridLayout(status_group)
        status_grid.setSpacing(4)

        status_grid.addWidget(QLabel("System:"), 0, 0)
        self.sys_status_lbl = QLabel("Disabled")
        self.sys_status_lbl.setStyleSheet("color: #888;")
        status_grid.addWidget(self.sys_status_lbl, 0, 1)
        status_grid.addWidget(QLabel("Error:"), 0, 2)
        self.error_lbl = QLabel("None")
        self.error_lbl.setStyleSheet("color: #080;")
        status_grid.addWidget(self.error_lbl, 0, 3)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        status_grid.addWidget(sep, 1, 0, 1, 5)

        for col, text in enumerate(["", "Iq actual", "Position", "Velocity", "Status"]):
            lbl = QLabel(text)
            lbl.setStyleSheet("font-weight: bold;")
            status_grid.addWidget(lbl, 2, col)

        self._iq_actual_lbls: list[QLabel] = []
        self._pos_lbls: list[QLabel] = []
        self._vel_lbls: list[QLabel] = []
        self._motor_status_lbls: list[QLabel] = []
        for i, name in enumerate(["Motor 1", "Motor 2"]):
            row = 3 + i
            status_grid.addWidget(QLabel(f"<b>{name}</b>"), row, 0)
            iq_lbl  = QLabel("0.000 A")
            pos_lbl = QLabel("0.000 rev")
            vel_lbl = QLabel("0.0 rpm")
            st_lbl  = QLabel("Disabled")
            st_lbl.setStyleSheet("color: #888;")
            status_grid.addWidget(iq_lbl,  row, 1)
            status_grid.addWidget(pos_lbl, row, 2)
            status_grid.addWidget(vel_lbl, row, 3)
            status_grid.addWidget(st_lbl,  row, 4)
            self._iq_actual_lbls.append(iq_lbl)
            self._pos_lbls.append(pos_lbl)
            self._vel_lbls.append(vel_lbl)
            self._motor_status_lbls.append(st_lbl)

        layout.addWidget(status_group)

        # ── Current (Iq) Control ─────────────────────────────────
        self.iq_group = QGroupBox("⚡ Current Control (A)")
        iq_grid = QGridLayout(self.iq_group)
        iq_grid.setSpacing(6)
        iq_grid.setColumnMinimumWidth(0, _LABEL_W)
        iq_grid.setColumnMinimumWidth(1, _SPIN_W)
        iq_grid.setColumnStretch(2, 1)

        lim_lbl = QLabel("Limit:")
        lim_lbl.setFixedWidth(_LABEL_W)
        iq_grid.addWidget(lim_lbl, 0, 0)
        self.iq_limit_spin = self._make_spin(0.1, 15.0, 2, 0.5)
        self.iq_limit_spin.setValue(2.0)
        self.iq_limit_spin.valueChanged.connect(self._on_iq_limit_changed)
        iq_grid.addWidget(self.iq_limit_spin, 0, 1)

        lim = self.iq_limit_spin.value()
        self.iq_spin1 = self._make_spin(-lim, lim, 3, 0.05)
        self.iq_slider1 = self._make_slider()
        self.iq_spin2 = self._make_spin(-lim, lim, 3, 0.05)
        self.iq_slider2 = self._make_slider()

        for row, (label, spin, slider) in enumerate(
            [("Motor 1:", self.iq_spin1, self.iq_slider1),
             ("Motor 2:", self.iq_spin2, self.iq_slider2)], start=1
        ):
            lbl = QLabel(label)
            lbl.setFixedWidth(_LABEL_W)
            iq_grid.addWidget(lbl,    row, 0)
            iq_grid.addWidget(spin,   row, 1)
            iq_grid.addWidget(slider, row, 2)

        self.iq_spin1.valueChanged.connect(
            lambda v: self._on_spin_changed(v, self.iq_spin1, self.iq_slider1, self._emit_iq))
        self.iq_slider1.valueChanged.connect(
            lambda s: self._on_slider_changed(s, self.iq_spin1, self._emit_iq))
        self.iq_spin2.valueChanged.connect(
            lambda v: self._on_spin_changed(v, self.iq_spin2, self.iq_slider2, self._emit_iq))
        self.iq_slider2.valueChanged.connect(
            lambda s: self._on_slider_changed(s, self.iq_spin2, self._emit_iq))

        iq_reset_btn = QPushButton("⟳ Reset All → 0")
        iq_reset_btn.clicked.connect(self._on_iq_reset)
        iq_grid.addWidget(iq_reset_btn, 3, 2, Qt.AlignmentFlag.AlignRight)

        layout.addWidget(self.iq_group)

        # ── Velocity Control ─────────────────────────────────────
        self.vel_group = QGroupBox("💨 Velocity Control (rpm)")
        vel_grid = QGridLayout(self.vel_group)
        vel_grid.setSpacing(6)
        vel_grid.setColumnMinimumWidth(0, _LABEL_W)
        vel_grid.setColumnMinimumWidth(1, _SPIN_W)
        vel_grid.setColumnStretch(2, 1)

        vlim_lbl = QLabel("Limit:")
        vlim_lbl.setFixedWidth(_LABEL_W)
        vel_grid.addWidget(vlim_lbl, 0, 0)
        self.vel_limit_spin = self._make_spin(100, 7200, 0, 100)
        self.vel_limit_spin.setValue(2000)
        self.vel_limit_spin.valueChanged.connect(self._on_vel_limit_changed)
        vel_grid.addWidget(self.vel_limit_spin, 0, 1)

        vlim = self.vel_limit_spin.value()
        self.vel_spin1 = self._make_spin(-vlim, vlim, 0, 50)
        self.vel_slider1 = self._make_slider()
        self.vel_spin2 = self._make_spin(-vlim, vlim, 0, 50)
        self.vel_slider2 = self._make_slider()

        for row, (label, spin, slider) in enumerate(
            [("Motor 1:", self.vel_spin1, self.vel_slider1),
             ("Motor 2:", self.vel_spin2, self.vel_slider2)], start=1
        ):
            lbl = QLabel(label)
            lbl.setFixedWidth(_LABEL_W)
            vel_grid.addWidget(lbl,    row, 0)
            vel_grid.addWidget(spin,   row, 1)
            vel_grid.addWidget(slider, row, 2)

        self.vel_spin1.valueChanged.connect(
            lambda v: self._on_spin_changed(v, self.vel_spin1, self.vel_slider1, self._emit_vel))
        self.vel_slider1.valueChanged.connect(
            lambda s: self._on_slider_changed(s, self.vel_spin1, self._emit_vel))
        self.vel_spin2.valueChanged.connect(
            lambda v: self._on_spin_changed(v, self.vel_spin2, self.vel_slider2, self._emit_vel))
        self.vel_slider2.valueChanged.connect(
            lambda s: self._on_slider_changed(s, self.vel_spin2, self._emit_vel))

        # Velocity PID gains
        vel_kp_lbl = QLabel("Kp:")
        vel_kp_lbl.setFixedWidth(_PID_LBL_W)
        vel_grid.addWidget(vel_kp_lbl, 3, 0)
        self.vel_kp_spin = self._make_pid_spin()
        vel_grid.addWidget(self.vel_kp_spin, 3, 1)
        vel_ki_kd_row = QHBoxLayout()
        vel_ki_lbl = QLabel("Ki:")
        vel_ki_lbl.setFixedWidth(_PID_LBL_W)
        vel_ki_kd_row.addWidget(vel_ki_lbl)
        self.vel_ki_spin = self._make_pid_spin()
        vel_ki_kd_row.addWidget(self.vel_ki_spin)
        vel_kd_lbl = QLabel("Kd:")
        vel_kd_lbl.setFixedWidth(_PID_LBL_W)
        vel_ki_kd_row.addWidget(vel_kd_lbl)
        self.vel_kd_spin = self._make_pid_spin()
        vel_ki_kd_row.addWidget(self.vel_kd_spin)
        vel_ki_kd_row.addStretch()
        vel_apply_pid_btn = QPushButton("Apply")
        vel_apply_pid_btn.setFixedWidth(60)
        vel_apply_pid_btn.clicked.connect(self._on_apply_vel_pid)
        vel_ki_kd_row.addWidget(vel_apply_pid_btn)
        vel_grid.addLayout(vel_ki_kd_row, 3, 2)

        vel_reset_btn = QPushButton("⟳ Reset All → 0")
        vel_reset_btn.clicked.connect(self._on_vel_reset)
        vel_grid.addWidget(vel_reset_btn, 4, 2, Qt.AlignmentFlag.AlignRight)

        layout.addWidget(self.vel_group)

        # ── Position Control (firmware PID) ──────────────────────
        self.pos_group = QGroupBox("📍 Position Control (rev)")
        pos_grid = QGridLayout(self.pos_group)
        pos_grid.setSpacing(6)
        pos_grid.setColumnMinimumWidth(0, _LABEL_W)
        pos_grid.setColumnMinimumWidth(1, _SPIN_W)
        pos_grid.setColumnStretch(2, 1)

        plim_lbl = QLabel("Limit:")
        plim_lbl.setFixedWidth(_LABEL_W)
        pos_grid.addWidget(plim_lbl, 0, 0)
        self.pos_limit_spin = self._make_spin(0.1, 500.0, 2, 0.5)
        self.pos_limit_spin.setValue(2.0)
        self.pos_limit_spin.valueChanged.connect(self._on_pos_limit_changed)
        pos_grid.addWidget(self.pos_limit_spin, 0, 1)

        self.pos_link_chk = QCheckBox("Link")
        self.pos_link_chk.setToolTip("Sync Motor 1 and Motor 2 position targets")
        pos_grid.addWidget(self.pos_link_chk, 0, 2, Qt.AlignmentFlag.AlignRight)

        plim = self.pos_limit_spin.value()
        self.pos_spin1 = self._make_spin(-plim, plim, 3, 0.1)
        self.pos_slider1 = self._make_slider()
        self.pos_spin2 = self._make_spin(-plim, plim, 3, 0.1)
        self.pos_slider2 = self._make_slider()

        for row, (label, spin, slider) in enumerate(
            [("Motor 1:", self.pos_spin1, self.pos_slider1),
             ("Motor 2:", self.pos_spin2, self.pos_slider2)], start=1
        ):
            lbl = QLabel(label)
            lbl.setFixedWidth(_LABEL_W)
            pos_grid.addWidget(lbl,    row, 0)
            pos_grid.addWidget(spin,   row, 1)
            pos_grid.addWidget(slider, row, 2)

        self.pos_spin1.valueChanged.connect(
            lambda v: self._on_pos_spin(v, self.pos_spin1, self.pos_slider1,
                                        self.pos_spin2, self.pos_slider2))
        self.pos_slider1.valueChanged.connect(
            lambda s: self._on_pos_slider(s, self.pos_spin1,
                                          self.pos_spin2, self.pos_slider2))
        self.pos_spin2.valueChanged.connect(
            lambda v: self._on_pos_spin(v, self.pos_spin2, self.pos_slider2,
                                        self.pos_spin1, self.pos_slider1))
        self.pos_slider2.valueChanged.connect(
            lambda s: self._on_pos_slider(s, self.pos_spin2,
                                          self.pos_spin1, self.pos_slider1))

        # Position PID gains
        pos_kp_lbl = QLabel("Kp:")
        pos_kp_lbl.setFixedWidth(_PID_LBL_W)
        pos_grid.addWidget(pos_kp_lbl, 3, 0)
        self.pos_kp_spin = self._make_pid_spin()
        pos_grid.addWidget(self.pos_kp_spin, 3, 1)
        pos_ki_kd_row = QHBoxLayout()
        pos_ki_lbl = QLabel("Ki:")
        pos_ki_lbl.setFixedWidth(_PID_LBL_W)
        pos_ki_kd_row.addWidget(pos_ki_lbl)
        self.pos_ki_spin = self._make_pid_spin()
        pos_ki_kd_row.addWidget(self.pos_ki_spin)
        pos_kd_lbl = QLabel("Kd:")
        pos_kd_lbl.setFixedWidth(_PID_LBL_W)
        pos_ki_kd_row.addWidget(pos_kd_lbl)
        self.pos_kd_spin = self._make_pid_spin()
        pos_ki_kd_row.addWidget(self.pos_kd_spin)
        pos_ki_kd_row.addStretch()
        pos_apply_pid_btn = QPushButton("Apply")
        pos_apply_pid_btn.setFixedWidth(60)
        pos_apply_pid_btn.clicked.connect(self._on_apply_pos_pid)
        pos_ki_kd_row.addWidget(pos_apply_pid_btn)
        pos_grid.addLayout(pos_ki_kd_row, 3, 2)

        action_row = QHBoxLayout()
        action_row.setSpacing(6)

        self.set_home_btn = QPushButton("🏠 Set Home")
        self.set_home_btn.setMinimumHeight(36)
        self.set_home_btn.setToolTip("Save current position as Home")
        self.set_home_btn.clicked.connect(self.set_home_clicked)

        self.pos_play_btn = QPushButton("▶  Play")
        self.pos_play_btn.setMinimumHeight(36)
        self.pos_play_btn.setToolTip("Send position target to motors (Play) / stop sending (Pause)")
        self.pos_play_btn.setStyleSheet(
            "QPushButton { font-weight: bold; background-color: #1a6a2a;"
            " color: white; border-radius: 4px; }"
            "QPushButton:hover { background-color: #2a9a3a; }"
        )
        self.pos_play_btn.clicked.connect(self._on_pos_play_toggle)

        self.go_home_btn = QPushButton("↩ Go Home")
        self.go_home_btn.setMinimumHeight(36)
        self.go_home_btn.setToolTip("Move both motors to the saved Home position")
        self.go_home_btn.clicked.connect(self._on_go_home)

        for btn in (self.set_home_btn, self.pos_play_btn, self.go_home_btn):
            action_row.addWidget(btn, stretch=1)

        pos_grid.addLayout(action_row, 4, 0, 1, 3)

        layout.addWidget(self.pos_group)
        layout.addStretch()

        self._apply_mode_style()

    # ── sync helpers ──────────────────────────────────────────────

    def _on_spin_changed(self, v: float, spin: QDoubleSpinBox,
                         slider: QSlider, emit_fn):
        slider.blockSignals(True)
        slider.setValue(self._to_slider(v, spin.minimum(), spin.maximum()))
        slider.blockSignals(False)
        emit_fn()

    def _on_slider_changed(self, s: int, spin: QDoubleSpinBox, emit_fn):
        v = self._from_slider(s, spin.minimum(), spin.maximum())
        spin.blockSignals(True)
        spin.setValue(v)
        spin.blockSignals(False)
        emit_fn()

    def _set_spin_slider(self, spin: QDoubleSpinBox, slider: QSlider, v: float):
        v = max(spin.minimum(), min(spin.maximum(), v))
        spin.blockSignals(True)
        slider.blockSignals(True)
        spin.setValue(v)
        slider.setValue(self._to_slider(v, spin.minimum(), spin.maximum()))
        spin.blockSignals(False)
        slider.blockSignals(False)

    def _update_spin_range(self, spin: QDoubleSpinBox, slider: QSlider,
                           vmin: float, vmax: float, vcurrent: float = None):
        if vcurrent is None:
            vcurrent = spin.value()
        v = max(vmin, min(vmax, vcurrent))
        spin.blockSignals(True)
        slider.blockSignals(True)
        spin.setRange(vmin, vmax)
        spin.setValue(v)
        slider.setValue(self._to_slider(v, vmin, vmax))
        spin.blockSignals(False)
        slider.blockSignals(False)

    # ── emit helpers ─────────────────────────────────────────────

    def _emit_iq(self):
        self.iq_ref_changed.emit(self.iq_spin1.value(), self.iq_spin2.value())

    def _emit_vel(self):
        self.vel_ref_changed.emit(self.vel_spin1.value() / 1000.0, self.vel_spin2.value() / 1000.0)

    def _emit_fw_pos(self):
        if self._pos_playing:
            self.fw_pos_ref_changed.emit(self.pos_spin1.value(), self.pos_spin2.value())

    # ── Mode ─────────────────────────────────────────────────────

    def _set_mode(self, mode: str):
        if mode == self._mode:
            return
        if mode in ("iq", "vel"):
            QMessageBox.warning(self, "Mode Switch Warning", "Only switch to this mode when the motors have no load.")
            self._on_iq_reset()
            self._on_vel_reset()
        # Leave pos mode: ensure paused; enter pos mode: start paused
        if self._mode == "pos" or mode == "pos":
            self._pos_set_paused()
        self._mode = mode
        self._apply_mode_style()
        self.mode_changed.emit(mode)

    def _apply_mode_style(self):
        active = (
            "QPushButton { font-weight: bold; background-color: #1a3a6a;"
            " color: white; border-radius: 4px; }"
        )
        self.iq_mode_btn.setChecked(self._mode == "iq")
        self.vel_mode_btn.setChecked(self._mode == "vel")
        self.pos_mode_btn.setChecked(self._mode == "pos")
        self.iq_mode_btn.setStyleSheet(active if self._mode == "iq" else "")
        self.vel_mode_btn.setStyleSheet(active if self._mode == "vel" else "")
        self.pos_mode_btn.setStyleSheet(active if self._mode == "pos" else "")

        if self.iq_mode_btn.isEnabled():  # only update groups when connected
            self.iq_group.setEnabled(self._mode == "iq")
            self.vel_group.setEnabled(self._mode == "vel")
            self.pos_group.setEnabled(self._mode == "pos")

    # ── Iq slots ──────────────────────────────────────────────────

    def _on_iq_limit_changed(self, value: float):
        self._update_spin_range(self.iq_spin1, self.iq_slider1, -value, value)
        self._update_spin_range(self.iq_spin2, self.iq_slider2, -value, value)
        self.iq_limit_changed.emit(value)

    def _on_iq_reset(self):
        self._set_spin_slider(self.iq_spin1, self.iq_slider1, 0.0)
        self._set_spin_slider(self.iq_spin2, self.iq_slider2, 0.0)
        self.iq_ref_changed.emit(0.0, 0.0)

    # ── Velocity slots ────────────────────────────────────────────

    def _on_vel_limit_changed(self, value: float):
        self._update_spin_range(self.vel_spin1, self.vel_slider1, -value, value)
        self._update_spin_range(self.vel_spin2, self.vel_slider2, -value, value)
        self.vel_limit_changed.emit(value)

    def _on_vel_reset(self):
        self._set_spin_slider(self.vel_spin1, self.vel_slider1, 0.0)
        self._set_spin_slider(self.vel_spin2, self.vel_slider2, 0.0)
        self.vel_ref_changed.emit(0.0, 0.0)

    def _on_apply_vel_pid(self):
        kp = self.vel_kp_spin.value()
        ki = self.vel_ki_spin.value()
        kd = self.vel_kd_spin.value()
        self.fw_pid_changed.emit("vel", 1, kp, ki, kd)
        self.fw_pid_changed.emit("vel", 2, kp, ki, kd)

    # ── Position slots ────────────────────────────────────────────

    def _pos_linked_value(self, v: float, src: QDoubleSpinBox,
                          dst: QDoubleSpinBox) -> float:
        src_span = src.maximum() - src.minimum()
        if src_span == 0:
            return dst.minimum()
        frac = (v - src.minimum()) / src_span
        return dst.minimum() + frac * (dst.maximum() - dst.minimum())

    def _on_pos_spin(self, v: float, spin: QDoubleSpinBox, slider: QSlider,
                     other_spin: QDoubleSpinBox, other_slider: QSlider):
        self._on_spin_changed(v, spin, slider, self._emit_fw_pos)
        if self.pos_link_chk.isChecked():
            self._set_spin_slider(other_spin, other_slider,
                                  self._pos_linked_value(v, spin, other_spin))

    def _on_pos_slider(self, s: int, spin: QDoubleSpinBox,
                       other_spin: QDoubleSpinBox, other_slider: QSlider):
        self._on_slider_changed(s, spin, self._emit_fw_pos)
        if self.pos_link_chk.isChecked():
            self._set_spin_slider(other_spin, other_slider,
                                  self._pos_linked_value(spin.value(), spin, other_spin))

    def _on_pos_limit_changed(self, value: float):
        self._update_spin_range(self.pos_spin1, self.pos_slider1,
                                self._home[0] - value, self._home[0] + value)
        self._update_spin_range(self.pos_spin2, self.pos_slider2,
                                self._home[1] - value, self._home[1] + value)

    def _on_go_home(self):
        """Set spinboxes to saved Home values (sends only if playing)."""
        self._set_spin_slider(self.pos_spin1, self.pos_slider1, self._home[0])
        self._set_spin_slider(self.pos_spin2, self.pos_slider2, self._home[1])
        self._emit_fw_pos()

    def _on_pos_play_toggle(self):
        self._pos_playing = not self._pos_playing
        if self._pos_playing:
            self.pos_play_btn.setText("⏸  Pause")
            self.pos_play_btn.setStyleSheet(
                "QPushButton { font-weight: bold; background-color: #8a3a00;"
                " color: white; border-radius: 4px; }"
                "QPushButton:hover { background-color: #c05000; }"
            )
            self._emit_fw_pos()
        else:
            self._pos_set_paused()

    def _pos_set_paused(self):
        self._pos_playing = False
        self.pos_play_btn.setText("▶  Play")
        self.pos_play_btn.setStyleSheet(
            "QPushButton { font-weight: bold; background-color: #1a6a2a;"
            " color: white; border-radius: 4px; }"
            "QPushButton:hover { background-color: #2a9a3a; }"
        )

    def _on_apply_pos_pid(self):
        kp = self.pos_kp_spin.value()
        ki = self.pos_ki_spin.value()
        kd = self.pos_kd_spin.value()
        self.fw_pid_changed.emit("pos", 1, kp, ki, kd)
        self.fw_pid_changed.emit("pos", 2, kp, ki, kd)

    # ── Public API ────────────────────────────────────────────────

    def update_home(self, pos1: float, pos2: float):
        """Store current motor positions as Home and update spinbox ranges."""
        self._home = [pos1, pos2]
        lim = self.pos_limit_spin.value()
        self._update_spin_range(self.pos_spin1, self.pos_slider1, pos1 - lim, pos1 + lim, pos1)
        self._update_spin_range(self.pos_spin2, self.pos_slider2, pos2 - lim, pos2 + lim, pos2)

    def zero_iq(self):
        """Programmatically zero Iq (called by MainWindow on enable motors)."""
        self._on_iq_reset()

    def zero_vel(self):
        """Zero velocity setpoints (called on mode switch away from vel)."""
        self._on_vel_reset()

    # ── Settings persistence ──────────────────────────────────────

    def _load_settings(self):
        s = QSettings(_SETTINGS_ORG, _SETTINGS_APP)

        def _f(key, default):
            try:
                return float(s.value(key, default))
            except (TypeError, ValueError):
                return float(default)

        self.iq_limit_spin.setValue(_f("control/iq_limit", 2.0))
        self.vel_limit_spin.setValue(_f("control/vel_limit", 5.0))
        self.pos_limit_spin.setValue(_f("control/pos_limit", 10.0))
        self.vel_kp_spin.setValue(_f("control/vel_kp", 0.0))
        self.vel_ki_spin.setValue(_f("control/vel_ki", 0.0))
        self.vel_kd_spin.setValue(_f("control/vel_kd", 0.0))
        self.pos_kp_spin.setValue(_f("control/pos_kp", 0.0))
        self.pos_ki_spin.setValue(_f("control/pos_ki", 0.0))
        self.pos_kd_spin.setValue(_f("control/pos_kd", 0.0))

    def _save_settings(self, *_):
        s = QSettings(_SETTINGS_ORG, _SETTINGS_APP)
        s.setValue("control/iq_limit",  self.iq_limit_spin.value())
        s.setValue("control/vel_limit", self.vel_limit_spin.value())
        s.setValue("control/pos_limit", self.pos_limit_spin.value())
        s.setValue("control/vel_kp",    self.vel_kp_spin.value())
        s.setValue("control/vel_ki",    self.vel_ki_spin.value())
        s.setValue("control/vel_kd",    self.vel_kd_spin.value())
        s.setValue("control/pos_kp",    self.pos_kp_spin.value())
        s.setValue("control/pos_ki",    self.pos_ki_spin.value())
        s.setValue("control/pos_kd",    self.pos_kd_spin.value())

    def _connect_settings_save(self):
        for spin in (self.iq_limit_spin, self.vel_limit_spin, self.pos_limit_spin,
                     self.vel_kp_spin, self.vel_ki_spin, self.vel_kd_spin,
                     self.pos_kp_spin, self.pos_ki_spin, self.pos_kd_spin):
            spin.valueChanged.connect(self._save_settings)

    def set_connected(self, connected: bool):
        for w in [
            self.enable_sys_btn, self.disable_sys_btn,
            self.enable_mtr_btn, self.disable_mtr_btn,
            self.iq_mode_btn, self.vel_mode_btn, self.pos_mode_btn,
        ]:
            w.setEnabled(connected)
        if connected:
            self._mode = ""
            self._apply_mode_style()
        else:
            self._pos_set_paused()
            self._mode = ""
            self._apply_mode_style()
            self.iq_group.setEnabled(False)
            self.vel_group.setEnabled(False)
            self.pos_group.setEnabled(False)

    def update_state(self, state):
        m1, m2 = state.motor1, state.motor2
        for m, iq_lbl, pos_lbl, vel_lbl, st_lbl in zip(
            [m1, m2],
            self._iq_actual_lbls, self._pos_lbls,
            self._vel_lbls, self._motor_status_lbls,
        ):
            iq_lbl.setText(f"{m.iq_actual:+.3f} A")
            pos_lbl.setText(f"{m.position:+.3f} rev")
            vel_lbl.setText(f"{m.velocity * 1000:+.1f} rpm")
            if not m.enabled:
                st_lbl.setText("Disabled")
                st_lbl.setStyleSheet("color: #888;")
            elif m.ready:
                st_lbl.setText("Ready ✓")
                st_lbl.setStyleSheet("color: #080; font-weight: bold;")
            else:
                st_lbl.setText("Aligning...")
                st_lbl.setStyleSheet("color: #c80;")

        if state.sys_enabled:
            self.sys_status_lbl.setText("Enabled ✓")
            self.sys_status_lbl.setStyleSheet("color: #080; font-weight: bold;")
        else:
            self.sys_status_lbl.setText("Disabled")
            self.sys_status_lbl.setStyleSheet("color: #888;")

        if state.has_error:
            from ..protocol.messages import ERROR_DESCRIPTIONS
            msg = ERROR_DESCRIPTIONS.get(state.error_code, "?")
            self.error_lbl.setText(f"⚠ {msg}")
            self.error_lbl.setStyleSheet("color: #c00; font-weight: bold;")
        else:
            self.error_lbl.setText("None")
            self.error_lbl.setStyleSheet("color: #080;")
