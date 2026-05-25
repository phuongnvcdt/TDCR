"""Connection panel: COM port selection, bitrate, connect/disconnect."""
from PySide6.QtCore import Signal, Qt
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QLabel, QComboBox, QPushButton, QGroupBox, QVBoxLayout,
)
from serial.tools import list_ports


class ConnectionPanel(QWidget):
    connect_requested = Signal(str, int, bool)  # port, bitrate_code, use_mock
    disconnect_requested = Signal()

    def __init__(self):
        super().__init__()
        self._build()

    def _build(self):
        group = QGroupBox("CAN Connection")
        outer = QVBoxLayout(self)
        outer.addWidget(group)

        layout = QHBoxLayout(group)

        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(140)
        self._refresh_ports()
        layout.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("⟳")
        self.refresh_btn.setToolTip("Refresh COM ports")
        self.refresh_btn.setFixedSize(26, 26)
        self.refresh_btn.setStyleSheet("font-size: 13pt; padding: 0px;")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        layout.addWidget(self.refresh_btn)

        layout.addWidget(QLabel("Bitrate:"))
        self.bitrate_combo = QComboBox()
        # Slcan codes: 4=125k, 6=500k, 8=1M
        for code, label in [(4, "125 kbps"), (6, "500 kbps"), (8, "1 Mbps (ODRI)")]:
            self.bitrate_combo.addItem(label, code)
        self.bitrate_combo.setCurrentIndex(2)  # default 1 Mbps
        layout.addWidget(self.bitrate_combo)

        self.mock_combo = QComboBox()
        self.mock_combo.addItem("Real hardware (slcan)", False)
        self.mock_combo.addItem("Mock (no hardware)", True)
        self.mock_combo.setCurrentIndex(0)
        self.mock_combo.setEnabled(False)
        layout.addWidget(self.mock_combo)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self.connect_btn)

        self.status_label = QLabel("● Disconnected")
        self.status_label.setStyleSheet("color: #c00; font-weight: bold;")
        layout.addWidget(self.status_label)
        layout.addStretch()

    def _refresh_ports(self):
        self.port_combo.clear()
        for p in list_ports.comports():
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)

        if self.port_combo.count() == 0:
            self.port_combo.addItem("(no ports found)", "")
            return

        # Auto-select the COM port whose description matches USD/USB Serial Device.
        preferred = None
        for index in range(self.port_combo.count()):
            text = self.port_combo.itemText(index).lower()
            if "usb serial device" in text:
                preferred = index
                break
        if preferred is not None:
            self.port_combo.setCurrentIndex(preferred)

    def _on_connect_clicked(self):
        if self.connect_btn.text() == "Connect":
            port = self.port_combo.currentData() or ""
            bitrate = self.bitrate_combo.currentData()
            use_mock = self.mock_combo.currentData()
            self.connect_requested.emit(port, bitrate, use_mock)
        else:
            self.disconnect_requested.emit()

    def set_connected(self, connected: bool, info: str = ""):
        if connected:
            self.connect_btn.setText("Disconnect")
            self.status_label.setText(f"● Connected {info}".strip())
            self.status_label.setStyleSheet("color: #080; font-weight: bold;")
            self.port_combo.setEnabled(False)
            self.bitrate_combo.setEnabled(False)
        else:
            self.connect_btn.setText("Connect")
            self.status_label.setText("● Disconnected")
            self.status_label.setStyleSheet("color: #c00; font-weight: bold;")
            self.port_combo.setEnabled(True)
            self.bitrate_combo.setEnabled(True)
