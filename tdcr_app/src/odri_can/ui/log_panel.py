"""Simple log panel showing recent log messages."""
import logging
import re
from collections import deque

from PySide6.QtCore import QEvent, QObject, Qt, QTimer, Signal
from PySide6.QtGui import QStandardItem, QStandardItemModel
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QGroupBox, QCheckBox,
    QComboBox, QLabel,
)

_CAN_RE = re.compile(r'CAN (TX|RX) (0x[0-9A-Fa-f]+)')


class _CanLogFilter(logging.Filter):
    """Applies TX/RX direction and CAN ID filters to a log handler."""

    def __init__(self):
        super().__init__()
        self.dir_filter: set[str] = set()
        self.id_filter: set[str] = set()

    def filter(self, record: logging.LogRecord) -> bool:
        if not self.dir_filter and not self.id_filter:
            return True
        m = _CAN_RE.search(record.getMessage())
        if not m:
            return True  # non-CAN messages always pass
        direction, msg_id = m.group(1), m.group(2).lower()
        if self.dir_filter and direction not in self.dir_filter:
            return False
        if self.id_filter and msg_id not in self.id_filter:
            return False
        return True

_ID_OPTIONS = [
    ("0x000  CMD",      "0x000"),
    ("0x005  IqRef",    "0x005"),
    ("0x010  Status",   "0x010"),
    ("0x020  Current",  "0x020"),
    ("0x030  Position", "0x030"),
    ("0x040  Velocity", "0x040"),
    ("0x050  ADC6",     "0x050"),
    ("0x060  EncIdx",   "0x060"),
]


class CheckableComboBox(QComboBox):
    """QComboBox with checkable (multi-select) items.

    Popup stays open while clicking items; closes on click outside.
    """
    selection_changed = Signal()

    def __init__(self, placeholder: str = "All", parent=None):
        super().__init__(parent)
        self.setEditable(True)
        self.lineEdit().setReadOnly(True)
        self._placeholder = placeholder
        self.setModel(QStandardItemModel(self))
        self.view().viewport().installEventFilter(self)
        self._update_text()

    def addCheckItem(self, label: str, value: str) -> None:
        item = QStandardItem(label)
        item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsUserCheckable)
        item.setCheckState(Qt.CheckState.Unchecked)
        item.setData(value, Qt.ItemDataRole.UserRole)
        self.model().appendRow(item)
        self._update_text()  # reset placeholder after QComboBox auto-updates display

    def checkedValues(self) -> list[str]:
        return [
            self.model().item(i).data(Qt.ItemDataRole.UserRole)
            for i in range(self.model().rowCount())
            if self.model().item(i).checkState() == Qt.CheckState.Checked
        ]

    def eventFilter(self, obj, event):
        if obj == self.view().viewport() and event.type() == QEvent.Type.MouseButtonRelease:
            index = self.view().indexAt(event.pos())
            item = self.model().itemFromIndex(index)
            if item:
                new_state = (Qt.CheckState.Unchecked
                             if item.checkState() == Qt.CheckState.Checked
                             else Qt.CheckState.Checked)
                item.setCheckState(new_state)
                self._update_text()
                self.selection_changed.emit()
            return True  # consume → popup stays open
        return super().eventFilter(obj, event)

    def _update_text(self):
        checked = [
            self.model().item(i).data(Qt.ItemDataRole.UserRole)
            for i in range(self.model().rowCount())
            if self.model().item(i).checkState() == Qt.CheckState.Checked
        ]
        self.lineEdit().setText(", ".join(checked) if checked else self._placeholder)


class _SignalLogHandler(logging.Handler, QObject):
    log_emitted = Signal(str)

    def __init__(self):
        logging.Handler.__init__(self)
        QObject.__init__(self)

    def emit(self, record):
        try:
            msg = self.format(record)
            self.log_emitted.emit(msg)
        except Exception:
            self.handleError(record)


class LogPanel(QWidget):
    MAX_LINES = 500

    def __init__(self):
        super().__init__()
        self._lines = deque(maxlen=self.MAX_LINES)
        self._dirty = False
        self._active_dir_filter: set[str] = set()
        self._active_id_filter: set[str] = set()
        self._build()
        self._install_handler()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._flush)
        self._timer.start(200)

    def _build(self):
        outer = QVBoxLayout(self)

        group = QGroupBox("Log")
        g_layout = QVBoxLayout(group)

        # Toolbar — filter combos + Debug checkbox on the same row
        toolbar = QHBoxLayout()

        # Filter widgets (hidden until Debug is checked)
        filter_inner = QHBoxLayout()
        filter_inner.setContentsMargins(0, 0, 0, 0)
        filter_inner.addWidget(QLabel("Direction:"))
        self._dir_combo = CheckableComboBox("All")
        self._dir_combo.setFixedWidth(110)
        self._dir_combo.addCheckItem("TX", "TX")
        self._dir_combo.addCheckItem("RX", "RX")
        self._dir_combo.selection_changed.connect(self._on_filter_changed)
        filter_inner.addWidget(self._dir_combo)

        filter_inner.addSpacing(10)
        filter_inner.addWidget(QLabel("ID:"))
        self._id_combo = CheckableComboBox("All")
        self._id_combo.setFixedWidth(195)
        for label, value in _ID_OPTIONS:
            self._id_combo.addCheckItem(label, value)
        self._id_combo.selection_changed.connect(self._on_filter_changed)
        filter_inner.addWidget(self._id_combo)

        self._filter_widget = QWidget()
        self._filter_widget.setLayout(filter_inner)
        self._filter_widget.setVisible(False)
        toolbar.addWidget(self._filter_widget)

        toolbar.addStretch()
        self.debug_chk = QCheckBox("Debug")
        self.debug_chk.setToolTip("Ghi tất cả CAN TX/RX kể cả gửi nhận định kỳ")
        self.debug_chk.toggled.connect(self._on_debug_toggled)
        toolbar.addWidget(self.debug_chk)
        g_layout.addLayout(toolbar)

        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(self.MAX_LINES)
        font = self.text.font()
        font.setFamily("Consolas")
        font.setPointSize(9)
        self.text.setFont(font)
        g_layout.addWidget(self.text)

        outer.addWidget(group)

    def _install_handler(self):
        self._handler = _SignalLogHandler()
        fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s",
                                datefmt="%H:%M:%S")
        self._handler.setFormatter(fmt)
        self._handler.setLevel(logging.INFO)
        self._handler.log_emitted.connect(self._append)

        self._can_filter = _CanLogFilter()
        root = logging.getLogger()
        for h in root.handlers:
            if isinstance(h, logging.StreamHandler) and not isinstance(h, _SignalLogHandler):
                h.addFilter(self._can_filter)

        root.addHandler(self._handler)

    def _on_debug_toggled(self, checked: bool):
        level = logging.DEBUG if checked else logging.INFO
        self._handler.setLevel(level)
        logging.getLogger().setLevel(logging.DEBUG if checked else logging.INFO)
        self._filter_widget.setVisible(checked)

    def _on_filter_changed(self):
        self._active_dir_filter = set(self._dir_combo.checkedValues())
        self._active_id_filter = {v.lower() for v in self._id_combo.checkedValues()}
        self._can_filter.dir_filter = self._active_dir_filter
        self._can_filter.id_filter = self._active_id_filter
        self._lines.clear()   # old messages no longer relevant to new filter
        self._dirty = True
        self._flush()

    def _passes_filter(self, msg: str) -> bool:
        if not self._active_dir_filter and not self._active_id_filter:
            return True
        m = _CAN_RE.search(msg)
        if not m:
            return True  # non-CAN messages (INFO/WARNING/ERROR) always shown
        direction, msg_id = m.group(1), m.group(2).lower()
        if self._active_dir_filter and direction not in self._active_dir_filter:
            return False
        if self._active_id_filter and msg_id not in self._active_id_filter:
            return False
        return True

    def _append(self, msg: str):
        if self._passes_filter(msg):
            self._lines.append(msg)
            self._dirty = True

    def _flush(self):
        if not self._dirty:
            return
        self._dirty = False
        self.text.setPlainText("\n".join(self._lines))
        self.text.verticalScrollBar().setValue(
            self.text.verticalScrollBar().maximum()
        )
