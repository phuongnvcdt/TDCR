"""Application entry point."""
import logging
import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

try:
    from odri_can.ui.main_window import MainWindow
except ImportError:
    # Support running as a script from the project root or from src/odri_can directly.
    src_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(src_root))
    from odri_can.ui.main_window import MainWindow


def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def main():
    setup_logging()
    app = QApplication(sys.argv)
    app.setApplicationName("Tendon-Driven Continuum Robot Controller")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
