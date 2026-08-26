"""Bottom scrolling command/event log."""

import time

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QPlainTextEdit

_MAX_LINES = 1000


class LogPanel(QPlainTextEdit):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(_MAX_LINES)
        self.setFixedHeight(140)
        self.setFocusPolicy(Qt.NoFocus)  # never steal WASD/arrow/space

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.appendPlainText(f"[{timestamp}] {message}")
