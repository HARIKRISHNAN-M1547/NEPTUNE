"""
Project NEPTUNE - Topside Control GUI (layout mockup)
Run:  pip install PyQt6   then   python neptune_gui.py

Layout:
  LEFT  : Video & Vision panel (camera feed + YOLO boxes later)
  RIGHT : Control panel (direction buttons + thruster slider)
          Telemetry panel (depth, water temp, Ethernet status)

Next steps (hooks are already in place):
  1. Socket communication  -> connect to command_sent / power_changed signals
  2. Camera streaming + AI -> replace VideoPanel placeholder with frames
  3. SQLite logging        -> log inside update_telemetry()
"""

import random
import sys

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

STYLE = """
QMainWindow, QWidget { background-color: #0f1720; color: #e6edf3; }
QGroupBox {
    border: 1px solid #243447; border-radius: 8px;
    margin-top: 14px; padding: 12px; font-weight: bold;
}
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #4fc3f7; }
QPushButton {
    background-color: #1b2a3a; border: 1px solid #2f4760; border-radius: 8px;
    padding: 14px; font-size: 14px; font-weight: bold;
}
QPushButton:hover   { background-color: #24405a; }
QPushButton:pressed { background-color: #4fc3f7; color: #0f1720; }
QPushButton#stop    { background-color: #7a1f1f; border-color: #b33a3a; }
QPushButton#stop:hover { background-color: #a02a2a; }
QSlider::groove:horizontal { height: 8px; background: #243447; border-radius: 4px; }
QSlider::sub-page:horizontal { background: #4fc3f7; border-radius: 4px; }
QSlider::handle:horizontal {
    background: #e6edf3; width: 20px; margin: -6px 0; border-radius: 10px;
}
"""


class VideoPanel(QGroupBox):
    """Left side: large area for camera feed and YOLO bounding boxes."""

    def __init__(self):
        super().__init__("Video & Vision")
        layout = QVBoxLayout(self)

        self.feed = QLabel("NO CAMERA SIGNAL")
        self.feed.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.feed.setMinimumSize(640, 480)
        self.feed.setFont(QFont("Arial", 18, QFont.Weight.Bold))
        self.feed.setStyleSheet(
            "background-color: #05090d; border: 2px dashed #2f4760; color: #4a6076;"
        )
        layout.addWidget(self.feed, stretch=1)

        self.detection_label = QLabel("Detections: 0")
        self.detection_label.setStyleSheet("color: #9fb3c8;")
        layout.addWidget(self.detection_label)


class ControlPanel(QGroupBox):
    """Right side: direction buttons + thruster power slider."""

    command_sent = pyqtSignal(str)   # e.g. "FORWARD"
    power_changed = pyqtSignal(int)  # 0-100

    def __init__(self):
        super().__init__("Control Panel")
        layout = QVBoxLayout(self)

        grid = QGridLayout()
        self.btn_forward = QPushButton("▲  FORWARD")
        self.btn_backward = QPushButton("▼  BACKWARD")
        self.btn_left = QPushButton("◄  LEFT")
        self.btn_right = QPushButton("RIGHT  ►")
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setObjectName("stop")

        grid.addWidget(self.btn_forward, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_backward, 2, 1)
        layout.addLayout(grid)

        for btn, cmd in [
            (self.btn_forward, "FORWARD"),
            (self.btn_backward, "BACKWARD"),
            (self.btn_left, "LEFT"),
            (self.btn_right, "RIGHT"),
            (self.btn_stop, "STOP"),
        ]:
            btn.clicked.connect(lambda _=False, c=cmd: self.command_sent.emit(c))

        layout.addSpacing(10)
        power_row = QHBoxLayout()
        power_row.addWidget(QLabel("Thruster Power"))
        self.power_value = QLabel("0 %")
        self.power_value.setAlignment(Qt.AlignmentFlag.AlignRight)
        power_row.addWidget(self.power_value)
        layout.addLayout(power_row)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self._on_power)
        layout.addWidget(self.slider)

    def _on_power(self, value: int):
        self.power_value.setText(f"{value} %")
        self.power_changed.emit(value)


class TelemetryPanel(QGroupBox):
    """Right side: depth, water temperature, Ethernet status."""

    def __init__(self):
        super().__init__("Telemetry")
        grid = QGridLayout(self)
        big = QFont("Arial", 20, QFont.Weight.Bold)

        self.depth = QLabel("-- m")
        self.temp = QLabel("-- °C")
        self.link = QLabel("DISCONNECTED")
        for w in (self.depth, self.temp, self.link):
            w.setFont(big)
        self.set_link(False)

        grid.addWidget(QLabel("Depth"), 0, 0)
        grid.addWidget(self.depth, 0, 1)
        grid.addWidget(QLabel("Water Temp"), 1, 0)
        grid.addWidget(self.temp, 1, 1)
        grid.addWidget(QLabel("Ethernet"), 2, 0)
        grid.addWidget(self.link, 2, 1)

    def set_link(self, connected: bool):
        self.link.setText("CONNECTED" if connected else "DISCONNECTED")
        color = "#4caf50" if connected else "#e53935"
        self.link.setStyleSheet(f"color: {color};")

    def update_values(self, depth_m: float, temp_c: float):
        self.depth.setText(f"{depth_m:.1f} m")
        self.temp.setText(f"{temp_c:.1f} °C")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NEPTUNE - Dam Inspection ROV")
        self.resize(1200, 700)

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        self.video = VideoPanel()
        self.controls = ControlPanel()
        self.telemetry = TelemetryPanel()

        right = QVBoxLayout()
        right.addWidget(self.controls)
        right.addWidget(self.telemetry)
        right.addStretch()

        right_box = QWidget()
        right_box.setLayout(right)
        right_box.setFixedWidth(380)

        root.addWidget(self.video, stretch=1)
        root.addWidget(right_box)

        # Hooks for the next step (socket communication)
        self.controls.command_sent.connect(self.on_command)
        self.controls.power_changed.connect(self.on_power)

        # Fake telemetry so you can see the panel working (remove later)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.fake_telemetry)
        self.timer.start(1000)

        self.statusBar().showMessage("Ready - layout mockup")

    def on_command(self, cmd: str):
        print(f"[CMD] {cmd}")
        self.statusBar().showMessage(f"Command: {cmd}", 2000)

    def on_power(self, value: int):
        print(f"[POWER] {value}%")

    def fake_telemetry(self):
        self.telemetry.update_values(
            depth_m=random.uniform(2.0, 3.0), temp_c=random.uniform(26.0, 28.0)
        )

    def keyPressEvent(self, event):
        keys = {
            Qt.Key.Key_W: "FORWARD",
            Qt.Key.Key_S: "BACKWARD",
            Qt.Key.Key_A: "LEFT",
            Qt.Key.Key_D: "RIGHT",
            Qt.Key.Key_Space: "STOP",
        }
        cmd = keys.get(event.key())
        if cmd:
            self.controls.command_sent.emit(cmd)
        else:
            super().keyPressEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())