"""
Project NEPTUNE - Topside Control GUI v2 (visual upgrade)
Run:  pip install PyQt6   then   python neptune_gui_v2.py

What's new vs v1:
  - Header bar with title, live clock and connection pill
  - HUD-style video panel (grid, corner brackets, crosshair, blinking REC)
  - Telemetry as cards, with a depth bar
  - Event log (detections / commands will appear here later)
  - Glow/gradient styling, hold-friendly big direction pad

Hooks kept for later:
  command_sent / power_changed signals -> socket communication
  HudVideo.set_frame(QImage)           -> camera stream + YOLO boxes
  MainWindow.log_event(text, level)    -> detections / database logging
"""

import random
import sys
from datetime import datetime

from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import (
    QColor,
    QFont,
    QImage,
    QLinearGradient,
    QPainter,
    QPen,
    QPixmap,
)
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

ACCENT = "#22d3ee"      # cyan
ACCENT_DIM = "#0e7490"
BG = "#0a1018"
CARD = "#111b27"
BORDER = "#1e2f42"
TEXT = "#e6edf3"
MUTED = "#7b8fa3"
GOOD = "#22c55e"
BAD = "#ef4444"
WARN = "#f59e0b"

STYLE = f"""
QMainWindow, QWidget {{ background-color: {BG}; color: {TEXT}; font-family: 'Segoe UI', Arial; }}

QFrame#card {{
    background-color: {CARD};
    border: 1px solid {BORDER};
    border-radius: 12px;
}}
QLabel#cardTitle {{
    color: {ACCENT}; font-size: 11px; font-weight: bold; letter-spacing: 2px;
    background: transparent;
}}
QLabel#caption {{ color: {MUTED}; font-size: 11px; background: transparent; }}
QLabel#value   {{ color: {TEXT}; font-size: 26px; font-weight: bold; background: transparent; }}

QFrame#header {{
    background-color: {CARD};
    border: 1px solid {BORDER};
    border-radius: 12px;
}}
QLabel#appTitle {{ font-size: 20px; font-weight: bold; letter-spacing: 4px; color: {TEXT}; background: transparent; }}
QLabel#appSub   {{ font-size: 11px; color: {MUTED}; background: transparent; }}
QLabel#clock    {{ font-size: 15px; color: {MUTED}; background: transparent; }}

QPushButton#dir {{
    background-color: #162536;
    border: 1px solid #27405a;
    border-radius: 12px;
    padding: 16px 6px;
    font-size: 12px;
    font-weight: bold;
}}
QPushButton#dir:hover   {{ background-color: #1d3550; border-color: {ACCENT_DIM}; }}
QPushButton#dir:pressed {{ background-color: {ACCENT}; color: {BG}; border-color: {ACCENT}; }}

QPushButton#stop {{
    background-color: #5b1818;
    border: 1px solid {BAD};
    border-radius: 12px;
    padding: 16px;
    font-size: 14px;
    font-weight: bold;
    letter-spacing: 2px;
}}
QPushButton#stop:hover   {{ background-color: #7f1d1d; }}
QPushButton#stop:pressed {{ background-color: {BAD}; color: white; }}

QSlider::groove:horizontal {{ height: 8px; background: #1b2a3b; border-radius: 4px; }}
QSlider::sub-page:horizontal {{
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {ACCENT_DIM}, stop:1 {ACCENT});
    border-radius: 4px;
}}
QSlider::handle:horizontal {{
    background: {TEXT}; width: 22px; margin: -7px 0; border-radius: 11px;
    border: 2px solid {ACCENT};
}}

QProgressBar {{
    background-color: #1b2a3b; border: none; border-radius: 5px;
    height: 10px; text-align: center; color: transparent;
}}
QProgressBar::chunk {{
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {ACCENT_DIM}, stop:1 {ACCENT});
    border-radius: 5px;
}}

QListWidget {{
    background-color: #0d1621; border: 1px solid {BORDER}; border-radius: 8px;
    padding: 4px; font-family: Consolas, 'Courier New', monospace; font-size: 11px;
}}
QListWidget::item {{ padding: 2px 4px; }}
QStatusBar {{ color: {MUTED}; }}
"""


def make_card(title: str) -> tuple[QFrame, QVBoxLayout]:
    """A rounded card with a small cyan title. Returns (card, body_layout)."""
    card = QFrame()
    card.setObjectName("card")
    outer = QVBoxLayout(card)
    outer.setContentsMargins(16, 12, 16, 14)
    outer.setSpacing(8)
    t = QLabel(title.upper())
    t.setObjectName("cardTitle")
    outer.addWidget(t)
    return card, outer


# --------------------------------------------------------------------------
# Video panel with HUD overlay
# --------------------------------------------------------------------------
class HudVideo(QWidget):
    """Paints the camera frame (when available) plus a HUD overlay."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(640, 400)
        self._frame: QPixmap | None = None
        self._rec_on = True
        self.depth_text = "-- m"
        self.detections = 0

        blink = QTimer(self)
        blink.timeout.connect(self._blink)
        blink.start(600)

    def _blink(self):
        self._rec_on = not self._rec_on
        self.update()

    def set_frame(self, image: QImage):
        """Call this later with each camera frame."""
        self._frame = QPixmap.fromImage(image)
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)

        # background
        grad = QLinearGradient(r.topLeft(), r.bottomRight())
        grad.setColorAt(0, QColor("#04080d"))
        grad.setColorAt(1, QColor("#0a1622"))
        p.setBrush(grad)
        p.setPen(QPen(QColor(BORDER), 1))
        p.drawRoundedRect(r, 12, 12)

        if self._frame:
            scaled = self._frame.scaled(
                self.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
            x = (self.width() - scaled.width()) // 2
            y = (self.height() - scaled.height()) // 2
            p.drawPixmap(x, y, scaled)
        else:
            # faint grid
            p.setPen(QPen(QColor(255, 255, 255, 12), 1))
            step = 40
            for gx in range(int(r.left()), int(r.right()), step):
                p.drawLine(gx, int(r.top()), gx, int(r.bottom()))
            for gy in range(int(r.top()), int(r.bottom()), step):
                p.drawLine(int(r.left()), gy, int(r.right()), gy)

            p.setPen(QColor("#3b5068"))
            p.setFont(QFont("Segoe UI", 20, QFont.Weight.Bold))
            p.drawText(r.adjusted(0, 90, 0, 0), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter, "NO CAMERA SIGNAL")

        # corner brackets
        pen = QPen(QColor(ACCENT), 2)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        m, L = 16, 28
        l, t, rr, b = r.left() + m, r.top() + m, r.right() - m, r.bottom() - m
        for (cx, cy, dx, dy) in [(l, t, 1, 1), (rr, t, -1, 1), (l, b, 1, -1), (rr, b, -1, -1)]:
            p.drawLine(QPointF(cx, cy), QPointF(cx + dx * L, cy))
            p.drawLine(QPointF(cx, cy), QPointF(cx, cy + dy * L))

        # center crosshair
        c = r.center()
        p.setPen(QPen(QColor(34, 211, 238, 140), 1))
        p.drawLine(QPointF(c.x() - 18, c.y()), QPointF(c.x() - 6, c.y()))
        p.drawLine(QPointF(c.x() + 6, c.y()), QPointF(c.x() + 18, c.y()))
        p.drawLine(QPointF(c.x(), c.y() - 18), QPointF(c.x(), c.y() - 6))
        p.drawLine(QPointF(c.x(), c.y() + 6), QPointF(c.x(), c.y() + 18))
        p.drawEllipse(c, 3, 3)

        # REC indicator (top-left)
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        if self._rec_on:
            p.setBrush(QColor(BAD))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(QPointF(l + 14, t + 24), 5, 5)
        p.setPen(QColor(TEXT))
        p.drawText(QPointF(l + 26, t + 28), "REC")

        # depth + detections (bottom-left), time (top-right)
        p.setPen(QColor(ACCENT))
        p.drawText(QPointF(l + 6, b - 24), f"DEPTH  {self.depth_text}")
        p.drawText(QPointF(l + 6, b - 8), f"DETECTIONS  {self.detections}")
        p.setPen(QColor(MUTED))
        stamp = datetime.now().strftime("%d-%m-%Y  %H:%M:%S")
        p.drawText(
            QRectF(r.left(), t + 12, r.width() - m - 6, 24),
            Qt.AlignmentFlag.AlignRight,
            stamp,
        )
        p.end()


# --------------------------------------------------------------------------
# Header
# --------------------------------------------------------------------------
class Header(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("header")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(18, 10, 18, 10)

        titles = QVBoxLayout()
        titles.setSpacing(0)
        t = QLabel("◈  NEPTUNE")
        t.setObjectName("appTitle")
        s = QLabel("Dam Inspection ROV  ·  Topside Control")
        s.setObjectName("appSub")
        titles.addWidget(t)
        titles.addWidget(s)
        lay.addLayout(titles)
        lay.addStretch()

        self.clock = QLabel()
        self.clock.setObjectName("clock")
        lay.addWidget(self.clock)
        lay.addSpacing(20)

        self.pill = QLabel()
        self.pill.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pill.setFixedHeight(30)
        self.pill.setMinimumWidth(170)
        lay.addWidget(self.pill)
        self.set_link(False)

        timer = QTimer(self)
        timer.timeout.connect(self._tick)
        timer.start(1000)
        self._tick()

    def _tick(self):
        self.clock.setText(datetime.now().strftime("%H:%M:%S"))

    def set_link(self, connected: bool):
        color = GOOD if connected else BAD
        text = "●  ETHERNET LINKED" if connected else "●  NO LINK"
        self.pill.setText(text)
        self.pill.setStyleSheet(
            f"color: {color}; background: transparent; border: 1px solid {color};"
            "border-radius: 15px; font-weight: bold; font-size: 11px; letter-spacing: 1px;"
        )


# --------------------------------------------------------------------------
# Control panel
# --------------------------------------------------------------------------
class ControlPanel(QFrame):
    command_sent = pyqtSignal(str)
    power_changed = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        card, body = make_card("Thruster Control")
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.addWidget(card)

        grid = QGridLayout()
        grid.setSpacing(10)
        specs = [
            ("▲  FORWARD", "FORWARD", 0, 1, "dir"),
            ("◄  LEFT", "LEFT", 1, 0, "dir"),
            ("■  STOP", "STOP", 1, 1, "stop"),
            ("RIGHT  ►", "RIGHT", 1, 2, "dir"),
            ("▼  BACKWARD", "BACKWARD", 2, 1, "dir"),
        ]
        for label, cmd, row, col, kind in specs:
            b = QPushButton(label)
            b.setObjectName(kind)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda _=False, c=cmd: self.command_sent.emit(c))
            grid.addWidget(b, row, col)
        body.addLayout(grid)

        body.addSpacing(6)
        row = QHBoxLayout()
        cap = QLabel("THRUSTER POWER")
        cap.setObjectName("caption")
        self.power_value = QLabel("0 %")
        self.power_value.setStyleSheet(
            f"color: {ACCENT}; font-weight: bold; font-size: 15px; background: transparent;"
        )
        row.addWidget(cap)
        row.addStretch()
        row.addWidget(self.power_value)
        body.addLayout(row)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.valueChanged.connect(self._on_power)
        body.addWidget(self.slider)

    def _on_power(self, v: int):
        self.power_value.setText(f"{v} %")
        self.power_changed.emit(v)


# --------------------------------------------------------------------------
# Telemetry
# --------------------------------------------------------------------------
class TelemetryPanel(QFrame):
    MAX_DEPTH = 10.0

    def __init__(self):
        super().__init__()
        card, body = make_card("Telemetry")
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.addWidget(card)

        # depth
        cap = QLabel("DEPTH")
        cap.setObjectName("caption")
        self.depth = QLabel("-- m")
        self.depth.setObjectName("value")
        body.addWidget(cap)
        body.addWidget(self.depth)
        self.bar = QProgressBar()
        self.bar.setRange(0, 100)
        self.bar.setTextVisible(False)
        body.addWidget(self.bar)

        body.addSpacing(8)

        # temperature
        cap2 = QLabel("WATER TEMPERATURE")
        cap2.setObjectName("caption")
        self.temp = QLabel("-- °C")
        self.temp.setObjectName("value")
        body.addWidget(cap2)
        body.addWidget(self.temp)

    def update_values(self, depth_m: float, temp_c: float):
        self.depth.setText(f"{depth_m:.1f} m")
        self.temp.setText(f"{temp_c:.1f} °C")
        self.bar.setValue(int(min(depth_m / self.MAX_DEPTH, 1.0) * 100))


# --------------------------------------------------------------------------
# Event log
# --------------------------------------------------------------------------
class EventLog(QFrame):
    def __init__(self):
        super().__init__()
        card, body = make_card("Event Log")
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.addWidget(card)
        self.list = QListWidget()
        body.addWidget(self.list)

    def add(self, text: str, level: str = "info"):
        from PyQt6.QtWidgets import QListWidgetItem

        color = {"info": MUTED, "cmd": ACCENT, "warn": WARN, "alert": BAD}.get(level, MUTED)
        item = QListWidgetItem(f"{datetime.now():%H:%M:%S}  {text}")
        item.setForeground(QColor(color))
        self.list.addItem(item)
        self.list.scrollToBottom()
        if self.list.count() > 200:
            self.list.takeItem(0)


# --------------------------------------------------------------------------
# Main window
# --------------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NEPTUNE - Dam Inspection ROV")
        self.resize(1280, 760)

        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(14, 14, 14, 8)
        outer.setSpacing(12)

        self._held_keys: list = []
        self.header = Header()
        outer.addWidget(self.header)

        body = QHBoxLayout()
        body.setSpacing(12)
        outer.addLayout(body, stretch=1)

        # left: video + log
        left = QVBoxLayout()
        left.setSpacing(12)
        self.video = HudVideo()
        left.addWidget(self.video, stretch=4)
        self.log = EventLog()
        self.log.setMaximumHeight(170)
        left.addWidget(self.log, stretch=1)
        body.addLayout(left, stretch=1)

        # right: controls + telemetry
        right = QVBoxLayout()
        right.setSpacing(12)
        self.controls = ControlPanel()
        self.telemetry = TelemetryPanel()
        right.addWidget(self.controls)
        right.addWidget(self.telemetry)
        right.addStretch()
        right_box = QWidget()
        right_box.setLayout(right)
        right_box.setFixedWidth(400)
        body.addWidget(right_box)

        # keep keyboard focus on the window so arrow keys never move the slider
        for wdg in self.findChildren(QPushButton) + self.findChildren(QSlider):
            wdg.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setFocus()

        # hooks for the socket step
        self.controls.command_sent.connect(self.on_command)
        self.controls.power_changed.connect(self.on_power)

        # fake telemetry (remove when real data arrives)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.fake_telemetry)
        self.timer.start(1000)

        self.statusBar().showMessage("Arrow keys: drive (hold)  ·  Space: STOP  ·  + / - : thruster power")
        self.log_event("GUI started", "info")

    # ---- public helpers -------------------------------------------------
    def log_event(self, text: str, level: str = "info"):
        self.log.add(text, level)

    def set_connected(self, connected: bool):
        self.header.set_link(connected)
        self.log_event("Ethernet linked" if connected else "Ethernet link lost",
                       "info" if connected else "warn")

    # ---- slots ----------------------------------------------------------
    def on_command(self, cmd: str):
        print(f"[CMD] {cmd}")
        self.log_event(f"Command: {cmd}", "cmd")

    def on_power(self, value: int):
        print(f"[POWER] {value}%")

    def fake_telemetry(self):
        depth = random.uniform(2.0, 3.0)
        temp = random.uniform(26.0, 28.0)
        self.telemetry.update_values(depth, temp)
        self.video.depth_text = f"{depth:.1f} m"
        self.video.update()

    # ---- keyboard control ----------------------------------------------
    # Arrow keys drive, Space = STOP, +/- change thruster power.
    # HOLD_TO_DRIVE = True : ROV moves only while the key is held and stops
    #                        when released (safer). False = click-style: one
    #                        press keeps moving until you press Space.
    HOLD_TO_DRIVE = True

    KEY_COMMANDS = {
        Qt.Key.Key_Up: "FORWARD",
        Qt.Key.Key_Down: "BACKWARD",
        Qt.Key.Key_Left: "LEFT",
        Qt.Key.Key_Right: "RIGHT",
    }

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in self.KEY_COMMANDS:
            self._held_keys.append(key)
            self.controls.command_sent.emit(self.KEY_COMMANDS[key])
        elif key == Qt.Key.Key_Space:
            self._held_keys.clear()
            self.controls.command_sent.emit("STOP")
        elif key in (Qt.Key.Key_Plus, Qt.Key.Key_Equal):
            self.controls.slider.setValue(self.controls.slider.value() + 5)
        elif key in (Qt.Key.Key_Minus, Qt.Key.Key_Underscore):
            self.controls.slider.setValue(self.controls.slider.value() - 5)
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if self.HOLD_TO_DRIVE and key in self.KEY_COMMANDS:
            if key in self._held_keys:
                self._held_keys.remove(key)
            # another arrow still held -> keep going that way, else stop
            if self._held_keys:
                self.controls.command_sent.emit(self.KEY_COMMANDS[self._held_keys[-1]])
            else:
                self.controls.command_sent.emit("STOP")
        else:
            super().keyReleaseEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())