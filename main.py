"""
Project NEPTUNE - Topside Control GUI v4
=========================================
Run:   pip install PyQt6      then      python neptune_gui_v4.py

Optional (only for real YOLO):   pip install ultralytics opencv-python

LAYOUT
  Header       : title, mission ID + timer, clock, heartbeat, link status
  Top strip    : depth, heading, range L/R, battery, safety status
  Left         : live video with HUD, YOLO boxes, laser spots, event log
  Right (tabs) : CONTROL  - drive pad, heave/pitch, power, thruster bars, payload
                 WATER    - pH, ORP, TDS, turbidity, temperature (+ raw ADC)
                 SENSORS  - health chips, attitude dial, compass, IMU numbers,
                            depth graph, vision info

KEYBOARD (hold to move, release = STOP)
  Arrow keys : Up/Down = surge forward/back,  Left/Right = yaw
  W / S      : ascend / descend (heave)
  Q / E      : pitch up / pitch down
  Space      : STOP        + / -  : thruster power +-5 %

MOCK MODE (MOCK_MODE = True) generates fake sensors and a fake crack so you
can test the whole GUI without hardware. Hold S to dive and watch the depth
alarm, hold Up to approach the wall and watch the range alarm.

CONNECTING REAL DATA (later, after the socket step)
  window.on_telemetry(packet)      packet = dict, see PACKET FORMAT below
  window.on_detections(list)       [{"cls": "crack", "conf": 0.87,
                                     "box": (x1, y1, x2, y2)}]  (0..1 coords)
  window.video.set_frame(QImage)   camera frame
  window.video.set_lasers(p1, p2)  laser spots (0..1 coords) -> scale
  window.controls.command_sent     signal(str) -> send over socket
  window.controls.power_changed    signal(int)
  window.controls.servo_changed    signal(int)

PACKET FORMAT (any group / key may be missing)
  {
   "imu":      {"roll","pitch","yaw","ax","ay","az","gx","gy","gz"},   deg, g, deg/s
   "depth":    {"depth_m","pressure_mbar"},
   "range":    {"left_m","right_m"},
   "water":    {"ph","orp_mv","tds_ppm","turbidity_ntu","temp_c",
                "raw": {"ph","orp","tds","turb"}},                    raw ADC counts
   "thrusters":{"port","stbd","fore","aft"},                          -100..100 %
   "battery":  {"v"},                                                 optional
  }
"""

import math
import os
import random
import sqlite3
import sys
import time
from collections import deque
from datetime import datetime

from PyQt6.QtCore import QEvent, QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import (
    QColor,
    QFont,
    QImage,
    QLinearGradient,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QPolygonF,
)
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSlider,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

# ==========================================================================
# CONFIG - edit these
# ==========================================================================
MOCK_MODE = True            # fake sensors + fake crack for testing
USE_YOLO = False            # True = run real YOLO on VIDEO_SOURCE
MODEL_PATH = "best.pt"      # your trained crack model (not available yet)
VIDEO_SOURCE = 0            # camera index or stream URL (e.g. rtsp://...)
HOLD_TO_DRIVE = True        # True: moves only while key/button is held
LASER_BASELINE_M = 0.100    # MEASURE the real spacing of your two lasers!
HEARTBEAT_TIMEOUT_S = 2.0   # no packet for this long = NO LINK
STALE_S = 2.5               # sensor group not updated for this long = stale
DET_COOLDOWN_S = 3.0        # min gap between logged detection events
DB_PATH = "neptune_log.db"
SNAP_DIR = "neptune_snapshots"

# Safety / alarm thresholds - placeholders, set them for your vehicle.
# The depth limit must be agreed with your supervisor after hull testing.
LIMITS = dict(
    MAX_DEPTH_M=10.0,       # depth bar full scale
    DEPTH_WARN_M=7.0,
    DEPTH_LIMIT_M=8.0,
    RANGE_WARN_M=0.60,      # distance to dam wall
    RANGE_CRIT_M=0.30,
    BATT_FULL_V=12.7,       # 12 V lead-acid (Amaron) - adjust if needed
    BATT_WARN_V=11.8,
    BATT_CRIT_V=11.4,
)

# ==========================================================================
# THEME
# ==========================================================================
ACCENT = "#22d3ee"
ACCENT_DIM = "#0e7490"
BG = "#0a1018"
CARD = "#111b27"
BORDER = "#1e2f42"
TEXT = "#e6edf3"
MUTED = "#7b8fa3"
GOOD = "#22c55e"
BAD = "#ef4444"
WARN = "#f59e0b"
PURPLE = "#a78bfa"

STYLE = f"""
QMainWindow, QWidget {{ background-color: {BG}; color: {TEXT}; font-family: 'Segoe UI', Arial; }}

QFrame#card {{ background-color: {CARD}; border: 1px solid {BORDER}; border-radius: 12px; }}
QLabel#cardTitle {{ color: {ACCENT}; font-size: 11px; font-weight: bold; letter-spacing: 2px; background: transparent; }}
QLabel#caption {{ color: {MUTED}; font-size: 11px; background: transparent; }}
QLabel#tileTitle {{ color: {MUTED}; font-size: 10px; font-weight: bold; letter-spacing: 1px; background: transparent; }}
QLabel#tileUnit  {{ color: {MUTED}; font-size: 12px; background: transparent; }}
QLabel#tileSub   {{ color: {MUTED}; font-size: 10px; font-family: Consolas, monospace; background: transparent; }}
QLabel#mono      {{ font-family: Consolas, 'Courier New', monospace; font-size: 13px; background: transparent; }}

QFrame#header {{ background-color: {CARD}; border: 1px solid {BORDER}; border-radius: 12px; }}
QLabel#appTitle {{ font-size: 20px; font-weight: bold; letter-spacing: 4px; background: transparent; }}
QLabel#appSub   {{ font-size: 11px; color: {MUTED}; background: transparent; }}
QLabel#clock    {{ font-size: 15px; color: {MUTED}; background: transparent; }}
QLabel#mission  {{ font-size: 12px; color: {ACCENT}; font-weight: bold; background: transparent; }}

QTabWidget::pane {{ border: 1px solid {BORDER}; border-radius: 12px; background: {CARD}; top: -1px; }}
QTabBar::tab {{ background: transparent; color: {MUTED}; padding: 9px 20px; font-weight: bold;
               letter-spacing: 1px; border-bottom: 2px solid transparent; }}
QTabBar::tab:selected {{ color: {ACCENT}; border-bottom: 2px solid {ACCENT}; }}
QTabBar::tab:hover {{ color: {TEXT}; }}

QScrollArea {{ border: none; background: transparent; }}
QScrollArea > QWidget {{ background: transparent; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}
QWidget#tabBody {{ background: transparent; }}
QScrollBar:vertical {{ background: transparent; width: 8px; }}
QScrollBar::handle:vertical {{ background: #243447; border-radius: 4px; min-height: 24px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0px; }}

QPushButton#dir {{ background-color: #162536; border: 1px solid #27405a; border-radius: 12px;
                   padding: 14px 6px; font-size: 12px; font-weight: bold; }}
QPushButton#dir:hover   {{ background-color: #1d3550; border-color: {ACCENT_DIM}; }}
QPushButton#dir:pressed {{ background-color: {ACCENT}; color: {BG}; border-color: {ACCENT}; }}
QPushButton#aux {{ background-color: #162536; border: 1px solid #27405a; border-radius: 10px;
                   padding: 9px 6px; font-size: 11px; font-weight: bold; }}
QPushButton#aux:hover   {{ background-color: #1d3550; border-color: {ACCENT_DIM}; }}
QPushButton#aux:pressed {{ background-color: {ACCENT}; color: {BG}; }}
QPushButton#stop {{ background-color: #5b1818; border: 1px solid {BAD}; border-radius: 12px;
                    padding: 14px 6px; font-size: 14px; font-weight: bold; letter-spacing: 2px; }}
QPushButton#stop:hover   {{ background-color: #7f1d1d; }}
QPushButton#stop:pressed {{ background-color: {BAD}; color: white; }}

QSlider::groove:horizontal {{ height: 8px; background: #1b2a3b; border-radius: 4px; }}
QSlider::sub-page:horizontal {{
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {ACCENT_DIM}, stop:1 {ACCENT});
    border-radius: 4px; }}
QSlider::handle:horizontal {{ background: {TEXT}; width: 22px; margin: -7px 0; border-radius: 11px;
                              border: 2px solid {ACCENT}; }}

QProgressBar {{ background-color: #1b2a3b; border: none; border-radius: 4px; text-align: center; color: transparent; }}
QProgressBar::chunk {{
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {ACCENT_DIM}, stop:1 {ACCENT});
    border-radius: 4px; }}

QListWidget {{ background-color: #0d1621; border: 1px solid {BORDER}; border-radius: 8px; padding: 4px;
               font-family: Consolas, 'Courier New', monospace; font-size: 11px; }}
QListWidget::item {{ padding: 2px 4px; }}
QStatusBar {{ color: {MUTED}; }}
"""


def make_card(title: str):
    """Rounded card with a small cyan title. Returns (card, body_layout)."""
    card = QFrame()
    card.setObjectName("card")
    outer = QVBoxLayout(card)
    outer.setContentsMargins(16, 12, 16, 14)
    outer.setSpacing(8)
    t = QLabel(title.upper())
    t.setObjectName("cardTitle")
    outer.addWidget(t)
    return card, outer


def section_title(text: str) -> QLabel:
    t = QLabel(text.upper())
    t.setObjectName("cardTitle")
    return t


def scrollable(widget: QWidget) -> QScrollArea:
    sa = QScrollArea()
    sa.setWidgetResizable(True)
    sa.setFrameShape(QFrame.Shape.NoFrame)
    sa.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    sa.setWidget(widget)
    return sa


def cardinal(deg: float) -> str:
    names = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    return names[int((deg % 360) / 45 + 0.5) % 8]


# ==========================================================================
# DATABASE
# ==========================================================================
class LogDB:
    """SQLite logging of telemetry and detections."""

    def __init__(self, path: str, mission: str):
        self.mission = mission
        self.con = sqlite3.connect(path)
        self.con.executescript(
            """
            CREATE TABLE IF NOT EXISTS telemetry(
                id INTEGER PRIMARY KEY AUTOINCREMENT, ts TEXT, mission TEXT,
                depth_m REAL, pressure_mbar REAL,
                roll REAL, pitch REAL, yaw REAL,
                range_left_m REAL, range_right_m REAL,
                ph REAL, orp_mv REAL, tds_ppm REAL, turbidity_ntu REAL, water_temp_c REAL,
                raw_ph INTEGER, raw_orp INTEGER, raw_tds INTEGER, raw_turb INTEGER,
                batt_v REAL);
            CREATE TABLE IF NOT EXISTS detections(
                id INTEGER PRIMARY KEY AUTOINCREMENT, ts TEXT, mission TEXT,
                cls TEXT, conf REAL, x1 REAL, y1 REAL, x2 REAL, y2 REAL,
                depth_m REAL, length_m REAL, snapshot TEXT);
            """
        )
        self.con.commit()

    @staticmethod
    def _g(state: dict, group: str, key: str):
        return state.get(group, {}).get(key)

    def log_telemetry(self, s: dict):
        g = lambda grp, k: self._g(s, grp, k)  # noqa: E731
        raw = s.get("water", {}).get("raw", {})
        self.con.execute(
            "INSERT INTO telemetry(ts,mission,depth_m,pressure_mbar,roll,pitch,yaw,"
            "range_left_m,range_right_m,ph,orp_mv,tds_ppm,turbidity_ntu,water_temp_c,"
            "raw_ph,raw_orp,raw_tds,raw_turb,batt_v) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
            (
                datetime.now().isoformat(timespec="seconds"), self.mission,
                g("depth", "depth_m"), g("depth", "pressure_mbar"),
                g("imu", "roll"), g("imu", "pitch"), g("imu", "yaw"),
                g("range", "left_m"), g("range", "right_m"),
                g("water", "ph"), g("water", "orp_mv"), g("water", "tds_ppm"),
                g("water", "turbidity_ntu"), g("water", "temp_c"),
                raw.get("ph"), raw.get("orp"), raw.get("tds"), raw.get("turb"),
                g("battery", "v"),
            ),
        )
        self.con.commit()

    def log_detection(self, cls, conf, box, depth_m, length_m, snapshot):
        self.con.execute(
            "INSERT INTO detections(ts,mission,cls,conf,x1,y1,x2,y2,depth_m,length_m,snapshot) "
            "VALUES (?,?,?,?,?,?,?,?,?,?,?)",
            (datetime.now().isoformat(timespec="seconds"), self.mission, cls, conf,
             box[0], box[1], box[2], box[3], depth_m, length_m, snapshot),
        )
        self.con.commit()

    def close(self):
        try:
            self.con.close()
        except Exception:
            pass


# ==========================================================================
# YOLO WORKER (optional, runs in its own thread so the GUI never freezes)
# ==========================================================================
class YoloWorker(QThread):
    frame_ready = pyqtSignal(QImage)
    detections_ready = pyqtSignal(list)
    fps_ready = pyqtSignal(float)
    error = pyqtSignal(str)

    def __init__(self, source, model_path, conf=0.4):
        super().__init__()
        self.source = source
        self.model_path = model_path
        self.conf = conf
        self._running = True

    def run(self):
        try:
            import cv2
            from ultralytics import YOLO
        except ImportError as e:
            self.error.emit(f"YOLO libraries missing: {e}. pip install ultralytics opencv-python")
            return
        try:
            model = YOLO(self.model_path)
        except Exception as e:  # missing / bad weights
            self.error.emit(f"Cannot load model '{self.model_path}': {e}")
            return
        cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            self.error.emit(f"Cannot open video source: {self.source}")
            return

        count, t0 = 0, time.monotonic()
        while self._running:
            ok, frame = cap.read()
            if not ok:
                self.msleep(50)
                continue
            result = model(frame, conf=self.conf, verbose=False)[0]
            h, w = frame.shape[:2]
            dets = []
            for b in result.boxes:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                dets.append({
                    "cls": result.names[int(b.cls[0])],
                    "conf": float(b.conf[0]),
                    "box": (x1 / w, y1 / h, x2 / w, y2 / h),
                })
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
            self.frame_ready.emit(img)
            self.detections_ready.emit(dets)

            count += 1
            if time.monotonic() - t0 >= 1.0:
                self.fps_ready.emit(count / (time.monotonic() - t0))
                count, t0 = 0, time.monotonic()
        cap.release()

    def stop(self):
        self._running = False
        self.wait(3000)


# ==========================================================================
# VIDEO PANEL WITH HUD, DETECTION BOXES, LASER SPOTS
# ==========================================================================
class HudVideo(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(560, 360)
        self._frame = None
        self.frame_size = (1280, 720)       # updated by set_frame
        self._dets = []
        self._lasers = None
        self.scale_m_per_px = None
        self._rec_on = True
        self.depth_text = "-- m"
        self.detections = 0
        self.mock = False

        blink = QTimer(self)
        blink.timeout.connect(self._blink)
        blink.start(600)

    # ---- inputs ----------------------------------------------------------
    def _blink(self):
        self._rec_on = not self._rec_on
        self.update()

    def set_frame(self, image: QImage):
        self._frame = QPixmap.fromImage(image)
        self.frame_size = (image.width(), image.height())
        self.update()

    def set_detections(self, dets: list):
        self._dets = dets
        self.update()

    def set_lasers(self, p1, p2, baseline_m: float = LASER_BASELINE_M):
        """p1, p2 = laser spot positions as (x, y) in 0..1 image coordinates."""
        self._lasers = (p1, p2)
        fw, fh = self.frame_size
        sep_px = math.hypot((p2[0] - p1[0]) * fw, (p2[1] - p1[1]) * fh)
        self.scale_m_per_px = baseline_m / sep_px if sep_px > 1 else None
        self.update()

    def box_length_m(self, box):
        if not self.scale_m_per_px:
            return None
        fw, fh = self.frame_size
        px = max((box[2] - box[0]) * fw, (box[3] - box[1]) * fh)
        return px * self.scale_m_per_px

    # ---- painting --------------------------------------------------------
    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)

        grad = QLinearGradient(r.topLeft(), r.bottomRight())
        grad.setColorAt(0, QColor("#04080d"))
        grad.setColorAt(1, QColor("#0a1622"))
        p.setBrush(grad)
        p.setPen(QPen(QColor(BORDER), 1))
        p.drawRoundedRect(r, 12, 12)

        disp = r                                    # area the image occupies
        if self._frame:
            scaled = self._frame.scaled(
                self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation)
            x = (self.width() - scaled.width()) // 2
            y = (self.height() - scaled.height()) // 2
            p.drawPixmap(x, y, scaled)
            disp = QRectF(x, y, scaled.width(), scaled.height())
        else:
            p.setPen(QPen(QColor(255, 255, 255, 12), 1))
            for gx in range(int(r.left()), int(r.right()), 40):
                p.drawLine(gx, int(r.top()), gx, int(r.bottom()))
            for gy in range(int(r.top()), int(r.bottom()), 40):
                p.drawLine(int(r.left()), gy, int(r.right()), gy)
            p.setPen(QColor("#3b5068"))
            p.setFont(QFont("Segoe UI", 18, QFont.Weight.Bold))
            p.drawText(r.adjusted(0, 110, 0, 0),
                       Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter,
                       "NO CAMERA SIGNAL")

        # laser spots + line
        if self._lasers:
            pts = [QPointF(disp.left() + q[0] * disp.width(),
                           disp.top() + q[1] * disp.height()) for q in self._lasers]
            p.setPen(QPen(QColor(0, 255, 120, 150), 1, Qt.PenStyle.DashLine))
            p.drawLine(pts[0], pts[1])
            for pt in pts:
                p.setBrush(QColor(0, 255, 120, 70))
                p.setPen(Qt.PenStyle.NoPen)
                p.drawEllipse(pt, 9, 9)
                p.setBrush(QColor(0, 255, 120))
                p.drawEllipse(pt, 4, 4)

        # YOLO boxes
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        fm = p.fontMetrics()
        for d in self._dets:
            x1, y1, x2, y2 = d["box"]
            rect = QRectF(disp.left() + x1 * disp.width(), disp.top() + y1 * disp.height(),
                          (x2 - x1) * disp.width(), (y2 - y1) * disp.height())
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(QColor(BAD), 2))
            p.drawRect(rect)
            label = f"{d['cls']} {d['conf']:.0%}"
            length = self.box_length_m(d["box"])
            if length is not None:
                label += f"  L~{length:.2f} m"
            tw = fm.horizontalAdvance(label) + 10
            th = fm.height() + 4
            ly = rect.top() - th if rect.top() - th > disp.top() else rect.top()
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(BAD))
            p.drawRect(QRectF(rect.left(), ly, tw, th))
            p.setPen(QColor("white"))
            p.drawText(QRectF(rect.left() + 5, ly, tw, th),
                       Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, label)

        # corner brackets
        p.setPen(QPen(QColor(ACCENT), 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        m, L = 16, 28
        l, t, rr, b = r.left() + m, r.top() + m, r.right() - m, r.bottom() - m
        for (cx, cy, dx, dy) in [(l, t, 1, 1), (rr, t, -1, 1), (l, b, 1, -1), (rr, b, -1, -1)]:
            p.drawLine(QPointF(cx, cy), QPointF(cx + dx * L, cy))
            p.drawLine(QPointF(cx, cy), QPointF(cx, cy + dy * L))

        # crosshair
        c = r.center()
        p.setPen(QPen(QColor(34, 211, 238, 140), 1))
        p.drawLine(QPointF(c.x() - 18, c.y()), QPointF(c.x() - 6, c.y()))
        p.drawLine(QPointF(c.x() + 6, c.y()), QPointF(c.x() + 18, c.y()))
        p.drawLine(QPointF(c.x(), c.y() - 18), QPointF(c.x(), c.y() - 6))
        p.drawLine(QPointF(c.x(), c.y() + 6), QPointF(c.x(), c.y() + 18))
        p.drawEllipse(c, 3, 3)

        # REC
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        if self._rec_on:
            p.setBrush(QColor(BAD))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(QPointF(l + 14, t + 24), 5, 5)
        p.setPen(QColor(TEXT))
        p.drawText(QPointF(l + 26, t + 28), "REC")
        if self.mock:
            p.setPen(QColor(WARN))
            p.drawText(QRectF(r.left(), t + 12, r.width(), 24),
                       Qt.AlignmentFlag.AlignHCenter, "MOCK DATA")

        # HUD text
        scale = (f"{self.scale_m_per_px * 1000:.2f} mm/px"
                 if self.scale_m_per_px else "--")
        p.setPen(QColor(ACCENT))
        p.drawText(QPointF(l + 6, b - 40), f"DEPTH  {self.depth_text}")
        p.drawText(QPointF(l + 6, b - 24), f"SCALE  {scale}")
        p.drawText(QPointF(l + 6, b - 8), f"DETECTIONS  {self.detections}")
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(r.left(), t + 12, r.width() - m - 6, 24),
                   Qt.AlignmentFlag.AlignRight,
                   datetime.now().strftime("%d-%m-%Y  %H:%M:%S"))
        p.end()


# ==========================================================================
# SMALL WIDGETS
# ==========================================================================
class StatTile(QFrame):
    """One reading: caption, big value, unit, optional bar and sub-text."""

    LEVEL_COLORS = {"ok": TEXT, "warn": WARN, "alert": BAD}

    def __init__(self, title: str, unit: str = "", bar: bool = False, sub: bool = False):
        super().__init__()
        self.setObjectName("tile")
        self.setMinimumHeight(92)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(2)

        cap = QLabel(title.upper())
        cap.setObjectName("tileTitle")
        lay.addWidget(cap)

        row = QHBoxLayout()
        row.setSpacing(4)
        self.value = QLabel("--")
        self.value.setFont(QFont("Segoe UI", 20, QFont.Weight.Bold))
        self.unit = QLabel(unit)
        self.unit.setObjectName("tileUnit")
        self.unit.setAlignment(Qt.AlignmentFlag.AlignBottom)
        row.addWidget(self.value)
        row.addWidget(self.unit)
        row.addStretch()
        lay.addLayout(row)

        self.sub = None
        if sub:
            self.sub = QLabel("raw --")
            self.sub.setObjectName("tileSub")
            lay.addWidget(self.sub)

        self.bar = None
        if bar:
            self.bar = QProgressBar()
            self.bar.setRange(0, 100)
            self.bar.setTextVisible(False)
            self.bar.setFixedHeight(8)
            lay.addWidget(self.bar)
        else:
            lay.addStretch()

        self._apply("ok", MUTED)

    def _apply(self, level: str, color):
        border = {"ok": BORDER, "warn": WARN, "alert": BAD}[level]
        self.setStyleSheet(
            f"QFrame#tile {{ background-color: {CARD}; border: 1px solid {border};"
            " border-radius: 12px; }"
        )
        c = color or self.LEVEL_COLORS[level]
        self.value.setStyleSheet(f"color: {c}; background: transparent;")

    def set_value(self, text, level="ok", bar_pct=None, color=None, sub_text=None):
        self.value.setText(text)
        self._apply(level, color)
        if self.bar is not None and bar_pct is not None:
            self.bar.setValue(max(0, min(100, int(bar_pct))))
        if self.sub is not None and sub_text is not None:
            self.sub.setText(sub_text)

    def set_unit(self, text: str):
        self.unit.setText(text)


class HealthChip(QLabel):
    def __init__(self, name: str):
        super().__init__(name)
        self.name = name
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedHeight(24)
        self.set_state("none")

    def set_state(self, state: str):
        color = {"ok": GOOD, "stale": BAD, "none": MUTED}[state]
        self.setStyleSheet(
            f"color: {color}; background: transparent; border: 1px solid {color};"
            "border-radius: 12px; font-size: 10px; font-weight: bold; padding: 0 8px;"
        )


class ThrusterBar(QWidget):
    """Bar from -100 % to +100 % with zero in the middle."""

    def __init__(self, name: str):
        super().__init__()
        self.name = name
        self.value = 0.0
        self.setFixedHeight(26)
        self.setMinimumWidth(150)

    def set_value(self, v):
        self.value = max(-100.0, min(100.0, float(v)))
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        label_w = 46
        bar = QRectF(label_w, 6, w - label_w - 40, h - 12)
        p.setPen(QColor(MUTED))
        p.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        p.drawText(QRectF(0, 0, label_w, h),
                   Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self.name)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor("#1b2a3b"))
        p.drawRoundedRect(bar, 4, 4)
        mid, half = bar.center().x(), bar.width() / 2
        frac = self.value / 100.0
        if frac >= 0:
            fill, color = QRectF(mid, bar.top(), half * frac, bar.height()), ACCENT
        else:
            fill, color = QRectF(mid + half * frac, bar.top(), half * -frac, bar.height()), WARN
        p.setBrush(QColor(color))
        p.drawRoundedRect(fill, 3, 3)
        p.setPen(QPen(QColor(TEXT), 1))
        p.drawLine(QPointF(mid, bar.top() - 2), QPointF(mid, bar.bottom() + 2))
        p.setPen(QColor(TEXT))
        p.drawText(QRectF(bar.right() + 4, 0, 36, h),
                   Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight,
                   f"{self.value:+.0f}")
        p.end()


class AttitudeWidget(QWidget):
    """Artificial horizon: shows roll and pitch."""

    def __init__(self):
        super().__init__()
        self.roll = 0.0
        self.pitch = 0.0
        self.setMinimumSize(160, 160)

    def set_attitude(self, roll: float, pitch: float):
        self.roll, self.pitch = roll, pitch
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        s = min(self.width(), self.height())
        R = s / 2 - 4
        cx, cy = self.width() / 2, self.height() / 2

        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), R, R)
        p.save()
        p.setClipPath(clip)
        p.translate(cx, cy)
        p.rotate(-self.roll)
        k = R / 40.0                         # pixels per degree of pitch
        p.translate(0, self.pitch * k)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor("#1d5f8f"))
        p.drawRect(QRectF(-3 * R, -6 * R, 6 * R, 6 * R))
        p.setBrush(QColor("#6b4423"))
        p.drawRect(QRectF(-3 * R, 0, 6 * R, 6 * R))
        p.setPen(QPen(QColor("white"), 2))
        p.drawLine(QPointF(-3 * R, 0), QPointF(3 * R, 0))
        p.setPen(QPen(QColor(255, 255, 255, 170), 1))
        for deg in (-20, -10, 10, 20):
            y = -deg * k
            half = R * (0.35 if abs(deg) == 20 else 0.2)
            p.drawLine(QPointF(-half, y), QPointF(half, y))
        p.restore()

        p.setPen(QPen(QColor(ACCENT), 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(cx, cy), R, R)
        p.setPen(QPen(QColor(WARN), 3))
        p.drawLine(QPointF(cx - R * 0.55, cy), QPointF(cx - R * 0.18, cy))
        p.drawLine(QPointF(cx + R * 0.18, cy), QPointF(cx + R * 0.55, cy))
        p.setBrush(QColor(WARN))
        p.drawEllipse(QPointF(cx, cy), 3, 3)
        p.end()


class CompassWidget(QWidget):
    """Rotating compass card showing heading (yaw)."""

    def __init__(self):
        super().__init__()
        self.heading = 0.0
        self.setMinimumSize(160, 160)

    def set_heading(self, deg: float):
        self.heading = deg % 360
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        s = min(self.width(), self.height())
        R = s / 2 - 4
        cx, cy = self.width() / 2, self.height() / 2

        p.setPen(QPen(QColor(BORDER), 2))
        p.setBrush(QColor("#0d1621"))
        p.drawEllipse(QPointF(cx, cy), R, R)

        p.save()
        p.translate(cx, cy)
        p.rotate(-self.heading)
        for deg in range(0, 360, 10):
            p.save()
            p.rotate(deg)
            long = deg % 30 == 0
            p.setPen(QPen(QColor(TEXT if long else MUTED), 1.5 if long else 1))
            p.drawLine(QPointF(0, -R + 3), QPointF(0, -R + (12 if long else 7)))
            p.restore()
        p.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        for deg, name in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            p.save()
            p.rotate(deg)
            p.setPen(QColor(ACCENT if name == "N" else TEXT))
            p.drawText(QRectF(-12, -R + 14, 24, 18), Qt.AlignmentFlag.AlignCenter, name)
            p.restore()
        p.restore()

        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(WARN))
        p.drawPolygon(QPolygonF([QPointF(cx - 6, cy - R - 1), QPointF(cx + 6, cy - R - 1),
                                 QPointF(cx, cy - R + 10)]))
        p.setPen(QColor(TEXT))
        p.setFont(QFont("Consolas", 13, QFont.Weight.Bold))
        p.drawText(QRectF(cx - 40, cy - 12, 80, 24), Qt.AlignmentFlag.AlignCenter,
                   f"{self.heading:.0f}°")
        p.end()


class Sparkline(QWidget):
    """Small live graph. invert=True draws larger values lower (for depth)."""

    def __init__(self, title: str, unit: str, maxlen: int = 150, invert: bool = False):
        super().__init__()
        self.title, self.unit, self.invert = title, unit, invert
        self.maxlen = maxlen
        self.data = deque(maxlen=maxlen)
        self.setMinimumHeight(110)

    def push(self, v: float):
        self.data.append(v)
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        p.setPen(QPen(QColor(BORDER), 1))
        p.setBrush(QColor("#0d1621"))
        p.drawRoundedRect(r, 8, 8)

        p.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(r.left() + 10, r.top() + 6, 200, 16),
                   Qt.AlignmentFlag.AlignLeft, self.title.upper())
        if len(self.data) < 2:
            p.end()
            return

        lo, hi = min(self.data), max(self.data)
        if hi - lo < 0.5:
            mid = (hi + lo) / 2
            lo, hi = mid - 0.25, mid + 0.25
        area = r.adjusted(10, 26, -10, -10)
        path = QPainterPath()
        for i, v in enumerate(self.data):
            frac = (v - lo) / (hi - lo)
            x = area.left() + i * area.width() / (self.maxlen - 1)
            y = area.top() + frac * area.height() if self.invert else area.bottom() - frac * area.height()
            if i == 0:
                path.moveTo(x, y)
            else:
                path.lineTo(x, y)
        p.setPen(QPen(QColor(ACCENT), 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPath(path)
        p.setPen(QColor(TEXT))
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        p.drawText(QRectF(r.left(), r.top() + 6, r.width() - 10, 16),
                   Qt.AlignmentFlag.AlignRight, f"{self.data[-1]:.2f} {self.unit}")
        p.end()


# ==========================================================================
# HEADER
# ==========================================================================
class Header(QFrame):
    def __init__(self, mission_id: str):
        super().__init__()
        self.setObjectName("header")
        self._t0 = time.monotonic()
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
        lay.addSpacing(30)

        mission = QVBoxLayout()
        mission.setSpacing(0)
        self.mission = QLabel(f"MISSION  {mission_id}")
        self.mission.setObjectName("mission")
        self.elapsed = QLabel("00:00:00")
        self.elapsed.setObjectName("appSub")
        mission.addWidget(self.mission)
        mission.addWidget(self.elapsed)
        lay.addLayout(mission)
        lay.addStretch()

        self.hb = QLabel("HB --")
        self.hb.setObjectName("clock")
        lay.addWidget(self.hb)
        lay.addSpacing(16)
        self.clock = QLabel()
        self.clock.setObjectName("clock")
        lay.addWidget(self.clock)
        lay.addSpacing(16)

        self.pill = QLabel()
        self.pill.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pill.setFixedHeight(30)
        self.pill.setMinimumWidth(150)
        lay.addWidget(self.pill)
        self.set_link(False)

        timer = QTimer(self)
        timer.timeout.connect(self._tick)
        timer.start(1000)
        self._tick()

    def _tick(self):
        self.clock.setText(datetime.now().strftime("%H:%M:%S"))
        e = int(time.monotonic() - self._t0)
        self.elapsed.setText(f"{e // 3600:02d}:{e % 3600 // 60:02d}:{e % 60:02d}")

    def set_heartbeat(self, age):
        self.hb.setText("HB --" if age is None else f"HB {age:.1f}s")

    def set_link(self, ok: bool):
        color = GOOD if ok else BAD
        self.pill.setText("●  LINK OK" if ok else "●  NO LINK")
        self.pill.setStyleSheet(
            f"color: {color}; background: transparent; border: 1px solid {color};"
            "border-radius: 15px; font-weight: bold; font-size: 11px; letter-spacing: 1px;")


# ==========================================================================
# TOP STRIP
# ==========================================================================
class TelemetryStrip(QWidget):
    def __init__(self):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)
        self.tiles = {
            "depth":   StatTile("Depth", "m", bar=True),
            "heading": StatTile("Heading", "°"),
            "range_l": StatTile("Range Left", "m"),
            "range_r": StatTile("Range Right", "m"),
            "battery": StatTile("Battery", "V", bar=True),
            "safety":  StatTile("Safety"),
        }
        for t in self.tiles.values():
            lay.addWidget(t, stretch=1)


# ==========================================================================
# CONTROL TAB
# ==========================================================================
class ControlPanel(QWidget):
    command_sent = pyqtSignal(str)
    power_changed = pyqtSignal(int)
    servo_changed = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.setObjectName("tabBody")
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(8)

        # --- surge / yaw pad
        root.addWidget(section_title("Drive  (surge / yaw)"))
        grid = QGridLayout()
        grid.setSpacing(8)
        for label, cmd, row, col, kind in [
            ("▲  FORWARD", "FORWARD", 0, 1, "dir"),
            ("◄  LEFT", "LEFT", 1, 0, "dir"),
            ("■  STOP", "STOP", 1, 1, "stop"),
            ("RIGHT  ►", "RIGHT", 1, 2, "dir"),
            ("▼  BACKWARD", "BACKWARD", 2, 1, "dir"),
        ]:
            grid.addWidget(self._make_button(label, cmd, kind), row, col)
        root.addLayout(grid)

        # --- heave / pitch
        root.addWidget(section_title("Depth & pitch  (heave / pitch)"))
        g2 = QGridLayout()
        g2.setSpacing(8)
        for label, cmd, row, col in [
            ("ASCEND  [W]", "ASCEND", 0, 0),
            ("DESCEND  [S]", "DESCEND", 0, 1),
            ("PITCH UP  [Q]", "PITCH_UP", 1, 0),
            ("PITCH DOWN  [E]", "PITCH_DOWN", 1, 1),
        ]:
            g2.addWidget(self._make_button(label, cmd, "aux"), row, col)
        root.addLayout(g2)

        # --- power
        row = QHBoxLayout()
        cap = QLabel("THRUSTER POWER")
        cap.setObjectName("caption")
        self.power_value = QLabel("0 %")
        self.power_value.setStyleSheet(
            f"color: {ACCENT}; font-weight: bold; font-size: 15px; background: transparent;")
        row.addWidget(cap)
        row.addStretch()
        row.addWidget(self.power_value)
        root.addLayout(row)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.valueChanged.connect(self._on_power)
        root.addWidget(self.slider)

        # --- thruster bars
        root.addWidget(section_title("Thruster output"))
        tg = QGridLayout()
        tg.setHorizontalSpacing(14)
        tg.setVerticalSpacing(2)
        self.thr = {
            "port": ThrusterBar("PORT"), "stbd": ThrusterBar("STBD"),
            "fore": ThrusterBar("FORE"), "aft": ThrusterBar("AFT"),
        }
        tg.addWidget(self.thr["port"], 0, 0)
        tg.addWidget(self.thr["stbd"], 0, 1)
        tg.addWidget(self.thr["fore"], 1, 0)
        tg.addWidget(self.thr["aft"], 1, 1)
        root.addLayout(tg)

        # --- payload
        root.addWidget(section_title("Payload"))
        srow = QHBoxLayout()
        scap = QLabel("MG995 SERVO")
        scap.setObjectName("caption")
        self.servo_value = QLabel("90°")
        self.servo_value.setStyleSheet(
            f"color: {ACCENT}; font-weight: bold; font-size: 14px; background: transparent;")
        srow.addWidget(scap)
        srow.addStretch()
        srow.addWidget(self.servo_value)
        root.addLayout(srow)
        self.servo = QSlider(Qt.Orientation.Horizontal)
        self.servo.setRange(0, 180)
        self.servo.setValue(90)
        self.servo.valueChanged.connect(self._on_servo)
        root.addWidget(self.servo)

        arow = QHBoxLayout()   # linear actuator - placeholder commands
        for label, cmd in [("ACTUATOR  EXTEND", "ACT_EXTEND"), ("ACTUATOR  RETRACT", "ACT_RETRACT")]:
            arow.addWidget(self._make_button(label, cmd, "aux"))
        root.addLayout(arow)
        root.addStretch()

    def _make_button(self, label: str, cmd: str, kind: str) -> QPushButton:
        b = QPushButton(label)
        b.setObjectName(kind)
        b.setCursor(Qt.CursorShape.PointingHandCursor)
        b.pressed.connect(lambda c=cmd: self.command_sent.emit(c))
        if HOLD_TO_DRIVE and cmd not in ("STOP",):
            stop_cmd = "ACT_STOP" if cmd.startswith("ACT_") else "STOP"
            b.released.connect(lambda s=stop_cmd: self.command_sent.emit(s))
        return b

    def _on_power(self, v: int):
        self.power_value.setText(f"{v} %")
        self.power_changed.emit(v)

    def _on_servo(self, v: int):
        self.servo_value.setText(f"{v}°")
        self.servo_changed.emit(v)

    def set_thrusters(self, d: dict):
        for k, bar in self.thr.items():
            if k in d:
                bar.set_value(d[k])


# ==========================================================================
# WATER TAB
# ==========================================================================
class WaterTab(QWidget):
    def __init__(self):
        super().__init__()
        self.setObjectName("tabBody")
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(8)
        root.addWidget(section_title("Water quality  (calibrated value, raw ADC below)"))
        grid = QGridLayout()
        grid.setSpacing(10)
        self.tiles = {
            "ph":   StatTile("pH", "pH", bar=True, sub=True),
            "orp":  StatTile("ORP", "mV", sub=True),
            "tds":  StatTile("TDS", "ppm", sub=True),
            "turb": StatTile("Turbidity", "NTU", sub=True),
            "temp": StatTile("Water Temp (DS18B20)", "°C"),
        }
        grid.addWidget(self.tiles["ph"], 0, 0)
        grid.addWidget(self.tiles["orp"], 0, 1)
        grid.addWidget(self.tiles["tds"], 1, 0)
        grid.addWidget(self.tiles["turb"], 1, 1)
        grid.addWidget(self.tiles["temp"], 2, 0, 1, 2)
        root.addLayout(grid)
        note = QLabel("Readings are only meaningful after calibration "
                      "(pH buffers, ORP/TDS standards, turbidity curve).")
        note.setObjectName("caption")
        note.setWordWrap(True)
        root.addWidget(note)
        root.addStretch()

    def update_data(self, w: dict):
        raw = w.get("raw", {})
        T = self.tiles

        def rawtxt(k):
            return f"raw {raw[k]}" if k in raw else None

        if "ph" in w:
            T["ph"].set_value(f"{w['ph']:.2f}", bar_pct=w["ph"] / 14 * 100, sub_text=rawtxt("ph"))
        if "orp_mv" in w:
            T["orp"].set_value(f"{w['orp_mv']:.0f}", sub_text=rawtxt("orp"))
        if "tds_ppm" in w:
            T["tds"].set_value(f"{w['tds_ppm']:.0f}", sub_text=rawtxt("tds"))
        if "turbidity_ntu" in w:
            T["turb"].set_value(f"{w['turbidity_ntu']:.1f}", sub_text=rawtxt("turb"))
        if "temp_c" in w:
            T["temp"].set_value(f"{w['temp_c']:.1f}")


# ==========================================================================
# SENSORS TAB
# ==========================================================================
class SensorsTab(QWidget):
    GROUPS = [("imu", "IMU"), ("depth", "DEPTH"), ("range", "SONAR"),
              ("water", "WATER"), ("thrusters", "THRUST"), ("battery", "BATT")]

    def __init__(self):
        super().__init__()
        self.setObjectName("tabBody")
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(8)

        root.addWidget(section_title("Sensor health"))
        hrow = QHBoxLayout()
        hrow.setSpacing(6)
        self.chips = {}
        for key, name in self.GROUPS:
            chip = HealthChip(name)
            self.chips[key] = chip
            hrow.addWidget(chip)
        root.addLayout(hrow)

        root.addWidget(section_title("Orientation  (JY901S IMU)"))
        dials = QHBoxLayout()
        self.attitude = AttitudeWidget()
        self.compass = CompassWidget()
        dials.addWidget(self.attitude)
        dials.addWidget(self.compass)
        root.addLayout(dials)

        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(3)
        self.num = {}
        rows = [("ATT °", ["roll", "pitch", "yaw"]),
                ("ACC g", ["ax", "ay", "az"]),
                ("GYRO °/s", ["gx", "gy", "gz"])]
        for r, (title, keys) in enumerate(rows):
            cap = QLabel(title)
            cap.setObjectName("caption")
            grid.addWidget(cap, r, 0)
            for c, k in enumerate(keys):
                lab = QLabel(f"{k[-1].upper() if r else k[:1].upper() + k[1:]}  --")
                lab.setObjectName("mono")
                self.num[k] = lab
                grid.addWidget(lab, r, c + 1)
        root.addLayout(grid)

        root.addWidget(section_title("Depth  (MS5837)"))
        self.pressure = QLabel("Pressure  -- mbar")
        self.pressure.setObjectName("mono")
        root.addWidget(self.pressure)
        self.depth_graph = Sparkline("Depth history", "m", invert=True)
        root.addWidget(self.depth_graph)

        root.addWidget(section_title("Vision"))
        self.vision = QLabel("Scale --   ·   Detections 0   ·   AI FPS --")
        self.vision.setObjectName("mono")
        self.vision.setWordWrap(True)
        root.addWidget(self.vision)
        root.addStretch()

    def update_imu(self, d: dict):
        if "roll" in d and "pitch" in d:
            self.attitude.set_attitude(d["roll"], d["pitch"])
        if "yaw" in d:
            self.compass.set_heading(d["yaw"])
        for k in ("roll", "pitch", "yaw"):
            if k in d:
                self.num[k].setText(f"{k[:1].upper() + k[1:]}  {d[k]:+7.1f}")
        for k in ("ax", "ay", "az", "gx", "gy", "gz"):
            if k in d:
                self.num[k].setText(f"{k[-1].upper()}  {d[k]:+7.2f}")

    def update_depth(self, d: dict):
        if "pressure_mbar" in d:
            self.pressure.setText(f"Pressure  {d['pressure_mbar']:.0f} mbar")
        if "depth_m" in d:
            self.depth_graph.push(d["depth_m"])

    def set_health(self, states: dict):
        for key, chip in self.chips.items():
            chip.set_state(states.get(key, "none"))

    def set_vision(self, scale_m_per_px, detections: int, fps):
        s = f"{scale_m_per_px * 1000:.2f} mm/px" if scale_m_per_px else "--"
        f = f"{fps:.1f}" if fps is not None else "--"
        self.vision.setText(f"Scale {s}   ·   Detections {detections}   ·   AI FPS {f}")


# ==========================================================================
# EVENT LOG
# ==========================================================================
class EventLog(QFrame):
    COLORS = {"info": MUTED, "cmd": ACCENT, "warn": WARN, "alert": BAD, "det": PURPLE}

    def __init__(self):
        super().__init__()
        card, body = make_card("Event Log")
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.addWidget(card)
        self.list = QListWidget()
        body.addWidget(self.list)

    def add(self, text: str, level: str = "info"):
        item = QListWidgetItem(f"{datetime.now():%H:%M:%S}  {text}")
        item.setForeground(QColor(self.COLORS.get(level, MUTED)))
        self.list.addItem(item)
        self.list.scrollToBottom()
        if self.list.count() > 300:
            self.list.takeItem(0)


# ==========================================================================
# MAIN WINDOW
# ==========================================================================
class MainWindow(QMainWindow):
    KEY_COMMANDS = {
        Qt.Key.Key_Up: "FORWARD", Qt.Key.Key_Down: "BACKWARD",
        Qt.Key.Key_Left: "LEFT", Qt.Key.Key_Right: "RIGHT",
        Qt.Key.Key_W: "ASCEND", Qt.Key.Key_S: "DESCEND",
        Qt.Key.Key_Q: "PITCH_UP", Qt.Key.Key_E: "PITCH_DOWN",
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("NEPTUNE - Dam Inspection ROV")
        self.resize(1480, 880)

        self.mission_id = datetime.now().strftime("%Y-%m-%d_%H-%M")
        self.db = LogDB(DB_PATH, self.mission_id)
        self.state = {}             # latest merged telemetry
        self.last_seen = {}         # group -> monotonic time
        self.last_packet = None
        self.comm_ok = False
        self._held_keys = []
        self._active_alerts = {}
        self.det_count = 0
        self._last_det_event = 0.0
        self.ai_fps = None
        self.current_cmd = None
        self.power = 0
        self.worker = None

        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(14, 14, 14, 8)
        outer.setSpacing(12)

        self.header = Header(self.mission_id)
        outer.addWidget(self.header)
        self.strip = TelemetryStrip()
        outer.addWidget(self.strip)

        body = QHBoxLayout()
        body.setSpacing(12)
        outer.addLayout(body, stretch=1)

        left = QVBoxLayout()
        left.setSpacing(12)
        self.video = HudVideo()
        self.log = EventLog()
        self.log.setMaximumHeight(160)
        left.addWidget(self.video, stretch=4)
        left.addWidget(self.log, stretch=1)
        body.addLayout(left, stretch=1)

        self.tabs = QTabWidget()
        self.controls = ControlPanel()
        self.water = WaterTab()
        self.sensors = SensorsTab()
        self.tabs.addTab(scrollable(self.controls), "CONTROL")
        self.tabs.addTab(scrollable(self.water), "WATER")
        self.tabs.addTab(scrollable(self.sensors), "SENSORS")
        self.tabs.setFixedWidth(470)
        body.addWidget(self.tabs)

        # keyboard events must always reach this window
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        for w in self.findChildren(QWidget):
            w.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setFocus()

        self.controls.command_sent.connect(self.on_command)
        self.controls.power_changed.connect(self.on_power)
        self.controls.servo_changed.connect(self.on_servo)

        # link / safety / health supervisor
        self.supervisor = QTimer(self)
        self.supervisor.timeout.connect(self.supervise)
        self.supervisor.start(500)

        # database logging, once per second
        self.db_timer = QTimer(self)
        self.db_timer.timeout.connect(self._log_db)
        self.db_timer.start(1000)

        self._t0 = time.time()
        self._mock_depth = 2.5
        self._mock_wall = 1.6
        self._thr_smooth = dict(port=0.0, stbd=0.0, fore=0.0, aft=0.0)
        if MOCK_MODE:
            self.video.mock = True
            self.video.set_lasers((0.46, 0.55), (0.54, 0.55))
            self.mock_timer = QTimer(self)
            self.mock_timer.timeout.connect(self.mock_tick)
            self.mock_timer.start(200)

        if USE_YOLO:
            self.worker = YoloWorker(VIDEO_SOURCE, MODEL_PATH)
            self.worker.frame_ready.connect(self.video.set_frame)
            self.worker.detections_ready.connect(self.on_detections)
            self.worker.fps_ready.connect(self._on_fps)
            self.worker.error.connect(lambda m: self.log_event(m, "alert"))
            self.worker.start()

        self.statusBar().showMessage(
            "Arrows: drive  ·  W/S: up/down  ·  Q/E: pitch  ·  Space: STOP  ·  +/-: power"
            + ("   ·   MOCK MODE" if MOCK_MODE else ""))
        self.log_event("GUI started" + (" (mock data)" if MOCK_MODE else ""), "info")
        self._refresh_vision()

    # ---------------------------------------------------------------- helpers
    def log_event(self, text: str, level: str = "info"):
        self.log.add(text, level)

    def _on_fps(self, fps: float):
        self.ai_fps = fps
        self._refresh_vision()

    def _refresh_vision(self):
        self.sensors.set_vision(self.video.scale_m_per_px, self.det_count, self.ai_fps)

    # ---------------------------------------------------------------- commands
    def on_command(self, cmd: str):
        self.current_cmd = None if cmd in ("STOP", "ACT_STOP") else cmd
        print(f"[CMD] {cmd}")
        self.log_event(f"Command: {cmd}", "cmd")
        # TODO (socket step): send `cmd` and self.power to the ROV here

    def on_power(self, value: int):
        self.power = value
        print(f"[POWER] {value}%")

    def on_servo(self, value: int):
        print(f"[SERVO] {value}")

    # ---------------------------------------------------------------- telemetry
    def on_telemetry(self, packet: dict):
        """Feed every telemetry packet from the ROV into this method."""
        now = time.monotonic()
        self.last_packet = now
        for group, data in packet.items():
            if not isinstance(data, dict):
                continue
            self.last_seen[group] = now
            self.state.setdefault(group, {}).update(data)

        S = self.strip.tiles
        L = LIMITS

        if "imu" in packet:
            self.sensors.update_imu(packet["imu"])
            if "yaw" in packet["imu"]:
                y = packet["imu"]["yaw"] % 360
                S["heading"].set_value(f"{y:.0f}")
                S["heading"].set_unit(f"° {cardinal(y)}")

        if "depth" in packet:
            d = packet["depth"]
            self.sensors.update_depth(d)
            if "depth_m" in d:
                v = d["depth_m"]
                lvl = "alert" if v >= L["DEPTH_LIMIT_M"] else "warn" if v >= L["DEPTH_WARN_M"] else "ok"
                S["depth"].set_value(f"{v:.2f}", lvl, v / L["MAX_DEPTH_M"] * 100)
                self.video.depth_text = f"{v:.1f} m"
                self.video.update()

        if "range" in packet:
            r = packet["range"]
            for key, tile in (("left_m", "range_l"), ("right_m", "range_r")):
                if key in r:
                    v = r[key]
                    lvl = "alert" if v <= L["RANGE_CRIT_M"] else "warn" if v <= L["RANGE_WARN_M"] else "ok"
                    S[tile].set_value(f"{v:.2f}", lvl)

        if "water" in packet:
            self.water.update_data(packet["water"])

        if "thrusters" in packet:
            self.controls.set_thrusters(packet["thrusters"])

        if "battery" in packet and "v" in packet["battery"]:
            v = packet["battery"]["v"]
            lo, full = L["BATT_CRIT_V"], L["BATT_FULL_V"]
            lvl = "alert" if v <= lo else "warn" if v <= L["BATT_WARN_V"] else "ok"
            S["battery"].set_value(f"{v:.2f}", lvl, (v - lo) / (full - lo) * 100)

        self.supervise()

    # ---------------------------------------------------------------- supervisor
    def _group_states(self) -> dict:
        now = time.monotonic()
        out = {}
        for key, _ in SensorsTab.GROUPS:
            if key not in self.last_seen:
                out[key] = "none"
            else:
                out[key] = "ok" if now - self.last_seen[key] < STALE_S else "stale"
        return out

    def _compute_alerts(self) -> dict:
        L = LIMITS
        a = {}
        if not self.comm_ok:
            a["comm"] = ("alert", "Telemetry link lost - failsafe expected on ROV")
            return a
        depth = self.state.get("depth", {}).get("depth_m")
        if depth is not None:
            if depth >= L["DEPTH_LIMIT_M"]:
                a["depth"] = ("alert", f"DEPTH LIMIT exceeded: {depth:.1f} m")
            elif depth >= L["DEPTH_WARN_M"]:
                a["depth"] = ("warn", f"Depth high: {depth:.1f} m")
        rng = self.state.get("range", {})
        vals = [v for v in (rng.get("left_m"), rng.get("right_m")) if v is not None]
        if vals:
            m = min(vals)
            if m <= L["RANGE_CRIT_M"]:
                a["range"] = ("alert", f"Too close to wall: {m:.2f} m")
            elif m <= L["RANGE_WARN_M"]:
                a["range"] = ("warn", f"Close to wall: {m:.2f} m")
        bv = self.state.get("battery", {}).get("v")
        if bv is not None:
            if bv <= L["BATT_CRIT_V"]:
                a["battery"] = ("alert", f"Battery CRITICAL: {bv:.2f} V")
            elif bv <= L["BATT_WARN_V"]:
                a["battery"] = ("warn", f"Battery low: {bv:.2f} V")
        for key, st in self._group_states().items():
            if st == "stale":
                a[f"stale_{key}"] = ("warn", f"{key} data stale")
        return a

    def supervise(self):
        now = time.monotonic()
        age = None if self.last_packet is None else now - self.last_packet
        ok = age is not None and age < HEARTBEAT_TIMEOUT_S
        if ok != self.comm_ok:
            self.comm_ok = ok
            self.header.set_link(ok)
            if ok:
                self.log_event("Telemetry link established", "info")
        self.header.set_heartbeat(age)
        self.sensors.set_health(self._group_states())

        alerts = self._compute_alerts()
        for key, (level, msg) in alerts.items():
            if key not in self._active_alerts or self._active_alerts[key][0] != level:
                self.log_event(msg, "alert" if level == "alert" else "warn")
        for key in set(self._active_alerts) - set(alerts):
            self.log_event(f"{key}: back to normal", "info")
        self._active_alerts = alerts

        # safety tile
        tile = self.strip.tiles["safety"]
        if not ok:
            tile.set_value("NO LINK", "alert")
        elif any(v[0] == "alert" for v in alerts.values()):
            tile.set_value("ALARM", "alert")
        elif alerts:
            tile.set_value("CHECK", "warn")
        else:
            tile.set_value("OK", "ok", color=GOOD)

        # battery tile when no battery monitor is present
        if "battery" not in self.last_seen:
            self.strip.tiles["battery"].set_value("N/A", "ok", 0, color=MUTED)

    def _log_db(self):
        if self.state:
            try:
                self.db.log_telemetry(self.state)
            except sqlite3.Error as e:
                self.log_event(f"DB error: {e}", "warn")

    # ---------------------------------------------------------------- detections
    def on_detections(self, dets: list):
        self.video.set_detections(dets)
        now = time.monotonic()
        if not dets or now - self._last_det_event < DET_COOLDOWN_S:
            return
        self._last_det_event = now
        best = max(dets, key=lambda d: d["conf"])
        self.det_count += 1
        self.video.detections = self.det_count
        length = self.video.box_length_m(best["box"])
        depth = self.state.get("depth", {}).get("depth_m")

        os.makedirs(SNAP_DIR, exist_ok=True)
        snap = os.path.join(SNAP_DIR, f"{self.mission_id}_{self.det_count:04d}.png")
        self.video.grab().save(snap)

        msg = f"{best['cls']} detected {best['conf']:.0%}"
        if length is not None:
            msg += f"  (L~{length:.2f} m)"
        if depth is not None:
            msg += f"  @ {depth:.1f} m"
        self.log_event(msg, "det")
        try:
            self.db.log_detection(best["cls"], best["conf"], best["box"], depth, length, snap)
        except sqlite3.Error as e:
            self.log_event(f"DB error: {e}", "warn")
        self._refresh_vision()

    # ---------------------------------------------------------------- mock data
    def mock_tick(self):
        t = time.time() - self._t0
        cmd = self.current_cmd

        if cmd == "DESCEND":
            self._mock_depth = min(9.5, self._mock_depth + 0.05)
        elif cmd == "ASCEND":
            self._mock_depth = max(0.0, self._mock_depth - 0.05)
        if cmd == "FORWARD":
            self._mock_wall = max(0.15, self._mock_wall - 0.03)
        elif cmd == "BACKWARD":
            self._mock_wall = min(3.0, self._mock_wall + 0.03)
        depth = self._mock_depth + 0.05 * math.sin(t)

        # fake thruster demand from the current command (display only)
        p = float(self.power)
        target = dict(port=0.0, stbd=0.0, fore=0.0, aft=0.0)
        if cmd == "FORWARD":
            target.update(port=p, stbd=p)
        elif cmd == "BACKWARD":
            target.update(port=-p, stbd=-p)
        elif cmd == "LEFT":
            target.update(port=-p, stbd=p)
        elif cmd == "RIGHT":
            target.update(port=p, stbd=-p)
        elif cmd == "DESCEND":
            target.update(fore=-p, aft=-p)
        elif cmd == "ASCEND":
            target.update(fore=p, aft=p)
        elif cmd == "PITCH_UP":
            target.update(fore=p, aft=-p)
        elif cmd == "PITCH_DOWN":
            target.update(fore=-p, aft=p)
        for k in self._thr_smooth:
            self._thr_smooth[k] += (target[k] - self._thr_smooth[k]) * 0.4

        self.on_telemetry({
            "imu": {
                "roll": 8 * math.sin(0.7 * t), "pitch": 5 * math.sin(0.5 * t + 1),
                "yaw": (120 + 6 * t) % 360,
                "ax": 0.03 * math.sin(t), "ay": 0.02 * math.cos(1.3 * t),
                "az": 1.0 + 0.02 * math.sin(2 * t),
                "gx": 3 * math.cos(0.7 * t), "gy": 2 * math.cos(0.5 * t), "gz": 6.0,
            },
            "depth": {"depth_m": depth, "pressure_mbar": 1013 + depth * 98.07},
            "range": {"left_m": self._mock_wall + 0.1 * math.sin(t),
                      "right_m": self._mock_wall + 0.1 * math.cos(t)},
            "water": {
                "ph": 7.4 + 0.1 * math.sin(0.3 * t), "orp_mv": 220 + 10 * math.sin(0.2 * t),
                "tds_ppm": 180 + 6 * math.sin(0.25 * t),
                "turbidity_ntu": 4.0 + 0.8 * math.sin(0.4 * t),
                "temp_c": 28.0 - depth * 0.3,
                "raw": {"ph": 12000 + random.randint(-40, 40), "orp": 15000 + random.randint(-40, 40),
                        "tds": 9000 + random.randint(-40, 40), "turb": 7000 + random.randint(-40, 40)},
            },
            "thrusters": dict(self._thr_smooth),
            "battery": {"v": 12.45 - 0.0004 * t},
        })

        # fake crack that sweeps across the image every 12 s
        phase = t % 12
        if 3 < phase < 8:
            k = (phase - 3) / 5
            cx = 0.3 + 0.4 * k
            conf = 0.82 + 0.08 * math.sin(3 * t)
            self.on_detections([{"cls": "crack", "conf": conf,
                                 "box": (cx - 0.10, 0.50, cx + 0.10, 0.58)}])
        else:
            self.on_detections([])

    # ---------------------------------------------------------------- keyboard
    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in self.KEY_COMMANDS:
            if key not in self._held_keys:
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
        if HOLD_TO_DRIVE and key in self.KEY_COMMANDS:
            if key in self._held_keys:
                self._held_keys.remove(key)
            if self._held_keys:
                self.controls.command_sent.emit(self.KEY_COMMANDS[self._held_keys[-1]])
            else:
                self.controls.command_sent.emit("STOP")
        else:
            super().keyReleaseEvent(event)

    def changeEvent(self, event):
        # safety: if the window loses focus while a key is held, stop
        if event.type() == QEvent.Type.ActivationChange and not self.isActiveWindow():
            if self._held_keys:
                self._held_keys.clear()
                self.controls.command_sent.emit("STOP")
        super().changeEvent(event)

    def closeEvent(self, event):
        self.controls.command_sent.emit("STOP")
        if self.worker:
            self.worker.stop()
        self.db.close()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())