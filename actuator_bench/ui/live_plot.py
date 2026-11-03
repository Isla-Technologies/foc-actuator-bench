"""Live charts: torque, motor current and output speed, stacked on one shared time axis."""

from __future__ import annotations

import os

import numpy as np

os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")
import pyqtgraph as pg  # noqa: E402
from PySide6.QtCore import QPointF, Qt  # noqa: E402
from PySide6.QtGui import QColor, QPainter, QPen  # noqa: E402
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QPushButton, QVBoxLayout, QWidget  # noqa: E402

from . import theme  # noqa: E402
from .widgets import label  # noqa: E402


class RingBuffer:
    """Fixed-size history per field. Storage is doubled so the newest N values are always contiguous."""

    def __init__(self, fields: tuple[str, ...], capacity: int):
        self.fields = fields
        self.capacity = capacity
        self._data = {f: np.zeros(2 * capacity) for f in fields}
        self._end = 0
        self._count = 0

    def clear(self) -> None:
        self._end = 0
        self._count = 0

    def __len__(self) -> int:
        return self._count

    def extend(self, columns: dict[str, np.ndarray]) -> None:
        n = len(columns[self.fields[0]])
        if n == 0:
            return
        n_keep = min(n, self.capacity)
        idx = (self._end + np.arange(n_keep)) % self.capacity
        for field in self.fields:
            values = np.asarray(columns[field][-n_keep:], dtype=float)
            buf = self._data[field]
            buf[idx] = values
            buf[idx + self.capacity] = values
        self._end = (self._end + n_keep) % self.capacity
        self._count = min(self._count + n_keep, self.capacity)

    def last(self, field: str, count: int | None = None) -> np.ndarray:
        count = self._count if count is None else min(count, self._count)
        stop = self._end + self.capacity
        return self._data[field][stop - count:stop]


class LineKey(QWidget):
    """Legend key: a short 2 px line in the series color (the label beside it stays in ink)."""

    def __init__(self, color: str, parent=None):
        super().__init__(parent)
        self.color = QColor(color)
        self.setFixedSize(22, 14)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        pen = QPen(self.color, 2.5)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        y = self.height() / 2
        painter.drawLine(QPointF(2, y), QPointF(self.width() - 2, y))


def _pen(color: str, width: float) -> pg.mkPen:
    pen = pg.mkPen(color=color, width=width)
    pen.setCapStyle(Qt.RoundCap)
    pen.setJoinStyle(Qt.RoundJoin)
    pen.setCosmetic(True)
    return pen


class LivePlots(QWidget):
    WINDOWS = (5, 10, 30, 60)

    def __init__(self, parent=None):
        super().__init__(parent)
        pg.setConfigOptions(antialias=True, background=theme.SURFACE, foreground=theme.INK_SECONDARY)
        self.window_s = 10.0
        self.factor = 1.0
        self.unit = "N·m"
        self.min_span = {"torque": 0.1, "current": 0.5, "speed": 5.0}
        self._snapshot: dict[str, np.ndarray] | None = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)
        for color, text in ((theme.ACCENT, "Measured"), (theme.CONTEXT_SERIES, "Commanded / setpoint")):
            toolbar.addWidget(LineKey(color))
            toolbar.addWidget(label(text, "secondary"))
            toolbar.addSpacing(10)
        toolbar.addStretch(1)
        toolbar.addWidget(label("Window", "muted"))
        self.window_combo = QComboBox()
        for seconds in self.WINDOWS:
            self.window_combo.addItem(f"{seconds} s", seconds)
        self.window_combo.setCurrentIndex(1)
        self.window_combo.currentIndexChanged.connect(lambda: setattr(self, "window_s", float(self.window_combo.currentData())))
        toolbar.addWidget(self.window_combo)
        self.pause_button = QPushButton("Pause")
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(lambda on: self.pause_button.setText("Resume" if on else "Pause"))
        toolbar.addWidget(self.pause_button)
        layout.addLayout(toolbar)
        self.readout = label("Hover over the charts to read exact values.", "muted")
        self.readout.setTextFormat(Qt.RichText)
        layout.addWidget(self.readout)

        self.view = pg.GraphicsLayoutWidget()
        self.view.setBackground(theme.SURFACE)
        self.view.setMinimumHeight(260)
        self.view.ci.setSpacing(4)
        layout.addWidget(self.view, 1)

        self.p_torque = self._make_plot(0, "Output torque (N·m)")
        self.p_current = self._make_plot(1, "Motor current (A)")
        self.p_speed = self._make_plot(2, "Output speed (rpm)")
        for plot in (self.p_current, self.p_speed):
            plot.setXLink(self.p_torque)
        for plot in (self.p_torque, self.p_current):
            plot.getAxis("bottom").setStyle(showValues=False)
        self.p_speed.setLabel("bottom", "Time", units="s", color=theme.INK_MUTED)

        accent, context = _pen(theme.ACCENT, 2), _pen(theme.CONTEXT_SERIES, 2)
        self.c_torque = self.p_torque.plot(pen=accent)
        self.c_torque_cmd = self.p_torque.plot(pen=context)
        self.c_iq = self.p_current.plot(pen=accent)
        self.c_iq_set = self.p_current.plot(pen=context)
        self.c_speed = self.p_speed.plot(pen=accent)
        for curve in (self.c_torque, self.c_iq, self.c_speed):
            curve.setZValue(2)
        for curve in (self.c_torque_cmd, self.c_iq_set):
            curve.setZValue(1)
        for curve in (self.c_torque, self.c_torque_cmd, self.c_iq, self.c_iq_set, self.c_speed):
            curve.setClipToView(True)

        self.crosshairs = []
        for plot in (self.p_torque, self.p_current, self.p_speed):
            line = pg.InfiniteLine(angle=90, movable=False, pen=_pen(theme.INK_MUTED, 1))
            line.hide()
            plot.addItem(line, ignoreBounds=True)
            self.crosshairs.append(line)
        self.view.scene().sigMouseMoved.connect(self._mouse_moved)

    def _make_plot(self, row: int, title: str) -> pg.PlotItem:
        plot = self.view.addPlot(row=row, col=0)
        plot.setTitle(title, color=theme.INK_SECONDARY, size="10pt", justify="left")
        plot.setMenuEnabled(False)
        plot.hideButtons()
        plot.setMouseEnabled(x=False, y=False)
        plot.showGrid(x=True, y=True, alpha=0.28)
        for name in ("left", "bottom"):
            axis = plot.getAxis(name)
            axis.setPen(_pen(theme.AXIS, 1))
            axis.setTextPen(pg.mkPen(theme.INK_MUTED))
        plot.getAxis("left").setWidth(56)
        return plot

    def set_scale(self, max_torque: float, max_current: float, max_speed: float) -> None:
        """Smallest y-span per chart (5 % of the limits) so idle noise isn't blown up to full height."""
        self.min_span = {"torque": 0.05 * max_torque, "current": 0.05 * max_current, "speed": 0.05 * max_speed}

    def set_unit(self, unit: str, factor: float) -> None:
        self.unit, self.factor = unit, factor
        self.p_torque.setTitle(f"Output torque ({unit})", color=theme.INK_SECONDARY, size="10pt", justify="left")

    @property
    def paused(self) -> bool:
        return self.pause_button.isChecked()

    def clear(self) -> None:
        for curve in (self.c_torque, self.c_torque_cmd, self.c_iq, self.c_iq_set, self.c_speed):
            curve.setData([], [])
        self._snapshot = None

    def update_from(self, buffer: RingBuffer) -> None:
        if self.paused or not len(buffer):
            return
        t = buffer.last("t")
        start = int(np.searchsorted(t, t[-1] - self.window_s))
        x = t[start:]
        data = {
            "t": x,
            "torque": buffer.last("torque_out")[start:] * self.factor,
            "torque_cmd": buffer.last("torque_cmd_out")[start:] * self.factor,
            "iq": buffer.last("iq")[start:],
            "iq_set": buffer.last("iq_set")[start:],
            "speed": buffer.last("speed_out_rpm")[start:],
        }
        self._snapshot = data
        self.c_torque.setData(x, data["torque"], skipFiniteCheck=True)
        self.c_torque_cmd.setData(x, data["torque_cmd"], skipFiniteCheck=True)
        self.c_iq.setData(x, data["iq"], skipFiniteCheck=True)
        self.c_iq_set.setData(x, data["iq_set"], skipFiniteCheck=True)
        self.c_speed.setData(x, data["speed"], skipFiniteCheck=True)
        for plot, key, series in ((self.p_torque, "torque", (data["torque"], data["torque_cmd"])),
                                  (self.p_current, "current", (data["iq"], data["iq_set"])),
                                  (self.p_speed, "speed", (data["speed"],))):
            self._set_y_range(plot, self.min_span[key], series)
        self.p_torque.setXRange(max(0.0, t[-1] - self.window_s), max(self.window_s, t[-1]), padding=0)

    @staticmethod
    def _set_y_range(plot: pg.PlotItem, min_span: float, series: tuple[np.ndarray, ...]) -> None:
        values = np.concatenate([s[np.isfinite(s)] for s in series])
        if not values.size:
            return
        low, high = float(values.min()), float(values.max())
        if high - low < min_span:
            middle = (high + low) / 2.0
            low, high = middle - min_span / 2.0, middle + min_span / 2.0
        pad = 0.08 * (high - low)
        plot.setYRange(low - pad, high + pad, padding=0)

    def _mouse_moved(self, pos) -> None:
        data = self._snapshot
        plot = next((p for p in (self.p_torque, self.p_current, self.p_speed)
                     if p.sceneBoundingRect().contains(pos)), None)
        if plot is None or data is None or not len(data["t"]):
            for line in self.crosshairs:
                line.hide()
            return
        x = plot.vb.mapSceneToView(pos).x()
        i = int(np.clip(np.searchsorted(data["t"], x), 0, len(data["t"]) - 1))
        if i > 0 and abs(data["t"][i - 1] - x) < abs(data["t"][i] - x):
            i -= 1
        t = data["t"][i]
        for line in self.crosshairs:
            line.setPos(t)
            line.show()
        u = self.unit
        self.readout.setText(
            f"<b>{t:.2f}</b> s &nbsp;·&nbsp; torque <b>{data['torque'][i]:.3f}</b> {u} "
            f"(commanded {data['torque_cmd'][i]:.3f}) &nbsp;·&nbsp; current <b>{data['iq'][i]:.2f}</b> A "
            f"(setpoint {data['iq_set'][i]:.2f}) &nbsp;·&nbsp; speed <b>{data['speed'][i]:.1f}</b> rpm")

    def save_image(self, path: str) -> bool:
        return self.view.grab().save(path)
