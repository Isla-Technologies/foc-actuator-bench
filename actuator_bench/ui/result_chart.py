"""A test's result chart: what was measured (accent) against the fitted model (gray, context)."""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget

from . import theme
from .live_plot import LineKey, _pen, pg
from .widgets import label


class ResultChart(QWidget):
    def __init__(self, measured: str = "Measured", fitted: str = "Fit", parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        keys = QHBoxLayout()
        keys.setSpacing(6)
        self.key_labels = []
        for color, text in ((theme.ACCENT, measured), (theme.CONTEXT_SERIES, fitted)):
            keys.addWidget(LineKey(color))
            key_label = label(text, "secondary")
            self.key_labels.append(key_label)
            keys.addWidget(key_label)
            keys.addSpacing(10)
        keys.addStretch(1)
        self.readout = label("", "muted")
        self.readout.setTextFormat(Qt.RichText)
        keys.addWidget(self.readout)
        layout.addLayout(keys)

        self.view = pg.PlotWidget(background=theme.SURFACE)
        self.view.setMinimumHeight(220)
        plot = self.plot = self.view.getPlotItem()
        plot.setMenuEnabled(False)
        plot.hideButtons()
        plot.setMouseEnabled(x=False, y=False)
        plot.showGrid(x=True, y=True, alpha=0.28)
        for name in ("left", "bottom"):
            axis = plot.getAxis(name)
            axis.setPen(_pen(theme.AXIS, 1))
            axis.setTextPen(pg.mkPen(theme.INK_MUTED))
            axis.enableAutoSIPrefix(False)  # show N·m as N·m, not "×0.001"
        plot.getAxis("left").setWidth(56)
        self.fit_curve = plot.plot(pen=_pen(theme.CONTEXT_SERIES, 2))
        self.measured_curve = plot.plot(pen=_pen(theme.ACCENT, 2))
        self.points = pg.ScatterPlotItem(size=9, pen=pg.mkPen(theme.SURFACE, width=2), brush=pg.mkBrush(theme.ACCENT))
        plot.addItem(self.points)
        self.points.setZValue(3)
        self.measured_curve.setZValue(2)
        self.fit_curve.setZValue(1)
        self.empty = pg.TextItem("", color=theme.INK_MUTED, anchor=(0.5, 0.5))
        plot.addItem(self.empty, ignoreBounds=True)
        layout.addWidget(self.view, 1)
        self._x = np.empty(0)
        self._y = np.empty(0)
        self._units = ("", "")
        self.view.scene().sigMouseMoved.connect(self._mouse_moved)
        self.show_empty("Run the test to see its chart.")

    def _axis_values(self, shown: bool) -> None:
        for name in ("left", "bottom"):
            self.plot.getAxis(name).setStyle(showValues=shown)

    def show_empty(self, text: str) -> None:
        self.clear()
        self._axis_values(False)
        self.plot.setLabel("bottom", "")
        self.plot.setLabel("left", "")
        self.empty.setText(text)
        self.plot.setXRange(0, 1, padding=0)
        self.plot.setYRange(0, 1, padding=0)
        self.empty.setPos(0.5, 0.5)

    def clear(self) -> None:
        for item in (self.fit_curve, self.measured_curve):
            item.setData([], [])
        self.points.setData([], [])
        self.empty.setText("")
        self.readout.setText("")
        self._x, self._y = np.empty(0), np.empty(0)

    def show(self, x, y, *, x_label: str, y_label: str, x_unit: str = "", y_unit: str = "",
             fit_x=None, fit_y=None, as_points: bool = False, from_zero: bool = False) -> None:
        """Measured ``x, y`` as points or a line, the fit as a gray line, both axes titled."""
        self.clear()
        self._axis_values(True)
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        if as_points:
            self.points.setData(x, y)
        else:
            self.measured_curve.setData(x, y)
        if fit_x is not None and fit_y is not None:
            self.fit_curve.setData(np.asarray(fit_x, dtype=float), np.asarray(fit_y, dtype=float), connect="finite")
        self.plot.setLabel("bottom", x_label, units=None, color=theme.INK_MUTED)
        self.plot.setLabel("left", y_label, units=None, color=theme.INK_MUTED)
        self._x, self._y, self._units = x, y, (x_unit, y_unit)
        self.plot.enableAutoRange()
        self.plot.autoRange(padding=0.08)
        if from_zero:  # a zoomed axis would make a few percent of scatter look like a trend
            values = np.concatenate([y[np.isfinite(y)]] + ([np.asarray(fit_y, dtype=float)] if fit_y is not None else []))
            values = values[np.isfinite(values)]
            if values.size:
                low, high = min(0.0, float(values.min())), max(0.0, float(values.max()))
                pad = 0.08 * ((high - low) or 1.0)
                self.plot.setYRange(low - (pad if low < 0 else 0.0), high + pad, padding=0)

    def _mouse_moved(self, pos) -> None:
        if not self._x.size or not self.view.sceneBoundingRect().contains(pos):
            self.readout.setText("")
            return
        point = self.plot.vb.mapSceneToView(pos)
        view = self.plot.vb.viewRange()
        span_x = (view[0][1] - view[0][0]) or 1.0
        span_y = (view[1][1] - view[1][0]) or 1.0
        i = int(np.argmin(((self._x - point.x()) / span_x) ** 2 + ((self._y - point.y()) / span_y) ** 2))
        x_unit, y_unit = self._units
        self.readout.setText(f"<b>{self._y[i]:.4g}</b> {y_unit} at <b>{self._x[i]:.4g}</b> {x_unit}")

    def save_image(self, path: str) -> bool:
        return self.view.grab().save(path)
