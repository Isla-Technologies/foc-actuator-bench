"""Step 3: live dashboard, manual control, tests, calibration and recording."""

from __future__ import annotations

import math
import time

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..device import SlowReading
from ..odrive_api import AxisState, state_label
from ..samples import SAMPLE_FIELDS, SampleBatch
from ..settings import ActuatorProfile, AppSettings
from ..torque import TORQUE_UNITS, LowPass, TorqueStats
from .calibration_panel import CalibrationPanel
from .control_panel import ControlPanel
from .live_plot import LivePlots, RingBuffer
from .tests_panel import HoldTestPanel, StallTestPanel
from .widgets import HeroReadout, StatTile, StatusPill, card, fixed, fmt, label

BUFFER_SECONDS = 60
MAX_RATE_HZ = 500


def _scrollable(widget: QWidget) -> QScrollArea:
    area = QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QScrollArea.NoFrame)
    area.setStyleSheet("QScrollArea { background: transparent; }")
    area.viewport().setAutoFillBackground(False)
    area.setWidget(widget)
    return area


class MeasurePage(QWidget):
    back = Signal()
    record_toggled = Signal(bool)
    open_data_folder = Signal()
    clear_errors_requested = Signal()

    def __init__(self, worker, settings: AppSettings, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.settings = settings
        self.profile: ActuatorProfile = settings.profile()
        self.buffer = RingBuffer(SAMPLE_FIELDS, BUFFER_SECONDS * MAX_RATE_HZ)
        self.lowpass = LowPass(settings.display_filter_ms / 1000.0)
        self.stats = TorqueStats()
        self.filtered_torque = math.nan
        self.slow: SlowReading | None = None
        self.armed = False
        self._banner_until = 0.0

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 10, 16, 12)
        root.setSpacing(10)
        root.addLayout(self._top_bar())

        splitter = QSplitter(Qt.Vertical)
        splitter.setChildrenCollapsible(False)
        upper = QWidget()
        upper_layout = QHBoxLayout(upper)
        upper_layout.setContentsMargins(0, 0, 0, 0)
        upper_layout.setSpacing(12)
        upper_layout.addWidget(self._readouts_card(), 0)
        plots_card, plots_layout = card(margins=(12, 10, 12, 8))
        plots_layout.addLayout(self._kpi_strip())
        self.plots = LivePlots()
        plots_layout.addWidget(self.plots, 1)
        upper_layout.addWidget(plots_card, 1)
        splitter.addWidget(upper)

        self.tabs = QTabWidget()
        self.control = ControlPanel(worker)
        self.stall = StallTestPanel(worker)
        self.hold = HoldTestPanel(worker)
        self.calibration = CalibrationPanel(worker)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        for widget, title in ((self.control, "Manual control"), (self.stall, "Stall test (peak torque)"),
                              (self.hold, "Hold test"), (self.calibration, "Calibration")):
            self.tabs.addTab(_scrollable(widget), title)
        self.tabs.addTab(self.log_view, "Log")
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([480, 300])
        root.addWidget(splitter, 1)
        root.addLayout(self._record_bar())

        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(lambda: self.plots.update_from(self.buffer))
        self.plot_timer.start(33)
        self.readout_timer = QTimer(self)
        self.readout_timer.timeout.connect(self._update_readouts)
        self.readout_timer.start(100)
        self.set_profile(self.profile)

    # --- layout ----------------------------------------------------------------------------
    def _top_bar(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(10)
        self.pill = StatusPill()
        row.addWidget(self.pill)
        self.device_text = label("", "secondary")
        row.addWidget(self.device_text)
        self.banner = label("", wrap=True)
        row.addWidget(self.banner, 1)
        self.clear_button = QPushButton("Clear errors")
        self.clear_button.clicked.connect(self.clear_errors_requested.emit)
        row.addWidget(self.clear_button)
        back = QPushButton("←  Setup")
        back.clicked.connect(self.back.emit)
        row.addWidget(back)
        return row

    def _readouts_card(self):
        frame, layout = card(margins=(16, 12, 16, 12), spacing=10)
        frame.setMinimumWidth(300)
        frame.setMaximumWidth(360)
        self.hero = HeroReadout("Output torque")
        layout.addWidget(self.hero)
        stats = QGridLayout()
        stats.setHorizontalSpacing(16)
        self.t_peak = StatTile("Peak", value_size=17)
        self.t_mean = StatTile("Mean", value_size=17)
        self.t_rms = StatTile("RMS", value_size=17)
        stats.addWidget(self.t_peak, 0, 0)
        stats.addWidget(self.t_mean, 0, 1)
        stats.addWidget(self.t_rms, 0, 2)
        layout.addLayout(stats)
        row = QHBoxLayout()
        self.stats_text = label("since reset", "muted", size=12)
        row.addWidget(self.stats_text, 1)
        reset = QPushButton("Reset")
        reset.clicked.connect(self.reset_stats)
        row.addWidget(reset)
        layout.addLayout(row)
        layout.addStretch(1)
        return frame

    def _kpi_strip(self) -> QGridLayout:
        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(6)
        self.t_motor = StatTile("Motor torque", value_size=16)
        self.t_current = StatTile("Motor current", "A", value_size=16)
        self.t_speed = StatTile("Output speed", "rpm", value_size=16)
        self.t_angle = StatTile("Output angle", "°", value_size=16)
        self.t_vbus = StatTile("Bus voltage", "V", value_size=16)
        self.t_power = StatTile("Electrical power", "W", value_size=16)
        self.t_fet = StatTile("ODrive temperature", "°C", value_size=16)
        self.t_rate = StatTile("Sample rate", "Hz", value_size=16)
        for i, tile in enumerate((self.t_motor, self.t_current, self.t_speed, self.t_angle,
                                  self.t_vbus, self.t_power, self.t_fet, self.t_rate)):
            grid.addWidget(tile, 0, i)
        return grid

    def _record_bar(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self.record_button = QPushButton("●  Record")
        self.record_button.setObjectName("record")
        self.record_button.setCheckable(True)
        self.record_button.setMinimumHeight(32)
        self.record_button.toggled.connect(self._record_clicked)
        row.addWidget(self.record_button)
        self.record_text = label("Records every sample to a CSV file (opens in Excel, Numbers, Python…).", "secondary")
        row.addWidget(self.record_text, 1)
        save_plot = QPushButton("Save chart image…")
        save_plot.clicked.connect(self._save_plot)
        row.addWidget(save_plot)
        folder = QPushButton("Open data folder")
        folder.clicked.connect(self.open_data_folder.emit)
        row.addWidget(folder)
        return row

    # --- configuration ---------------------------------------------------------------------
    @property
    def factor(self) -> float:
        return TORQUE_UNITS[self.profile.unit]

    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        self.profile = profile
        unit = profile.unit
        self.hero.unit.setText(unit)
        for tile in (self.t_peak, self.t_mean, self.t_rms, self.t_motor):
            tile.set_unit(unit)
        self.plots.set_unit(unit, self.factor)
        model = profile.torque_model(kt_odrive)
        self.plots.set_scale(model.max_output_torque(profile.max_current_a) * self.factor,
                             profile.max_current_a, profile.max_speed_rpm)
        source = "weight calibration" if model.is_calibrated else "Kt × current × gear ratio × efficiency"
        smoothing = self.settings.display_filter_ms
        self.hero.note.setText(f"From {source} · {model.k_out:.4f} N·m/A"
                               + (f" · smoothed {smoothing:g} ms" if smoothing else ""))
        self.lowpass = LowPass(smoothing / 1000.0)
        for panel in (self.control, self.stall, self.hold, self.calibration):
            panel.set_profile(profile, kt_odrive)

    def set_connection(self, connected: bool, info=None, is_simulator: bool = False, scenario: str = "") -> None:
        if connected:
            self.buffer.clear()
            self.plots.clear()
            self.lowpass.reset()
            self.reset_stats()
            self.device_text.setText(f"ODrive {info.hardware} · fw {info.firmware} · {info.connection}" if info else "")
        else:
            self.pill.set("neutral", "Not connected")
            self.device_text.setText("")
            if self.record_button.isChecked():
                self.record_button.setChecked(False)
        self.control.set_connected(connected)
        self.stall.set_connected(connected)
        self.hold.set_connected(connected)
        self.calibration.set_connected(connected, is_simulator, scenario)

    # --- live data -------------------------------------------------------------------------
    def add_batch(self, batch: SampleBatch) -> None:
        if not len(batch):
            return
        self.buffer.extend(batch.columns)
        filtered = self.lowpass.apply(batch["t"], batch["torque_out"])
        self.filtered_torque = float(filtered[-1])
        self.stats.add(batch["t"], filtered)
        self.control.set_latest_angle(float(batch["angle_out_deg"][-1]))
        self.calibration.set_latest_position(float(batch["pos"][-1]))

    def set_slow(self, slow: SlowReading) -> None:
        self.slow = slow
        if slow.errors:
            self.pill.set("critical", "Error")
        elif slow.state == AxisState.CLOSED_LOOP_CONTROL:
            self.pill.set("warning", "Motor ON")
        elif slow.state is not None and slow.state not in (AxisState.IDLE, AxisState.UNDEFINED):
            self.pill.set("info", state_label(slow.state))
        else:
            self.pill.set("neutral", "Motor off (monitoring)")
        self.clear_button.setVisible(bool(slow.errors))
        if slow.errors and time.monotonic() > self._banner_until:
            names = ", ".join(f"{e.name} ({e.explanation})" for e in slow.errors[:2])
            self._set_banner("critical", names)

    def set_armed(self, armed: bool, mode) -> None:
        self.armed = armed
        self.control.set_armed(armed, mode)
        self.calibration.set_armed(armed, mode)

    def set_busy(self, busy: bool) -> None:
        self.control.set_busy(busy)
        self.stall.set_busy(busy)
        self.hold.set_busy(busy)
        self.calibration.set_busy(busy)

    def show_message(self, level: str, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_view.appendPlainText(f"{stamp}  {level.upper():7s} {text}")
        if level in ("warning", "error"):
            self._set_banner("critical" if level == "error" else "warning", text)
            self._banner_until = time.monotonic() + 8.0

    def _set_banner(self, kind: str, text: str) -> None:
        icon, color = {"critical": ("✖", "#9b2222"), "warning": ("▲", "#7a5200")}[kind]
        self.banner.setText(f"{icon}  {text}")
        self.banner.setStyleSheet(f"color: {color};")

    def reset_stats(self) -> None:
        self.stats.reset()

    def show_panel(self, panel: QWidget) -> None:
        """Select the tab that contains ``panel`` (tabs wrap panels in scroll areas)."""
        for i in range(self.tabs.count()):
            page = self.tabs.widget(i)
            if page is panel or (isinstance(page, QScrollArea) and page.widget() is panel):
                self.tabs.setCurrentIndex(i)
                return

    def _update_readouts(self) -> None:
        if not len(self.buffer):
            return
        f = self.factor
        latest = {name: float(self.buffer.last(name, 1)[0]) for name in
                  ("torque_motor", "iq", "speed_out_rpm", "angle_out_deg", "power_elec")}
        self.hero.value.setText(fixed(self.filtered_torque * f, 3))
        self.t_peak.set(fmt(self.stats.peak_abs * f, 4))
        self.t_mean.set(fmt(self.stats.mean * f, 4))
        self.t_rms.set(fmt(self.stats.rms * f, 4))
        self.stats_text.setText(f"over {self.stats.duration:.1f} s since reset (smoothed values)")
        self.t_motor.set(fmt(latest["torque_motor"] * f, 3))
        self.t_current.set(fixed(latest["iq"], 2))
        self.t_speed.set(fixed(latest["speed_out_rpm"], 1))
        self.t_angle.set(fixed(latest["angle_out_deg"], 1))
        slow = self.slow
        self.t_vbus.set(fixed(slow.vbus, 1) if slow else "—")
        self.t_power.set(fixed(latest["power_elec"], 1))
        self.t_fet.set(fixed(slow.fet_temp, 0) if slow else "—")
        self.t_rate.set(f"{slow.sample_rate_hz:.0f}" if slow and slow.sample_rate_hz else "—")
        if time.monotonic() > self._banner_until and not (slow and slow.errors):
            self.banner.setText("")

    def latest_values(self) -> dict[str, np.ndarray]:
        return {name: self.buffer.last(name) for name in SAMPLE_FIELDS}

    # --- recording -------------------------------------------------------------------------
    def _record_clicked(self, on: bool) -> None:
        self.record_button.setText("■  Stop recording" if on else "●  Record")
        self.record_toggled.emit(on)

    def set_record_text(self, text: str) -> None:
        self.record_text.setText(text)

    def _save_plot(self) -> None:
        from PySide6.QtWidgets import QFileDialog

        from ..recorder import timestamp

        default = str(self.settings.data_path / f"{timestamp()}_chart.png")
        self.settings.data_path.mkdir(parents=True, exist_ok=True)
        path, _ = QFileDialog.getSaveFileName(self, "Save chart image", default, "PNG image (*.png)")
        if path:
            self.plots.save_image(path)
