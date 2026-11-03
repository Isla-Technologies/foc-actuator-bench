"""Stall test and hold test tabs."""

from __future__ import annotations

import math

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..procedures import HoldTest, HoldTestParams, ProcedureStatus, StallTest, StallTestParams
from ..settings import ActuatorProfile
from ..torque import TORQUE_UNITS
from .widgets import NumberField, StatTile, fixed, fmt, form_grid, label


class _TestPanel(QWidget):
    open_folder = Signal()
    procedure_name = ""

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.profile: ActuatorProfile | None = None
        self.kt_odrive: float | None = None
        self.connected = False
        self.running = False
        self.busy = False

    def _build_common(self, root: QVBoxLayout, start_text: str) -> None:
        row = QHBoxLayout()
        self.start_button = QPushButton(start_text)
        self.start_button.setObjectName("primary")
        self.start_button.setMinimumHeight(34)
        self.start_button.clicked.connect(self._start)
        self.stop_button = QPushButton("Stop test")
        self.stop_button.setMinimumHeight(34)
        self.stop_button.clicked.connect(self.worker.stop_procedure)
        row.addWidget(self.start_button)
        row.addWidget(self.stop_button)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setTextVisible(False)
        row.addWidget(self.progress, 1)
        self.phase = label("", "secondary")
        self.phase.setMinimumWidth(130)
        row.addWidget(self.phase)
        root.addLayout(row)
        self.message = label("", "secondary", wrap=True)
        root.addWidget(self.message)

    @property
    def factor(self) -> float:
        return TORQUE_UNITS[self.profile.unit] if self.profile else 1.0

    @property
    def unit(self) -> str:
        return self.profile.unit if self.profile else "N·m"

    def max_torque(self) -> float:
        """Largest output torque allowed by the current limit, in display units."""
        if not self.profile:
            return 1.0
        model = self.profile.torque_model(self.kt_odrive)
        return model.max_output_torque(self.profile.max_current_a) * self.factor

    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        self.profile, self.kt_odrive = profile, kt_odrive

    def set_connected(self, connected: bool) -> None:
        self.connected = connected
        self._update_enabled()

    def set_busy(self, busy: bool) -> None:
        self.busy = busy
        self._update_enabled()

    def _update_enabled(self) -> None:
        self.start_button.setEnabled(self.connected and not self.busy)
        self.stop_button.setEnabled(self.running)

    def _start(self) -> None:
        raise NotImplementedError

    def show_status(self, status: ProcedureStatus) -> None:
        self.running = not status.done
        self.progress.setValue(int(status.progress * 1000))
        self.phase.setText(status.phase if not status.done else ("Done" if status.success else "Stopped"))
        if status.done:
            prefix = "✔ " if status.success else "▲ "
            self.message.setText(prefix + status.message)
            if status.result:
                self.show_result(status.result)
        else:
            self.message.setText("Running… press STOP (Esc) to switch the motor off immediately.")
        self._update_enabled()

    def show_result(self, result: dict) -> None:
        raise NotImplementedError

    def show_saved(self, folder: str) -> None:
        self.message.setText(self.message.text() + f"\nResults saved to {folder}")


class StallTestPanel(_TestPanel):
    procedure_name = StallTest.name

    def __init__(self, worker, parent=None):
        super().__init__(worker, parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)
        root.addWidget(label(
            "<b>Peak (stall) torque.</b> 1. Press the arm firmly against a solid object (bench, clamp, stop) in the "
            "direction you test. 2. Enter a target torque. 3. Start: the torque ramps up, holds, then releases. "
            "The test stops by itself if the arm moves, so it only measures a real stall. Keep hands clear.",
            "secondary", wrap=True))
        grid = form_grid()
        self.target = NumberField(0.001, 1000, 1.0, decimals=3, step=0.1, suffix="N·m")
        self.direction = QComboBox()
        self.direction.addItem("Positive (+)", 1)
        self.direction.addItem("Negative (−)", -1)
        self.ramp = NumberField(0.2, 60, 3.0, decimals=1, step=0.5, suffix="s")
        self.hold = NumberField(0.0, 10, 1.0, decimals=1, step=0.5, suffix="s")
        self.movement = NumberField(0.5, 90, 10.0, decimals=1, step=1, suffix="°")
        self.max_info = label("", "muted")
        grid.addWidget(label("Target torque"), 0, 0)
        grid.addWidget(self.target, 0, 1)
        grid.addWidget(self.max_info, 0, 2, 1, 2)
        grid.addWidget(label("Direction"), 1, 0)
        grid.addWidget(self.direction, 1, 1)
        grid.addWidget(label("Ramp-up time"), 2, 0)
        grid.addWidget(self.ramp, 2, 1)
        grid.addWidget(label("Hold at target"), 3, 0)
        grid.addWidget(self.hold, 3, 1)
        grid.addWidget(label("keep short: a stalled motor heats up fast", "muted"), 3, 2, 1, 2)
        grid.addWidget(label("Stop if the arm moves more than"), 4, 0)
        grid.addWidget(self.movement, 4, 1)
        root.addLayout(grid)
        self._build_common(root, "Start stall test")
        results = QGridLayout()
        results.setHorizontalSpacing(24)
        self.r_peak = StatTile("Peak torque")
        self.r_current = StatTile("Current at peak", "A")
        self.r_hold = StatTile("Average while holding")
        self.r_move = StatTile("Arm movement", "°")
        self.r_temp = StatTile("Power stage temperature", "°C")
        for col, tile in enumerate((self.r_peak, self.r_current, self.r_hold, self.r_move, self.r_temp)):
            results.addWidget(tile, 0, col)
        root.addLayout(results)
        root.addStretch(1)
        self._update_enabled()

    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        super().set_profile(profile, kt_odrive)
        maximum = self.max_torque()
        self.target.setSuffix(f" {profile.unit}")
        self.target.setMaximum(maximum)
        if self.target.value() > maximum:
            self.target.setValue(maximum * 0.5)
        self.max_info.setText(f"max {fmt(maximum)} {profile.unit} with your {profile.max_current_a:g} A limit")
        for tile in (self.r_peak, self.r_hold):
            tile.set_unit(profile.unit)

    def _start(self) -> None:
        target = self.target.value() / self.factor * int(self.direction.currentData())
        params = StallTestParams(target_torque_out=target, ramp_time_s=self.ramp.value(), hold_time_s=self.hold.value(),
                                 ramp_down_time_s=0.5, max_movement_deg=self.movement.value())
        for tile in (self.r_peak, self.r_current, self.r_hold, self.r_move, self.r_temp):
            tile.set("—")
        self.running = True
        self._update_enabled()
        self.worker.start_procedure(StallTest(params))

    def show_result(self, r: dict) -> None:
        f = self.factor
        self.r_peak.set(fmt(r["peak_torque_Nm"] * f, 4))
        self.r_current.set(fixed(r["current_at_peak_A"], 2))
        self.r_hold.set(fmt(r["hold_mean_torque_Nm"] * f, 4))
        self.r_move.set(fixed(r["max_movement_deg"], 2))
        start, end = r.get("fet_temp_start_C", math.nan), r.get("fet_temp_end_C", math.nan)
        self.r_temp.set(f"{fixed(start, 0)} → {fixed(end, 0)}" if math.isfinite(start) else "—")


class HoldTestPanel(_TestPanel):
    procedure_name = HoldTest.name

    def __init__(self, worker, parent=None):
        super().__init__(worker, parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)
        root.addWidget(label(
            "<b>Holding torque.</b> The motor holds its current position while you apply the load: push the arm, "
            "hang a weight, or run your rig. The app records how much torque it takes to hold. The torque limit "
            "below decides when the actuator gives way.", "secondary", wrap=True))
        grid = form_grid()
        self.limit = NumberField(0.001, 1000, 1.0, decimals=3, step=0.1, suffix="N·m")
        self.duration = NumberField(0, 3600, 15, decimals=0, step=5, suffix="s")
        self.deflection = NumberField(1, 180, 30, decimals=0, step=5, suffix="°")
        self.max_info = label("", "muted")
        grid.addWidget(label("Torque limit"), 0, 0)
        grid.addWidget(self.limit, 0, 1)
        grid.addWidget(self.max_info, 0, 2, 1, 2)
        grid.addWidget(label("Duration"), 1, 0)
        grid.addWidget(self.duration, 1, 1)
        grid.addWidget(label("0 = until you press Stop test", "muted"), 1, 2, 1, 2)
        grid.addWidget(label("Stop if pushed further than"), 2, 0)
        grid.addWidget(self.deflection, 2, 1)
        root.addLayout(grid)
        self._build_common(root, "Start hold test")
        results = QGridLayout()
        results.setHorizontalSpacing(24)
        self.r_peak = StatTile("Peak torque")
        self.r_steady = StatTile("Steady holding torque")
        self.r_rms = StatTile("RMS torque")
        self.r_deflect = StatTile("Max deflection", "°")
        for col, tile in enumerate((self.r_peak, self.r_steady, self.r_rms, self.r_deflect)):
            results.addWidget(tile, 0, col)
        root.addLayout(results)
        root.addStretch(1)
        self._update_enabled()

    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        super().set_profile(profile, kt_odrive)
        maximum = self.max_torque()
        self.limit.setSuffix(f" {profile.unit}")
        self.limit.setMaximum(maximum)
        if self.limit.value() >= maximum or self.limit.value() == 1.0:
            self.limit.setValue(maximum * 0.8)
        self.max_info.setText(f"max {fmt(maximum)} {profile.unit} with your {profile.max_current_a:g} A limit")
        for tile in (self.r_peak, self.r_steady, self.r_rms):
            tile.set_unit(profile.unit)

    def _start(self) -> None:
        params = HoldTestParams(max_torque_out=self.limit.value() / self.factor, duration_s=self.duration.value(),
                                max_deflection_deg=self.deflection.value())
        for tile in (self.r_peak, self.r_steady, self.r_rms, self.r_deflect):
            tile.set("—")
        self.running = True
        self._update_enabled()
        self.worker.start_procedure(HoldTest(params))

    def show_result(self, r: dict) -> None:
        f = self.factor
        self.r_peak.set(fmt(r["peak_abs_Nm"] * f, 4))
        self.r_steady.set(fmt(r["steady_torque_Nm"] * f, 4))
        self.r_rms.set(fmt(r["rms_Nm"] * f, 4))
        self.r_deflect.set(fixed(r["max_deflection_deg"], 2))
