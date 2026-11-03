"""Known-weight calibration wizard (see calibration.py for the method)."""

from __future__ import annotations

from datetime import date

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHBoxLayout,
    QHeaderView,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..calibration import CalibrationFit, CalibrationPoint, fit_calibration
from ..odrive_api import ControlMode
from ..procedures import CalibrationSweep, CalibrationSweepParams, ProcedureStatus
from ..settings import ActuatorProfile
from .widgets import NumberField, form_grid, label, section_title


class CalibrationPanel(QWidget):
    procedure_name = CalibrationSweep.name
    calibration_saved = Signal(object)  # CalibrationFit

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.profile: ActuatorProfile | None = None
        self.connected = False
        self.busy = False
        self.running = False
        self.armed = False
        self.armed_mode: ControlMode | None = None
        self.is_simulator = False
        self.sim_scenario = ""
        self.latest_pos: float | None = None
        self.horizontal_pos: float | None = None
        self.points: list[CalibrationPoint] = []
        self.fit: CalibrationFit | None = None

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(8)
        root.addWidget(label(
            "<b>Weight calibration</b> measures the real output torque per amp, which corrects gearbox losses and "
            "an inexact Kt. You need a rigid arm of known length on the output and two or three known weights "
            "(for example 0, 0.5 and 1 kg). The arm sweeps slowly ±10° around horizontal for each weight.",
            "secondary", wrap=True))
        self.sim_note = label("", "muted", wrap=True)
        root.addWidget(self.sim_note)

        root.addWidget(section_title("1  Arm"))
        grid = form_grid()
        self.arm_length = NumberField(0.01, 5, 0.30, decimals=3, step=0.01, suffix="m")
        self.amplitude = NumberField(4, 45, 10, decimals=1, step=1, suffix="°")
        self.speed = NumberField(1, 90, 10, decimals=1, step=1, suffix="°/s")
        self.cycles = NumberField(1, 10, 2, decimals=0, step=1, suffix="cycles")
        grid.addWidget(label("Arm length (axis to weight)"), 0, 0)
        grid.addWidget(self.arm_length, 0, 1)
        grid.addWidget(label("Sweep ±"), 0, 2)
        grid.addWidget(self.amplitude, 0, 3)
        grid.addWidget(label("Sweep speed"), 1, 0)
        grid.addWidget(self.speed, 1, 1)
        grid.addWidget(label("Repeat"), 1, 2)
        grid.addWidget(self.cycles, 1, 3)
        grid.setColumnStretch(3, 0)
        grid.setColumnStretch(4, 1)
        root.addLayout(grid)

        root.addWidget(section_title("2  Horizontal position"))
        row = QHBoxLayout()
        self.hold_button = QPushButton("Hold the arm (motor on)")
        self.hold_button.clicked.connect(self._hold)
        row.addWidget(self.hold_button)
        self.jog_buttons = []
        for delta in (-1.0, -0.2, 0.2, 1.0):
            button = QPushButton(f"{delta:+.1f}°")
            button.clicked.connect(lambda _=False, d=delta: self.worker.jog(d))
            row.addWidget(button)
            self.jog_buttons.append(button)
        self.set_horizontal_button = QPushButton("Arm is level: set horizontal")
        self.set_horizontal_button.setObjectName("primary")
        self.set_horizontal_button.clicked.connect(self._set_horizontal)
        row.addWidget(self.set_horizontal_button)
        row.addStretch(1)
        root.addLayout(row)
        self.horizontal_text = label("Hold the arm level by hand (motor off) or switch the motor on and jog it level, "
                                     "check with a spirit level, then press “set horizontal”.", "muted", wrap=True)
        root.addWidget(self.horizontal_text)

        root.addWidget(section_title("3  Weights"))
        row = QHBoxLayout()
        row.addWidget(label("Weight at the end of the arm"))
        self.mass = NumberField(0, 1000, 0.0, decimals=3, step=0.1, suffix="kg")
        row.addWidget(self.mass)
        self.measure_button = QPushButton("Measure this weight")
        self.measure_button.setObjectName("primary")
        self.measure_button.clicked.connect(self._measure)
        row.addWidget(self.measure_button)
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.worker.stop_procedure)
        row.addWidget(self.stop_button)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setTextVisible(False)
        row.addWidget(self.progress, 1)
        root.addLayout(row)
        self.message = label("", "secondary", wrap=True)
        root.addWidget(self.message)

        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Weight (kg)", "Gravity torque (N·m)", "Current (A)", "Friction (A)"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setMaximumHeight(140)
        root.addWidget(self.table)
        row = QHBoxLayout()
        remove = QPushButton("Remove selected")
        remove.clicked.connect(self._remove_selected)
        clear = QPushButton("Clear all")
        clear.clicked.connect(self._clear_points)
        row.addWidget(remove)
        row.addWidget(clear)
        row.addStretch(1)
        self.release_button = QPushButton("Release the arm (motor off)…")
        self.release_button.clicked.connect(self._release)
        row.addWidget(self.release_button)
        root.addLayout(row)

        root.addWidget(section_title("4  Result"))
        row = QHBoxLayout()
        self.compute_button = QPushButton("Compute calibration")
        self.compute_button.clicked.connect(self._compute)
        self.save_button = QPushButton("Save to profile")
        self.save_button.setObjectName("primary")
        self.save_button.clicked.connect(self._save)
        row.addWidget(self.compute_button)
        row.addWidget(self.save_button)
        row.addStretch(1)
        root.addLayout(row)
        self.result_text = label("", wrap=True)
        root.addWidget(self.result_text)
        root.addStretch(1)
        self._update_enabled()

    # --- state from the main window ------------------------------------------------------------
    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        self.profile = profile

    def set_connected(self, connected: bool, is_simulator: bool = False, scenario: str = "") -> None:
        self.connected = connected
        self.is_simulator, self.sim_scenario = is_simulator, scenario
        if not connected:
            self.horizontal_pos = None
            self.latest_pos = None
            self.horizontal_text.setText("Horizontal: not set.")
        if is_simulator and scenario != "lever":
            self.sim_note.setText("Simulator: choose “Arm with a hanging weight” on the Connect page for this test.")
        elif is_simulator:
            self.sim_note.setText("Simulator: the arm starts level, so you can press “set horizontal” right away. "
                                  "Weights are hung automatically.")
        else:
            self.sim_note.setText("")
        self._update_enabled()

    def set_busy(self, busy: bool) -> None:
        self.busy = busy
        self._update_enabled()

    def set_armed(self, armed: bool, mode: ControlMode | None) -> None:
        self.armed, self.armed_mode = armed, mode
        self._update_enabled()

    def set_latest_position(self, motor_turns: float) -> None:
        first = self.latest_pos is None
        self.latest_pos = motor_turns
        if first:
            self._update_enabled()

    def _update_enabled(self) -> None:
        idle = self.connected and not self.busy
        self.hold_button.setEnabled(idle and not self.armed)
        for button in self.jog_buttons:
            button.setEnabled(idle and self.armed and self.armed_mode == ControlMode.POSITION)
        self.set_horizontal_button.setEnabled(idle and self.latest_pos is not None)
        self.measure_button.setEnabled(idle and self.horizontal_pos is not None)
        self.stop_button.setEnabled(self.running)
        self.release_button.setEnabled(self.connected and self.armed and not self.running)
        distinct = {round(p.mass_kg, 6) for p in self.points}
        self.compute_button.setEnabled(len(distinct) >= 2 and not self.running)
        self.save_button.setEnabled(self.fit is not None and not self.running)

    # --- actions ---------------------------------------------------------------------------
    def _hold(self) -> None:
        self.worker.arm(ControlMode.POSITION)

    def _set_horizontal(self) -> None:
        if self.latest_pos is None:
            return
        self.horizontal_pos = self.latest_pos
        self.horizontal_text.setText(f"Horizontal set (motor position {self.horizontal_pos:.4f} turns). "
                                     "Now hang the first weight (0 kg = the bare arm).")
        self._update_enabled()

    def _measure(self) -> None:
        if self.horizontal_pos is None:
            return
        mass = self.mass.value()
        if self.is_simulator and self.sim_scenario == "lever":
            self.worker.configure_simulator(mass_kg=mass)
        params = CalibrationSweepParams(mass_kg=mass, arm_length_m=self.arm_length.value(),
                                        horizontal_pos=self.horizontal_pos, amplitude_deg=self.amplitude.value(),
                                        speed_deg_s=self.speed.value(), cycles=int(self.cycles.value()))
        self.running = True
        self.message.setText(f"Measuring {mass:g} kg…")
        self._update_enabled()
        self.worker.start_procedure(CalibrationSweep(params))

    def show_status(self, status: ProcedureStatus) -> None:
        self.running = not status.done
        self.progress.setValue(int(status.progress * 1000))
        if not status.done:
            self.message.setText(f"{status.phase.capitalize()}…")
        elif status.success and status.result:
            point: CalibrationPoint = status.result["point"]
            self.points.append(point)
            self._render_points()
            self.message.setText(f"✔ {status.message} Hang the next weight, or compute the calibration.")
        else:
            self.message.setText(f"▲ {status.message}")
        self._update_enabled()

    def _render_points(self) -> None:
        self.table.setRowCount(len(self.points))
        for row, p in enumerate(self.points):
            for col, text in enumerate((f"{p.mass_kg:g}", f"{p.torque_ref_nm:.4f}", f"{p.iq_static_a:.4f}",
                                        f"{p.iq_friction_a:.4f}")):
                self.table.setItem(row, col, QTableWidgetItem(text))
        self.fit = None
        self.result_text.setText("")

    def _remove_selected(self) -> None:
        rows = sorted({index.row() for index in self.table.selectedIndexes()}, reverse=True)
        for row in rows:
            del self.points[row]
        self._render_points()
        self._update_enabled()

    def _clear_points(self) -> None:
        self.points.clear()
        self._render_points()
        self._update_enabled()

    def _release(self) -> None:
        answer = QMessageBox.warning(self, "Release the arm",
                                     "The motor switches off and the arm will drop. Support the arm or remove the "
                                     "weight first. Continue?", QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)
        if answer == QMessageBox.Yes:
            self.worker.disarm()

    def _compute(self) -> None:
        try:
            self.fit = fit_calibration(self.points)
        except ValueError as exc:
            self.result_text.setText(f"▲ {exc}")
            self.fit = None
            self._update_enabled()
            return
        fit = self.fit
        text = (f"<b>{fit.k_out_nm_per_a:.4f} N·m per amp</b> at the output "
                f"(fit R² {fit.r_squared:.5f}, largest deviation {fit.max_residual_nm:.4f} N·m).")
        if self.profile:
            nominal = self.profile.kt_motor * self.profile.gear_ratio
            text += (f"<br>Nominal Kt × gear ratio = {nominal:.4f} N·m/A, so the actuator delivers "
                     f"{fit.implied_efficiency(self.profile.kt_motor, self.profile.gear_ratio) * 100:.1f} % of it "
                     "(gearbox efficiency and Kt error together).")
        if fit.r_squared < 0.995:
            text += "<br>▲ The points don't lie on a straight line; check the arm length, the weights and that the arm was level."
        self.result_text.setText(text)
        self._update_enabled()

    def _save(self) -> None:
        if self.fit is None:
            return
        self.calibration_saved.emit(self.fit)
        self.result_text.setText(self.result_text.text() + f"<br>✔ Saved to the profile on {date.today().isoformat()}. "
                                 "Torque readings now use this calibration.")

    def reset_results(self) -> None:
        self.progress.setValue(0)
        self.message.setText("")
