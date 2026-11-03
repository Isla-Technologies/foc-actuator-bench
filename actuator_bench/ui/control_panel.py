"""Manual control: arm the motor in torque, speed or position mode and set a setpoint."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QGridLayout,
    QHBoxLayout,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ..odrive_api import ControlMode
from ..settings import ActuatorProfile
from ..torque import TORQUE_UNITS
from .widgets import NumberField, fmt, label

SLIDER_STEPS = 1000


class ControlPanel(QWidget):
    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.profile: ActuatorProfile | None = None
        self.armed = False
        self.armed_mode: ControlMode | None = None
        self.busy = False
        self.connected = False
        self.confirmed_this_session = False
        self.latest_angle = 0.0

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)
        root.addWidget(label(
            "Monitor torque with the motor off, or switch it on in a control mode. Operational torque is what "
            "the dashboard shows while the actuator works; record it with the Record button.", "secondary", wrap=True))

        modes = QHBoxLayout()
        modes.addWidget(label("Control mode", "secondary"))
        self.mode_group = QButtonGroup(self)
        self.mode_buttons = {}
        for mode, text in ((ControlMode.TORQUE, "Torque"), (ControlMode.VELOCITY, "Speed"),
                           (ControlMode.POSITION, "Position (hold)")):
            button = QRadioButton(text)
            self.mode_group.addButton(button)
            self.mode_buttons[mode] = button
            modes.addWidget(button)
        self.mode_buttons[ControlMode.POSITION].setChecked(True)
        for mode, button in self.mode_buttons.items():
            button.toggled.connect(lambda on, m=mode: on and self._mode_selected(m))
        modes.addStretch(1)
        root.addLayout(modes)

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)
        self.torque_field = NumberField(-1, 1, 0, decimals=3, step=0.05, suffix="N·m")
        self.torque_slider = QSlider(Qt.Horizontal)
        self.torque_slider.setRange(-SLIDER_STEPS, SLIDER_STEPS)
        self.torque_zero = QPushButton("0")
        grid.addWidget(label("Torque"), 0, 0)
        grid.addWidget(self.torque_field, 0, 1)
        grid.addWidget(self.torque_slider, 0, 2)
        grid.addWidget(self.torque_zero, 0, 3)

        self.speed_field = NumberField(-1, 1, 0, decimals=1, step=1, suffix="rpm")
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(-SLIDER_STEPS, SLIDER_STEPS)
        self.speed_zero = QPushButton("0")
        grid.addWidget(label("Speed"), 1, 0)
        grid.addWidget(self.speed_field, 1, 1)
        grid.addWidget(self.speed_slider, 1, 2)
        grid.addWidget(self.speed_zero, 1, 3)

        self.angle_field = NumberField(-100000, 100000, 0, decimals=1, step=1, suffix="°")
        jog = QHBoxLayout()
        self.jog_buttons = []
        for delta in (-10, -1, 1, 10):
            button = QPushButton(f"{delta:+d}°")
            button.clicked.connect(lambda _=False, d=delta: self._jog(d))
            jog.addWidget(button)
            self.jog_buttons.append(button)
        self.zero_button = QPushButton("Set angle zero here")
        self.zero_button.clicked.connect(self.worker.zero_angle)
        jog.addWidget(self.zero_button)
        jog.addStretch(1)
        grid.addWidget(label("Angle target"), 2, 0)
        grid.addWidget(self.angle_field, 2, 1)
        grid.addLayout(jog, 2, 2, 1, 2)
        grid.setColumnStretch(2, 1)
        root.addLayout(grid)

        row = QHBoxLayout()
        self.arm_button = QPushButton("Switch motor on")
        self.arm_button.setObjectName("primary")
        self.arm_button.setMinimumHeight(34)
        self.arm_button.clicked.connect(self._arm)
        self.disarm_button = QPushButton("Switch motor off")
        self.disarm_button.setMinimumHeight(34)
        self.disarm_button.clicked.connect(self.worker.disarm)
        row.addWidget(self.arm_button)
        row.addWidget(self.disarm_button)
        self.state_text = label("", "secondary", wrap=True)
        row.addWidget(self.state_text, 1)
        root.addLayout(row)
        self.limits_text = label("", "muted", wrap=True)
        root.addWidget(self.limits_text)
        root.addStretch(1)

        self.torque_field.valueChanged.connect(self._torque_changed)
        self.torque_slider.valueChanged.connect(lambda v: self.torque_field.setValue(v / SLIDER_STEPS * self.torque_field.maximum()))
        self.torque_zero.clicked.connect(lambda: self.torque_field.setValue(0.0))
        self.speed_field.valueChanged.connect(self._speed_changed)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_field.setValue(v / SLIDER_STEPS * self.speed_field.maximum()))
        self.speed_zero.clicked.connect(lambda: self.speed_field.setValue(0.0))
        self.angle_field.valueChanged.connect(self._angle_changed)
        self._update_enabled()

    # --- configuration ---------------------------------------------------------------------
    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        self.profile = profile
        factor = TORQUE_UNITS[profile.unit]
        model = profile.torque_model(kt_odrive)
        max_torque = model.max_output_torque(profile.max_current_a) * factor
        for field, maximum, suffix in ((self.torque_field, max_torque, profile.unit),
                                       (self.speed_field, profile.max_speed_rpm, "rpm")):
            field.blockSignals(True)
            field.setRange(-maximum, maximum)
            field.setSuffix(f" {suffix}")
            if not self.armed:  # never change what the running motor is told
                field.setValue(0.0)
            field.setSingleStep(max(maximum / 100, 0.001))
            field.blockSignals(False)
        self.limits_text.setText(
            f"Limits from Setup: {profile.max_current_a:g} A (≈ {fmt(max_torque)} {profile.unit} at the output), "
            f"{profile.max_speed_rpm:g} rpm. Torque mode with the output free makes the actuator spin up to the speed limit.")

    def set_connected(self, connected: bool) -> None:
        self.connected = connected
        if not connected:
            self.set_armed(False, None)
        self._update_enabled()

    def set_busy(self, busy: bool) -> None:
        """A test is running: manual control is locked."""
        self.busy = busy
        self._update_enabled()

    def set_armed(self, armed: bool, mode: ControlMode | None) -> None:
        self.armed, self.armed_mode = armed, mode
        if armed and mode is not None:
            button = self.mode_buttons.get(mode)
            if button and not button.isChecked():
                button.blockSignals(True)
                button.setChecked(True)
                button.blockSignals(False)
            self._reset_setpoints()
        self.state_text.setText("Motor ON" + (f" in {self._mode_name(mode)} mode." if mode is not None else ".")
                                if armed else "Motor off: torque is still measured (monitoring).")
        self._update_enabled()

    def set_latest_angle(self, angle_deg: float) -> None:
        self.latest_angle = angle_deg

    # --- actions ---------------------------------------------------------------------------
    @staticmethod
    def _mode_name(mode: ControlMode | None) -> str:
        return {ControlMode.TORQUE: "torque", ControlMode.VELOCITY: "speed", ControlMode.POSITION: "position"}.get(mode, "")

    def selected_mode(self) -> ControlMode:
        return next(mode for mode, button in self.mode_buttons.items() if button.isChecked())

    def _arm(self) -> None:
        if not self.confirmed_this_session and self.profile is not None:
            box = QMessageBox(self)
            box.setIcon(QMessageBox.Warning)
            box.setWindowTitle("Switch the motor on")
            box.setText(f"The motor will be powered in {self._mode_name(self.selected_mode())} mode.")
            box.setInformativeText(
                f"Limits: {self.profile.max_current_a:g} A and {self.profile.max_speed_rpm:g} rpm at the output.\n"
                "Make sure the actuator is clamped down and nothing is in its way. "
                "Press STOP or Esc at any time to switch it off.")
            dont_ask = QCheckBox("Don't ask again until the app restarts")
            box.setCheckBox(dont_ask)
            box.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            box.setDefaultButton(QMessageBox.Cancel)
            if box.exec() != QMessageBox.Ok:
                return
            self.confirmed_this_session = dont_ask.isChecked()
        self._reset_setpoints()
        self.worker.arm(self.selected_mode())

    def _mode_selected(self, mode: ControlMode) -> None:
        if self.armed and mode != self.armed_mode:
            self._reset_setpoints()
            self.worker.change_mode(mode)
        self._update_enabled()

    def _reset_setpoints(self) -> None:
        for field, value in ((self.torque_field, 0.0), (self.speed_field, 0.0), (self.angle_field, self.latest_angle)):
            field.blockSignals(True)
            field.setValue(value)
            field.blockSignals(False)
        for slider in (self.torque_slider, self.speed_slider):
            slider.blockSignals(True)
            slider.setValue(0)
            slider.blockSignals(False)

    def _sync_slider(self, slider: QSlider, field: NumberField) -> None:
        maximum = field.maximum() or 1.0
        slider.blockSignals(True)
        slider.setValue(int(round(field.value() / maximum * SLIDER_STEPS)))
        slider.blockSignals(False)

    def _torque_changed(self, value: float) -> None:
        self._sync_slider(self.torque_slider, self.torque_field)
        if self.armed and self.armed_mode == ControlMode.TORQUE and self.profile:
            self.worker.set_output_torque(value / TORQUE_UNITS[self.profile.unit])

    def _speed_changed(self, value: float) -> None:
        self._sync_slider(self.speed_slider, self.speed_field)
        if self.armed and self.armed_mode == ControlMode.VELOCITY:
            self.worker.set_output_speed(value)

    def _angle_changed(self, value: float) -> None:
        if self.armed and self.armed_mode == ControlMode.POSITION:
            self.worker.set_output_angle(value)

    def _jog(self, delta: float) -> None:
        self.angle_field.setValue(self.angle_field.value() + delta)

    def _update_enabled(self) -> None:
        mode = self.selected_mode()
        can_control = self.connected and not self.busy
        self.arm_button.setEnabled(can_control and not self.armed)
        self.disarm_button.setEnabled(self.connected and self.armed and not self.busy)
        for button in self.mode_buttons.values():
            button.setEnabled(can_control)
        active = self.armed and not self.busy
        for widget in (self.torque_field, self.torque_slider, self.torque_zero):
            widget.setEnabled(active and mode == ControlMode.TORQUE)
        for widget in (self.speed_field, self.speed_slider, self.speed_zero):
            widget.setEnabled(active and mode == ControlMode.VELOCITY)
        for widget in (self.angle_field, *self.jog_buttons):
            widget.setEnabled(active and mode == ControlMode.POSITION)
        self.zero_button.setEnabled(self.connected)
