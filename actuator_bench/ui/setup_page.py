"""Step 1: the user enters the actuator's values before anything is measured."""

from __future__ import annotations

import copy

from PySide6.QtCore import Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ..settings import ActuatorProfile, AppSettings
from ..torque import KT_PER_KV, TORQUE_UNITS
from .widgets import NumberField, card, fmt, form_grid, label, section_title


class SetupPage(QWidget):
    completed = Signal(object)  # ActuatorProfile

    def __init__(self, settings: AppSettings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self._loading = False
        self.errors: dict[str, QLabel] = {}

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        outer.addWidget(scroll)
        body = QWidget()
        scroll.setWidget(body)
        root = QVBoxLayout(body)
        root.setContentsMargins(24, 18, 24, 18)
        root.setSpacing(14)

        root.addWidget(label("Enter your actuator's values", "pageTitle", size=22, weight=QFont.DemiBold))
        root.addWidget(label(
            "The ODrive measures motor current; these values turn that current into torque and set the "
            "safety limits for every test. You can come back and change them at any time.", "secondary", wrap=True))

        root.addLayout(self._profile_row())

        columns = QHBoxLayout()
        columns.setSpacing(14)
        left = QVBoxLayout()
        left.setSpacing(14)
        right = QVBoxLayout()
        right.setSpacing(14)
        left.addWidget(self._motor_card())
        left.addWidget(self._gearbox_card())
        left.addStretch(1)
        right.addWidget(self._limits_card())
        right.addWidget(self._display_card())
        right.addWidget(self._calibration_card())
        right.addStretch(1)
        columns.addLayout(left, 1)
        columns.addLayout(right, 1)
        root.addLayout(columns)

        footer = QHBoxLayout()
        self.summary = label("", "secondary", wrap=True)
        footer.addWidget(self.summary, 1)
        self.next_button = QPushButton("Next: Connect  →")
        self.next_button.setObjectName("primary")
        self.next_button.setMinimumHeight(36)
        self.next_button.clicked.connect(self._finish)
        footer.addWidget(self.next_button)
        root.addLayout(footer)

        self.load_profile(settings.profile())

    # --- layout ----------------------------------------------------------------------------
    def _profile_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(label("Profile", "secondary"))
        self.profile_combo = QComboBox()
        self.profile_combo.setMinimumWidth(220)
        self.profile_combo.currentTextChanged.connect(self._profile_selected)
        row.addWidget(self.profile_combo)
        new_button = QPushButton("Save as new profile…")
        new_button.clicked.connect(self._save_as_new)
        row.addWidget(new_button)
        self.delete_button = QPushButton("Delete profile")
        self.delete_button.clicked.connect(self._delete_profile)
        row.addWidget(self.delete_button)
        row.addStretch(1)
        self._refresh_profiles()
        return row

    def _error_label(self, key: str) -> QLabel:
        widget = label("", "fieldError", size=12, wrap=True)
        widget.hide()
        self.errors[key] = widget
        return widget

    def _motor_card(self):
        frame, layout = card()
        layout.addWidget(section_title("Motor"))
        self.kv_radio = QRadioButton("I know the motor KV")
        self.kt_radio = QRadioButton("I know the torque constant (Kt)")
        group = QButtonGroup(self)
        group.addButton(self.kv_radio)
        group.addButton(self.kt_radio)
        grid = form_grid()
        grid.addWidget(self.kv_radio, 0, 0, 1, 2)
        self.kv = NumberField(1, 10000, 100, decimals=1, step=10, suffix="rpm/V")
        grid.addWidget(label("Motor KV"), 1, 0)
        grid.addWidget(self.kv, 1, 1)
        grid.addWidget(self._error_label("kv"), 2, 1, 1, 3)
        grid.addWidget(self.kt_radio, 3, 0, 1, 2)
        self.kt = NumberField(0.0001, 100, 0.0827, decimals=4, step=0.001, suffix="N·m/A")
        grid.addWidget(label("Torque constant"), 4, 0)
        grid.addWidget(self.kt, 4, 1)
        grid.addWidget(self._error_label("kt"), 5, 1, 1, 3)
        layout.addLayout(grid)
        self.kt_info = label("", "secondary", wrap=True)
        layout.addWidget(self.kt_info)
        layout.addWidget(label(f"KV is on the motor's datasheet (rpm per volt). Kt ≈ {KT_PER_KV} / KV.", "muted", wrap=True))
        for widget in (self.kv_radio, self.kt_radio):
            widget.toggled.connect(self._changed)
        self.kv.valueChanged.connect(self._changed)
        self.kt.valueChanged.connect(self._changed)
        return frame

    def _gearbox_card(self):
        frame, layout = card()
        layout.addWidget(section_title("Gearbox and ODrive"))
        grid = form_grid()
        self.gear = NumberField(0.01, 10000, 1.0, decimals=3, step=1, suffix=": 1")
        grid.addWidget(label("Gear ratio"), 0, 0)
        grid.addWidget(self.gear, 0, 1)
        grid.addWidget(label("motor turns per output turn (1 = direct drive)", "muted", wrap=True), 0, 2, 1, 2)
        grid.addWidget(self._error_label("gear_ratio"), 1, 1, 1, 3)
        self.efficiency = NumberField(1, 100, 100, decimals=1, step=1, suffix="%")
        grid.addWidget(label("Gearbox efficiency"), 2, 0)
        grid.addWidget(self.efficiency, 2, 1)
        grid.addWidget(label("leave 100 % if unknown", "muted"), 2, 2, 1, 2)
        grid.addWidget(self._error_label("efficiency_pct"), 3, 1, 1, 3)
        self.axis = QComboBox()
        self.axis.addItem("M0 (axis0)", 0)
        self.axis.addItem("M1 (axis1)", 1)
        grid.addWidget(label("Motor connected to"), 4, 0)
        grid.addWidget(self.axis, 4, 1)
        layout.addLayout(grid)
        layout.addWidget(label("The weight calibration (Measure step) can measure the real gearbox efficiency.", "muted", wrap=True))
        self.gear.valueChanged.connect(self._changed)
        self.efficiency.valueChanged.connect(self._changed)
        self.axis.currentIndexChanged.connect(self._changed)
        return frame

    def _limits_card(self):
        frame, layout = card()
        layout.addWidget(section_title("Safety limits"))
        grid = form_grid()
        self.max_current = NumberField(0.1, 120, 10, decimals=1, step=1, suffix="A")
        grid.addWidget(label("Max motor current"), 0, 0)
        grid.addWidget(self.max_current, 0, 1)
        self.current_info = label("", "muted")
        grid.addWidget(self.current_info, 0, 2, 1, 2)
        grid.addWidget(self._error_label("max_current_a"), 1, 1, 1, 3)
        self.max_speed = NumberField(0.1, 100000, 60, decimals=1, step=5, suffix="rpm")
        grid.addWidget(label("Max output speed"), 2, 0)
        grid.addWidget(self.max_speed, 2, 1)
        self.speed_info = label("", "muted")
        grid.addWidget(self.speed_info, 2, 2, 1, 2)
        grid.addWidget(self._error_label("max_speed_rpm"), 3, 1, 1, 3)
        layout.addLayout(grid)
        layout.addWidget(label("Every test stays inside these limits. The ODrive's own current limit still applies on top.",
                               "muted", wrap=True))
        self.max_current.valueChanged.connect(self._changed)
        self.max_speed.valueChanged.connect(self._changed)
        return frame

    def _display_card(self):
        frame, layout = card()
        layout.addWidget(section_title("Display"))
        grid = form_grid()
        self.unit = QComboBox()
        for unit in TORQUE_UNITS:
            self.unit.addItem(unit)
        grid.addWidget(label("Torque unit"), 0, 0)
        grid.addWidget(self.unit, 0, 1)
        self.sample_rate = NumberField(10, 500, 100, decimals=0, step=10, suffix="Hz")
        grid.addWidget(label("Sample rate"), 1, 0)
        grid.addWidget(self.sample_rate, 1, 1)
        grid.addWidget(label("actual rate is shown while measuring", "muted"), 1, 2, 1, 2)
        self.smoothing = NumberField(0, 1000, 30, decimals=0, step=10, suffix="ms")
        grid.addWidget(label("Readout smoothing"), 2, 0)
        grid.addWidget(self.smoothing, 2, 1)
        grid.addWidget(label("raw values are always recorded", "muted"), 2, 2, 1, 2)
        self.watchdog = QCheckBox("Use the ODrive watchdog while the motor is on (recommended)")
        grid.addWidget(self.watchdog, 3, 0, 1, 4)
        layout.addLayout(grid)
        self.unit.currentIndexChanged.connect(self._changed)
        return frame

    def _calibration_card(self):
        frame, layout = card()
        layout.addWidget(section_title("Calibration"))
        self.calibration_text = label("", "secondary", wrap=True)
        layout.addWidget(self.calibration_text)
        row = QHBoxLayout()
        self.use_calibration = QCheckBox("Use the calibration")
        self.use_calibration.toggled.connect(self._changed)
        row.addWidget(self.use_calibration)
        self.clear_calibration = QPushButton("Remove calibration")
        self.clear_calibration.clicked.connect(self._remove_calibration)
        row.addWidget(self.clear_calibration)
        row.addStretch(1)
        layout.addLayout(row)
        return frame

    # --- profile handling ------------------------------------------------------------------
    def _refresh_profiles(self) -> None:
        self._loading = True
        self.profile_combo.clear()
        for profile in self.settings.profiles:
            self.profile_combo.addItem(profile.name)
        self.profile_combo.setCurrentText(self.settings.current_profile)
        self._loading = False

    def _profile_selected(self, name: str) -> None:
        if self._loading or not name:
            return
        self.settings.current_profile = name
        self.load_profile(self.settings.profile(name))

    def load_profile(self, profile: ActuatorProfile) -> None:
        self._loading = True
        self._profile = copy.deepcopy(profile)
        (self.kv_radio if profile.kt_source == "kv" else self.kt_radio).setChecked(True)
        self.kv.setValue(profile.kv)
        self.kt.setValue(profile.kt)
        self.gear.setValue(profile.gear_ratio)
        self.efficiency.setValue(profile.efficiency_pct)
        self.axis.setCurrentIndex(profile.axis)
        self.max_current.setValue(profile.max_current_a)
        self.max_speed.setValue(profile.max_speed_rpm)
        self.unit.setCurrentText(profile.unit)
        self.use_calibration.setChecked(profile.use_calibration)
        self.sample_rate.setValue(self.settings.sample_rate_hz)
        self.smoothing.setValue(self.settings.display_filter_ms)
        self.watchdog.setChecked(self.settings.watchdog_enabled)
        self._loading = False
        self._changed()

    def current_profile(self) -> ActuatorProfile:
        p = copy.deepcopy(self._profile)
        p.name = self.profile_combo.currentText().strip() or p.name
        p.kt_source = "kv" if self.kv_radio.isChecked() else "kt"
        p.kv = self.kv.value()
        p.kt = self.kt.value()
        p.gear_ratio = self.gear.value()
        p.efficiency_pct = self.efficiency.value()
        p.axis = int(self.axis.currentData())
        p.max_current_a = self.max_current.value()
        p.max_speed_rpm = self.max_speed.value()
        p.unit = self.unit.currentText()
        p.use_calibration = self.use_calibration.isChecked()
        return p

    def _save_as_new(self) -> None:
        name, ok = QInputDialog.getText(self, "Save as new profile", "Profile name:")
        name = name.strip()
        if not ok or not name:
            return
        if any(p.name == name for p in self.settings.profiles):
            QMessageBox.warning(self, "Profile exists", f"A profile called “{name}” already exists.")
            return
        profile = self.current_profile()
        profile.name = name
        self.settings.upsert_profile(profile)
        self._profile = copy.deepcopy(profile)
        self._refresh_profiles()

    def _delete_profile(self) -> None:
        name = self.profile_combo.currentText()
        if len(self.settings.profiles) <= 1:
            QMessageBox.information(self, "Delete profile", "This is the only profile; change its values instead.")
            return
        if QMessageBox.question(self, "Delete profile", f"Delete the profile “{name}”?") != QMessageBox.Yes:
            return
        self.settings.delete_profile(name)
        self._refresh_profiles()
        self.load_profile(self.settings.profile())

    def _remove_calibration(self) -> None:
        if QMessageBox.question(self, "Remove calibration",
                                "Remove the weight calibration from this profile?") != QMessageBox.Yes:
            return
        self._profile.calibration_k_out = None
        self._profile.calibration_date = ""
        self._profile.calibration_details = {}
        self._changed()

    def set_calibration(self, k_out: float, date: str, details: dict) -> None:
        """Called by the calibration panel when a new calibration is saved."""
        self._profile.calibration_k_out = k_out
        self._profile.calibration_date = date
        self._profile.calibration_details = details
        self._profile.use_calibration = True
        self.use_calibration.setChecked(True)
        self._changed()

    # --- validation ------------------------------------------------------------------------
    def _changed(self, *_args) -> None:
        if self._loading:
            return
        profile = self.current_profile()
        self.kv.setEnabled(profile.kt_source == "kv")
        self.kt.setEnabled(profile.kt_source == "kt")
        problems = profile.validate()
        for key, widget in self.errors.items():
            message = problems.get(key, "")
            widget.setText(message)
            widget.setVisible(bool(message))
        for key, field in (("kv", self.kv), ("kt", self.kt), ("gear_ratio", self.gear),
                           ("efficiency_pct", self.efficiency), ("max_current_a", self.max_current),
                           ("max_speed_rpm", self.max_speed)):
            field.set_invalid(key in problems)
        self.next_button.setEnabled(not problems)
        if problems:
            self.summary.setText("Fix the highlighted values to continue.")
            return
        unit = profile.unit
        factor = TORQUE_UNITS[unit]
        model = profile.torque_model()
        kt = profile.kt_motor
        self.kt_info.setText(f"Kt used for torque: {kt:.5f} N·m/A" + (f"  (= {KT_PER_KV} / {profile.kv:g})" if profile.kt_source == "kv" else ""))
        self.current_info.setText(f"≈ {fmt(model.max_output_torque(profile.max_current_a) * factor)} {unit} at the output")
        self.speed_info.setText(f"motor {profile.limits().max_motor_speed:.3g} turns/s")
        if profile.has_calibration:
            self.calibration_text.setText(
                f"Calibrated {profile.calibration_date}: {profile.calibration_k_out:.4f} N·m per amp at the output "
                f"(nominal value {kt * profile.gear_ratio * profile.efficiency_pct / 100:.4f}).")
            self.use_calibration.setVisible(True)
            self.clear_calibration.setVisible(True)
        else:
            self.calibration_text.setText(
                "Not calibrated: torque = Kt × current × gear ratio × efficiency. For better accuracy, run the "
                "weight calibration in the Measure step.")
            self.use_calibration.setVisible(False)
            self.clear_calibration.setVisible(False)
        source = "calibration" if model.is_calibrated else "Kt × gear ratio × efficiency"
        self.summary.setText(f"Output torque = {model.k_out:.4f} N·m per amp ({source}).")

    def _finish(self) -> None:
        profile = self.current_profile()
        if profile.validate():
            return
        self.settings.sample_rate_hz = self.sample_rate.value()
        self.settings.display_filter_ms = self.smoothing.value()
        self.settings.watchdog_enabled = self.watchdog.isChecked()
        self.settings.upsert_profile(profile)
        self._profile = copy.deepcopy(profile)
        self._refresh_profiles()
        self.completed.emit(profile)
