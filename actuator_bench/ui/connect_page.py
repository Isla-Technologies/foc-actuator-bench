"""Step 2: connect to the controller (or the simulator) and check that it is ready."""

from __future__ import annotations

import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ..acquisition import ConnectionStatus, ConnectRequest, connection_modes
from ..backends.native_backend import odrive_library_version
from ..backends.simulator import SCENARIOS, SimActuator
from ..control import MotorState, state_label
from ..controllers import CONTROLLER_NAMES
from ..controllers.commander import BAUD_RATES
from ..device import Capabilities, Readiness
from ..settings import ActuatorProfile, AppSettings
from ..torque import KT_PER_KV, TORQUE_UNITS
from . import theme
from .widgets import StatusPill, card, fmt, form_grid, label, section_title

_ICONS = {"ok": ("✔", theme.SUCCESS_TEXT), "warn": ("▲", "#8a5d00"), "bad": ("✖", theme.CRITICAL),
          "info": ("ℹ", theme.INK_SECONDARY)}

_INTRO = {
    "odrive": "Switch on the ODrive's DC power supply (USB alone does not power it), plug in the USB cable, then "
              "press Connect. No driver is needed. Use the Simulator to try the app without hardware.",
    "simplefoc": "Switch on the board's power supply, connect its USB port (or the USB-serial adapter wired to its "
                 "TX, RX and GND), choose the port and the sketch's baud rate, then press Connect. Use the "
                 "Simulator to try the app without hardware.",
}


class CheckRow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(8)
        self.icon = QLabel()
        self.icon.setFont(theme.font(14, QFont.Bold))
        self.icon.setFixedWidth(18)
        self.icon.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        self.text = label("", wrap=True)
        layout.addWidget(self.icon, 0, Qt.AlignTop)
        layout.addWidget(self.text, 1)

    def set(self, kind: str, text: str) -> None:
        symbol, color = _ICONS[kind]
        self.icon.setText(symbol)
        self.icon.setStyleSheet(f"color: {color};")
        self.text.setText(text)


class ConnectPage(QWidget):
    back = Signal()
    proceed = Signal()
    connect_requested = Signal(object)        # ConnectRequest
    disconnect_requested = Signal()
    clear_errors_requested = Signal()
    calibrate_requested = Signal()
    apply_kt_requested = Signal(float)
    refresh_requested = Signal()

    def __init__(self, settings: AppSettings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.profile: ActuatorProfile = settings.profile()
        self.status: ConnectionStatus | None = None
        self.readiness: Readiness | None = None
        self.connected = False

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
        self.title = label("", "pageTitle", size=22, weight=QFont.DemiBold)
        root.addWidget(self.title)
        self.intro = label("", "secondary", wrap=True)
        root.addWidget(self.intro)

        columns = QHBoxLayout()
        columns.setSpacing(14)
        columns.addWidget(self._connection_card(), 1)
        columns.addWidget(self._checks_card(), 1)
        root.addLayout(columns)
        root.addStretch(1)

        footer = QHBoxLayout()
        back = QPushButton("←  Back to setup")
        back.clicked.connect(self.back.emit)
        footer.addWidget(back)
        footer.addStretch(1)
        self.proceed_button = QPushButton("Start measuring  →")
        self.proceed_button.setObjectName("primary")
        self.proceed_button.setMinimumHeight(36)
        self.proceed_button.clicked.connect(self.proceed.emit)
        footer.addWidget(self.proceed_button)
        root.addLayout(footer)
        self._show_controller()
        self.set_status(None)

    @property
    def controller(self) -> str:
        return self.profile.controller

    @property
    def controller_name(self) -> str:
        return CONTROLLER_NAMES.get(self.controller, "controller")

    # --- layout ----------------------------------------------------------------------------
    def _connection_card(self):
        frame, layout = card()
        layout.addWidget(section_title("Connection"))
        grid = form_grid()
        self.mode = QComboBox()
        self.mode.currentIndexChanged.connect(self._update_mode_widgets)
        grid.addWidget(label("Connect using"), 0, 0)
        grid.addWidget(self.mode, 0, 1, 1, 2)

        self.port_label = label("Serial port")
        self.port = QComboBox()
        self.port.setMinimumWidth(240)
        self.refresh_ports_button = QPushButton("Refresh")
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        grid.addWidget(self.port_label, 1, 0)
        grid.addWidget(self.port, 1, 1)
        grid.addWidget(self.refresh_ports_button, 1, 2)

        self.baud_label = label("Baud rate")
        self.baud = QComboBox()
        for rate in BAUD_RATES:
            self.baud.addItem(f"{rate}", rate)
        self.baud.setCurrentIndex(max(0, self.baud.findData(self.settings.serial_baud)))
        grid.addWidget(self.baud_label, 2, 0)
        grid.addWidget(self.baud, 2, 1)

        self.sim_label = label("Simulated setup")
        self.scenario = QComboBox()
        for key, text in SCENARIOS.items():
            self.scenario.addItem(text, key)
        self.scenario.setCurrentIndex(max(0, self.scenario.findData(self.settings.sim_scenario)))
        grid.addWidget(self.sim_label, 3, 0)
        grid.addWidget(self.scenario, 3, 1, 1, 2)
        layout.addLayout(grid)

        self.mode_hint = label("", "muted", wrap=True)
        layout.addWidget(self.mode_hint)

        row = QHBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.connect_button.setObjectName("primary")
        self.connect_button.setMinimumHeight(34)
        self.connect_button.clicked.connect(self._connect)
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.disconnect_requested.emit)
        row.addWidget(self.connect_button)
        row.addWidget(self.disconnect_button)
        row.addStretch(1)
        layout.addLayout(row)
        self.pill = StatusPill()
        layout.addWidget(self.pill, 0, Qt.AlignLeft)
        self.status_text = label("", "secondary", wrap=True)
        layout.addWidget(self.status_text)
        layout.addStretch(1)
        return frame

    def _checks_card(self):
        frame, layout = card()
        layout.addWidget(section_title("Checks"))
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(4)
        self.info_labels = {}
        for row, key in enumerate(("Board", "Firmware", "Serial number", "Connection")):
            grid.addWidget(label(key, "muted"), row, 0)
            value = label("—")
            self.info_labels[key] = value
            grid.addWidget(value, row, 1)
        grid.setColumnStretch(1, 1)
        layout.addLayout(grid)
        self.check_rows = [CheckRow() for _ in range(12)]
        for row in self.check_rows:
            layout.addWidget(row)
        buttons = QHBoxLayout()
        self.clear_button = QPushButton("Clear errors")
        self.clear_button.clicked.connect(self.clear_errors_requested.emit)
        self.calibrate_button = QPushButton("Run motor calibration…")
        self.calibrate_button.clicked.connect(self._calibrate)
        self.kt_button = QPushButton("Write my Kt to the controller…")
        self.kt_button.clicked.connect(self._apply_kt)
        self.refresh_button = QPushButton("Re-check")
        self.refresh_button.clicked.connect(self.refresh_requested.emit)
        for button in (self.clear_button, self.calibrate_button, self.kt_button, self.refresh_button):
            buttons.addWidget(button)
        buttons.addStretch(1)
        layout.addLayout(buttons)
        layout.addStretch(1)
        return frame

    # --- behaviour -------------------------------------------------------------------------
    def set_profile(self, profile: ActuatorProfile) -> None:
        changed = profile.controller != self.profile.controller
        self.profile = profile
        if changed or self.mode.count() == 0:
            self._show_controller()
        self._render_checks()

    def _show_controller(self) -> None:
        """Title, intro and connection modes for the profile's controller."""
        name = self.controller_name
        self.title.setText(f"Connect to the {name}")
        self.intro.setText(_INTRO.get(self.controller, ""))
        self.kt_button.setText(f"Write my Kt to the {name}…")
        wanted = self.mode.currentData() or self.settings.connection_mode
        self.mode.blockSignals(True)
        self.mode.clear()
        for key, text in connection_modes(self.controller).items():
            self.mode.addItem(text, key)
        self.mode.setCurrentIndex(max(0, self.mode.findData(wanted)))
        self.mode.blockSignals(False)
        self.refresh_ports()
        self._update_mode_widgets()

    def refresh_ports(self) -> None:
        current = self.port.currentData() or self.settings.serial_port
        self.port.clear()
        self.port.addItem(f"Auto-detect the {self.controller_name}", "")
        try:
            from ..backends.serial_backend import list_serial_ports

            for port in list_serial_ports():
                self.port.addItem(port.label, port.device)
        except Exception as exc:  # pyserial problems shouldn't break the page
            self.port.addItem(f"(could not list ports: {exc})", "")
        index = self.port.findData(current)
        self.port.setCurrentIndex(max(0, index))

    def _update_mode_widgets(self) -> None:
        mode = self.mode.currentData()
        simplefoc = self.controller == "simplefoc"
        serial_like = mode in ("auto", "serial")
        for widget in (self.port_label, self.port, self.refresh_ports_button):
            widget.setVisible(serial_like)
        for widget in (self.baud_label, self.baud):
            widget.setVisible(simplefoc and mode == "serial")
        for widget in (self.sim_label, self.scenario):
            widget.setVisible(mode == "simulator")
        if simplefoc:
            hints = {
                "serial": "Talks to the sketch's Commander over the serial port: pick the baud rate from its "
                          "Serial.begin(). Auto-detect tries the USB-serial adapters (CP210x, CH340, FTDI…). Close "
                          "the Arduino serial monitor first: only one program can use the port.",
                "simulator": "A virtual SimpleFOC board and actuator: try every feature safely without hardware.",
            }
        else:
            library = odrive_library_version()
            hints = {
                "auto": "Tries the USB serial port first, then the official ODrive library"
                        + ("" if library else " (not installed, optional)") + ".",
                "serial": "Talks to the ODrive's built-in USB serial port (ASCII protocol). Works with odrivetool "
                          "open at the same time.",
                "native": (f"Uses the official odrive library {library}. Close odrivetool first: only one program can "
                           "use this connection." if library else
                           "The official odrive library isn't installed. Install it with: pip install odrive"),
                "simulator": "A virtual ODrive and actuator: try every feature safely without hardware.",
            }
        self.mode_hint.setText(hints.get(mode, ""))

    def connect_request(self) -> ConnectRequest:
        mode = self.mode.currentData()
        kv = self.profile.kv if self.profile.kt_source == "kv" else KT_PER_KV / self.profile.kt
        actuator = SimActuator(kv=kv, gear_ratio=self.profile.gear_ratio, scenario=self.scenario.currentData(),
                               mass_kg=self.settings.sim_mass_kg)
        if self.profile.pole_pairs:
            actuator.pole_pairs = self.profile.pole_pairs
        return ConnectRequest(mode=mode, port=self.port.currentData() or None, axis=self.profile.axis,
                              controller=self.controller, baud=int(self.baud.currentData()),
                              motor_letter=self.profile.motor_letter or "M", sim_actuator=actuator)

    def _connect(self) -> None:
        self.settings.connection_mode = self.mode.currentData()
        self.settings.serial_port = self.port.currentData() or ""
        self.settings.serial_baud = int(self.baud.currentData())
        self.settings.sim_scenario = self.scenario.currentData()
        self.connect_requested.emit(self.connect_request())

    def _calibrate(self) -> None:
        if self.controller == "simplefoc":
            text = ("The board re-runs its sensor alignment: the motor turns slowly one way and back.\n\n"
                    "The output must be free to move (no arm against an object, no load). Continue?")
        else:
            text = ("The motor will beep and then turn slowly in both directions for a few seconds.\n\n"
                    "The output must be free to move (no arm against an object, no load). Continue?")
        answer = QMessageBox.warning(self, "Run motor calibration", text,
                                     QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)
        if answer == QMessageBox.Yes:
            self.calibrate_requested.emit()

    def _apply_kt(self) -> None:
        kt = self.profile.kt_motor
        name = self.controller_name
        answer = QMessageBox.question(
            self, f"Write Kt to the {name}",
            f"Set the {name}'s torque constant to {kt:.5f} N·m/A?\n\n"
            f"This app already corrects for a mismatch, so this is optional. It lasts until the {name} restarts. "
            f"Note: the {name}'s control gains are in N·m, so changing the torque constant also changes how "
            "strongly it reacts.", QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)
        if answer == QMessageBox.Yes:
            self.apply_kt_requested.emit(kt)

    # --- status ----------------------------------------------------------------------------
    def set_status(self, status: ConnectionStatus | None) -> None:
        self.status = status
        state = status.state if status else "disconnected"
        self.connected = state == "connected"
        if status and status.readiness:
            self.readiness = status.readiness
        if state == "connecting":
            self.pill.set("info", "Connecting…")
        elif self.connected:
            self.pill.set("good", "Connected")
        elif state == "failed":
            self.pill.set("critical", "Connection failed")
        else:
            self.pill.set("neutral", "Not connected")
        text = status.message if status else ""
        if state == "failed":
            if self.controller == "simplefoc":
                text += ("\n\nChecklist: board powered? Right port and baud rate? Arduino serial monitor closed? "
                         "TX and RX crossed over (adapter TX to board RX)? Does the sketch call command.run() in "
                         "loop()?")
            else:
                text += ("\n\nChecklist: DC power on? Data-capable USB cable? Try another USB port. "
                         "For the official library connection, close odrivetool first.")
        self.status_text.setText(text)
        self.status_text.setObjectName("error" if state == "failed" else "secondary")
        self.status_text.style().unpolish(self.status_text)
        self.status_text.style().polish(self.status_text)
        self.connect_button.setEnabled(state != "connecting")
        self.connect_button.setText("Reconnect" if self.connected else "Connect")
        self.disconnect_button.setEnabled(self.connected)
        self.mode.setEnabled(not self.connected and state != "connecting")
        self.proceed_button.setEnabled(self.connected)
        if not self.connected:
            self.readiness = None
        info = status.info if status and self.connected else None
        self.info_labels["Board"].setText(info.board if info else "—")
        self.info_labels["Firmware"].setText(info.firmware if info else "—")
        self.info_labels["Serial number"].setText(info.serial if info else "—")
        self.info_labels["Connection"].setText(info.connection if info else "—")
        self._render_checks()

    def set_readiness(self, readiness: Readiness) -> None:
        self.readiness = readiness
        self._render_checks()

    def _render_checks(self) -> None:
        rows = self.check_rows
        for row in rows:
            row.hide()
        for button in (self.clear_button, self.calibrate_button, self.kt_button, self.refresh_button):
            button.setEnabled(self.connected)
        if not self.connected or self.readiness is None or self.status is None:
            # Until connected, show what this kind of controller may offer.
            self.clear_button.setVisible(True)
            self.calibrate_button.setVisible(True)
            self.kt_button.setVisible(self.controller == "odrive")
            return
        r, info, p = self.readiness, self.status.info, self.profile
        caps: Capabilities = info.capabilities
        name = info.controller
        checks: list[tuple[str, str]] = []
        if info.family != p.controller:
            checks.append(("bad", f"Setup says the motor is on a {CONTROLLER_NAMES.get(p.controller, p.controller)}, "
                                  f"but this is a {name}. Go back to Setup and choose the right controller."))
        if info.family == "odrive":
            if info.firmware_supported:
                checks.append(("ok", f"Firmware {info.firmware} is supported."))
            else:
                checks.append(("warn", f"Firmware {info.firmware}: this app is made for ODrive v3.6 firmware 0.5.x."))
        state_text = state_label(r.state)
        if r.motor_calibrated is None or r.encoder_ready is None:
            if info.family == "odrive":
                checks.append(("warn", f"Couldn't read the calibration flags over this connection. State: {state_text}. "
                                       "If switching the motor on fails, run the motor calibration."))
            else:
                checks.append(("info", f"State: {state_text}."))
        elif r.motor_calibrated and r.encoder_ready:
            checks.append(("ok", f"Motor calibrated and encoder ready. State: {state_text}."))
        elif info.family == "odrive":
            missing = [what for what, ok in (("motor not calibrated", r.motor_calibrated),
                                             ("encoder not ready", r.encoder_ready)) if not ok]
            checks.append(("bad", f"{', '.join(missing).capitalize()}. Run the motor calibration (output free to move), "
                                  "or set it up in odrivetool."))
        checks.extend(r.checks)
        if r.errors:
            lines = "".join(f"<br>• <b>{e.name}</b>: {e.explanation}" for e in r.errors)
            checks.append(("bad", f"The {name} reports errors:{lines}"))
        else:
            checks.append(("ok", "No errors."))
        kt_mine = p.kt_motor
        if r.kt_device and r.kt_device > 0:
            ratio = r.kt_device / kt_mine
            if abs(ratio - 1) <= 0.05:
                checks.append(("ok", f"{name} torque constant {r.kt_device:.4f} N·m/A matches your values."))
            else:
                hint = " (that is the 0.04 factory default)" if abs(r.kt_device - 0.04) < 1e-6 else ""
                checks.append(("warn", f"{name} torque constant is {r.kt_device:.4f} N·m/A{hint}, yours is "
                                       f"{kt_mine:.4f}. The app uses yours for torque readings and corrects its commands, "
                                       f"so this is fine; you can also write yours to the {name}."))
        if r.current_lim:
            if p.max_current_a > r.current_lim + 1e-6:
                checks.append(("warn", f"Your current limit ({p.max_current_a:g} A) is above the {name}'s own "
                                       f"({r.current_lim:g} A); the {name} will cap the current."))
            else:
                checks.append(("info", f"{name} current limit {r.current_lim:g} A; tests use at most {p.max_current_a:g} A."))
        if r.speed_limiter_gain:
            model = p.torque_model()
            cap = model.output_torque(p.limits().max_motor_speed * r.speed_limiter_gain)
            unit = p.unit
            if cap < model.max_output_torque(p.max_current_a) * 0.95:
                checks.append(("info", f"In manual torque mode the {name} limits standstill torque to about "
                                       f"{fmt(cap * TORQUE_UNITS[unit])} {unit} (speed limit × vel_gain). "
                                       "The Stall test raises this automatically."))
        if r.vbus and math.isfinite(r.vbus):
            kind = "ok" if r.vbus > 10 else "warn"
            checks.append((kind, f"Bus voltage {r.vbus:.1f} V."))
        for row, (kind, text) in zip(rows, checks):
            row.set(kind, text)
            row.show()
        self.clear_button.setVisible(caps.clear_errors)
        self.calibrate_button.setVisible(caps.motor_calibration)
        self.kt_button.setVisible(caps.write_torque_constant)
        self.calibrate_button.setEnabled(r.state != MotorState.CLOSED_LOOP)
