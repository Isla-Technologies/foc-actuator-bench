"""The Simulation model tab: the tests that measure what a robot simulator needs, and the export.

Each test tab says which simulator value it sets, how to set up the actuator, and stops by itself
when the setup isn't right. The Overview lists every value with where it came from and exports the
model. Values here are in SI units, as simulators use them.
"""

from __future__ import annotations

import json
import math

import numpy as np
from PySide6.QtCore import Signal
from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..backends.simulator import SCENARIOS
from ..model_export import CALCULATED, MEASURED, MISSING, SETUP, TESTS, model_values, simulation_model
from ..model_tests import (
    BacklashTest,
    BacklashTestParams,
    FrictionTest,
    FrictionTestParams,
    InertiaTest,
    InertiaTestParams,
    SpeedTest,
    SpeedTestParams,
    ThermalTest,
    ThermalTestParams,
)
from ..odrive_api import MAX_PHASE_VOLTAGE_PER_BUS_VOLT
from ..recorder import timestamp
from ..settings import ActuatorProfile, AppSettings
from .result_chart import ResultChart
from .tests_panel import _TestPanel
from .widgets import NumberField, StatTile, form_grid, label, scrollable, section_title

RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)


def sig(value, digits: int = 3) -> str:
    """``digits`` significant figures, or a dash."""
    if value is None or not math.isfinite(value):
        return "—"
    if abs(value) >= 10 ** digits:
        return f"{value:.0f}"
    return f"{value:.{digits}g}"


class _ModelTestPanel(_TestPanel):
    test_key = ""
    sim_scenarios: tuple[str, ...] = ("free",)
    sim_hint = ""

    def __init__(self, worker, intro: str, setup: str, measured: str, fitted: str, parent=None):
        super().__init__(worker, parent)
        self.is_simulator = False
        self.scenario = ""
        self._chart_profile = ""
        root = self.root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)
        root.addWidget(label(intro, "secondary", wrap=True))
        root.addWidget(label(setup, "secondary", wrap=True))
        self.sim_note = label("", "muted", wrap=True)
        root.addWidget(self.sim_note)
        self.grid = form_grid()
        root.addLayout(self.grid)
        self.chart = ResultChart(measured, fitted)

    def _finish_layout(self, start_text: str, tiles: list[StatTile]) -> None:
        self._build_common(self.root, start_text)
        results = QGridLayout()
        results.setHorizontalSpacing(24)
        self.tiles = tiles
        for col, tile in enumerate(tiles):
            results.addWidget(tile, 0, col)
        self.root.addLayout(results)
        self.root.addWidget(self.chart, 1)
        self._update_enabled()

    def field_row(self, row: int, text: str, widget: QWidget, hint: str = "") -> None:
        self.grid.addWidget(label(text), row, 0)
        self.grid.addWidget(widget, row, 1)
        if hint:
            self.grid.addWidget(label(hint, "muted", wrap=True), row, 2, 1, 2)

    def set_simulator(self, is_simulator: bool, scenario: str) -> None:
        self.is_simulator, self.scenario = is_simulator, scenario
        if is_simulator and scenario not in self.sim_scenarios:
            self.sim_note.setText(f"Simulator: choose “{SCENARIOS[self.sim_scenarios[0]]}” on the Connect page "
                                  "for this test.")
        else:
            self.sim_note.setText(self.simulator_hint() if is_simulator else "")

    def simulator_hint(self) -> str:
        return self.sim_hint

    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        changed = self.profile is None or self.profile.name != profile.name
        super().set_profile(profile, kt_odrive)
        if changed:
            self.defaults(profile)
        if self._chart_profile and self._chart_profile != profile.name:
            self.chart.show_empty("Run the test to see its chart.")
            self._chart_profile = ""
        if not self.running:
            self.show_values(profile.model_results.get(self.test_key) or {})

    def defaults(self, profile: ActuatorProfile) -> None:
        """Starting values that suit this actuator's limits."""

    def output_speed_limit(self) -> float:
        """The Setup speed limit, in output rad/s."""
        return self.profile.max_speed_rpm / RPM_PER_RAD_S if self.profile else 1.0

    def _launch(self, procedure) -> None:
        for tile in self.tiles:
            tile.set("—")
        self.running = True
        self._update_enabled()
        self.worker.start_procedure(procedure)

    def show_result(self, result: dict) -> None:
        self.show_values(result)
        if result.get("series"):
            self.show_chart(result)
            self._chart_profile = self.profile.name if self.profile else ""

    def show_values(self, result: dict) -> None:
        raise NotImplementedError

    def show_chart(self, result: dict) -> None:
        raise NotImplementedError


# --- Friction ----------------------------------------------------------------------------------

class FrictionPanel(_ModelTestPanel):
    procedure_name = FrictionTest.name
    test_key = "friction"
    sim_scenarios = ("free", "lever")
    sim_hint = "Simulator: the virtual actuator has friction you can measure."

    def __init__(self, worker, parent=None):
        super().__init__(worker, (
            "<b>Friction</b> sets the simulator's <code>frictionloss</code> (N·m, felt at any speed) and "
            "<code>damping</code> (N·m per rad/s, grows with speed). The output turns at several steady speeds, "
            "each forward and then back over the same angle. Half the difference between the two torques is "
            "friction; gravity on the output and current-sensor offsets cancel out."), (
            "<b>Set up:</b> nothing on the output, or a balanced arm. It must turn freely through the travel "
            "below, and the motor switches on by itself, so keep hands clear. The test stops if the output goes "
            "30° past the travel."), "Measured friction", "Coulomb + viscous fit", parent)
        self.travel = NumberField(10, 3600, 180, decimals=0, step=10, suffix="°")
        self.top_speed = NumberField(0.1, 100000, 30, decimals=1, step=5, suffix="rpm")
        self.levels = NumberField(3, 10, 6, decimals=0, step=1, suffix="speeds")
        self.field_row(0, "Travel per pass", self.travel, "more travel lets faster passes settle")
        self.field_row(1, "Fastest speed", self.top_speed, "at the output; the slowest is 1/12 of it")
        self.field_row(2, "Speeds", self.levels)
        self._finish_layout("Start friction test", [
            StatTile("Friction (frictionloss)", "N·m"), StatTile("Damping", "N·m·s/rad"),
            StatTile("Fit R²"), StatTile("Speeds used")])

    def defaults(self, profile: ActuatorProfile) -> None:
        self.top_speed.setMaximum(profile.max_speed_rpm)
        self.top_speed.setValue(0.6 * profile.max_speed_rpm)

    def _start(self) -> None:
        self._launch(FrictionTest(FrictionTestParams(
            travel_deg=self.travel.value(), top_speed=self.top_speed.value() / RPM_PER_RAD_S,
            levels=int(self.levels.value()))))

    def show_values(self, r: dict) -> None:
        friction, damping, fit, used = self.tiles
        friction.set(sig(r.get("coulomb_Nm")))
        damping.set(sig(r.get("viscous_Nms_per_rad")))
        fit.set(sig(r.get("r_squared"), 4))
        levels = r.get("levels") or []
        used.set(f"{len(levels)} of {len(levels) + len(r.get('skipped_speeds_rad_s') or [])}" if levels else "—")

    def show_chart(self, r: dict) -> None:
        levels = r.get("levels") or []
        if not levels:
            self.chart.show_empty("No speed gave steady readings.")
            return
        speed = [level["speed"] for level in levels]
        friction = [level["friction"] for level in levels]
        fit_x = fit_y = None
        if "coulomb_Nm" in r:
            fit_x = np.linspace(0.0, max(speed) * 1.05, 50)
            fit_y = r["coulomb_Nm"] + r["viscous_Nms_per_rad"] * fit_x
        self.chart.show(speed, friction, x_label="Output speed (rad/s)", y_label="Friction torque (N·m)",
                        x_unit="rad/s", y_unit="N·m", fit_x=fit_x, fit_y=fit_y, as_points=True)


# --- Inertia -----------------------------------------------------------------------------------

class InertiaPanel(_ModelTestPanel):
    procedure_name = InertiaTest.name
    test_key = "inertia"
    sim_hint = "Simulator: the virtual rotor and gearbox add a known inertia to find."

    def __init__(self, worker, parent=None):
        super().__init__(worker, (
            "<b>Inertia</b> sets the simulator's <code>armature</code>: the inertia the motor's rotor and the "
            "gearbox add to the joint, seen at the output (the rotor's own inertia times the gear ratio squared). "
            "The app applies a sinusoidal torque at three frequencies and adjusts it until the output swings "
            "about as far as you ask. The torque that goes into speeding up and slowing down gives the inertia; "
            "friction and gravity drop out."), (
            "<b>Set up:</b> nothing on the output. Anything attached adds its own inertia: enter it below to "
            "subtract it. The output must swing freely by the angle below, both ways. Keep hands clear: the motor "
            "switches on by itself. The test stops if the output swings twice as far as asked."),
            "Each frequency", "Fit (inertia, less any spring)", parent)
        self.swing = NumberField(2, 90, 20, decimals=0, step=5, suffix="°")
        self.peak_speed = NumberField(0.1, 100000, 20, decimals=1, step=5, suffix="rpm")
        self.cycles = NumberField(2, 20, 4, decimals=0, step=1, suffix="cycles")
        self.attached = NumberField(0, 100, 0, decimals=5, step=0.001, suffix="kg·m²")
        self.field_row(0, "Swing ±", self.swing, "at the output, at the slowest frequency")
        self.field_row(1, "Speed mid-swing", self.peak_speed, "at the output")
        self.field_row(2, "Measured cycles", self.cycles, "per frequency")
        self.field_row(3, "Attached inertia", self.attached, "anything bolted to the output (0 if bare)")
        self._finish_layout("Start inertia test", [
            StatTile("Inertia at the output (armature)", "kg·m²"), StatTile("At the motor", "kg·m²"),
            StatTile("Spread between frequencies", "%"), StatTile("Spring-like load", "N·m/rad")])

    def defaults(self, profile: ActuatorProfile) -> None:
        self.peak_speed.setMaximum(0.8 * profile.max_speed_rpm)
        self.peak_speed.setValue(0.4 * profile.max_speed_rpm)

    def _start(self) -> None:
        self._launch(InertiaTest(InertiaTestParams(
            swing_deg=self.swing.value(), peak_speed=self.peak_speed.value() / RPM_PER_RAD_S,
            cycles=int(self.cycles.value()), attached_inertia=self.attached.value())))

    def show_values(self, r: dict) -> None:
        output, motor, spread, spring = self.tiles
        inertia = r.get("inertia_kgm2")
        output.set(sig(inertia))
        gear = self.profile.gear_ratio if self.profile else 1.0
        motor.set(sig(inertia / gear ** 2) if inertia is not None else "—")
        spread.set(sig(100.0 * r["spread"], 2) if "spread" in r else "—")
        spring.set(sig(r.get("spring_Nm_per_rad")))

    def show_result(self, result: dict) -> None:
        super().show_result(result)
        if result.get("notes"):
            self.message.setText(self.message.text() + "\n" + " ".join(result["notes"]))

    def show_chart(self, r: dict) -> None:
        swings = r.get("swings") or []
        if not swings:
            self.chart.show_empty("No usable swing.")
            return
        attached = r.get("attached_inertia_kgm2", 0.0)
        frequency = [s["frequency"] for s in swings]
        apparent = [s["apparent_inertia"] - attached for s in swings]
        fit_x = np.linspace(min(frequency) * 0.9, max(frequency) * 1.1, 50)
        fit_y = r["inertia_kgm2"] - r["spring_Nm_per_rad"] / (2 * math.pi * fit_x) ** 2
        self.chart.show(frequency, apparent, x_label="Frequency (Hz)", y_label="Inertia (kg·m²)", x_unit="Hz",
                        y_unit="kg·m²", fit_x=fit_x, fit_y=fit_y, as_points=True)


# --- Speed -------------------------------------------------------------------------------------

class SpeedPanel(_ModelTestPanel):
    procedure_name = SpeedTest.name
    test_key = "speed"

    def simulator_hint(self) -> str:
        if not self.profile:
            return ""
        # The simulated motor: Kt 5 % above the profile's, on a 24 V supply (see backends/simulator.py).
        flux = self.profile.kt_motor * 1.05 / 1.5
        top = MAX_PHASE_VOLTAGE_PER_BUS_VOLT * 24.0 / flux / self.profile.gear_ratio * RPM_PER_RAD_S
        return (f"Simulator: the virtual motor tops out near {top:.0f} rpm at the output on its 24 V supply, so "
                f"Max output speed in Setup must be above that (about {1.5 * top:.0f} rpm).")

    def __init__(self, worker, parent=None):
        super().__init__(worker, (
            "<b>No-load speed</b> sets the simulator's <code>no_load_speed</code> and <code>corner_speed</code>: "
            "how fast the joint can turn, and up to what speed it still gives full torque. The speed rises until "
            "the motor runs out of voltage. The speed it tops out at gives the motor's back-EMF; with the "
            "resistance and inductance the ODrive measured in its motor calibration, the app works out both "
            "speeds at your robot's battery voltage (set on the Overview tab)."), (
            "<b>Set up:</b> nothing on the output, which must spin freely for many turns in both directions. "
            "Max output speed in Setup must be above the no-load speed, about 1.5 times what you expect: the test "
            "can't go faster. If you can, use the same supply voltage as the robot."),
            "Output speed", "Commanded speed", parent)
        self.ramp = NumberField(2, 60, 6, decimals=0, step=1, suffix="s")
        self.both = QCheckBox("Both directions (recommended)")
        self.both.setChecked(True)
        self.limit_info = label("", "muted", wrap=True)
        self.field_row(0, "Ramp to the speed limit in", self.ramp)
        self.grid.addWidget(self.both, 1, 1, 1, 3)
        self.grid.addWidget(self.limit_info, 2, 0, 1, 4)
        self._finish_layout("Start speed test", [
            StatTile("No-load speed", "rpm"), StatTile("Supply during the test", "V"),
            StatTile("No-load current", "A"), StatTile("Motor Kt from back-EMF", "N·m/A")])

    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        super().set_profile(profile, kt_odrive)
        self.limit_info.setText(f"Speed limit now: {profile.max_speed_rpm:g} rpm at the output "
                                f"({profile.max_speed_rpm / RPM_PER_RAD_S:.3g} rad/s).")
        self.set_simulator(self.is_simulator, self.scenario)

    def _start(self) -> None:
        self._launch(SpeedTest(SpeedTestParams(ramp_s=self.ramp.value(), both_directions=self.both.isChecked())))

    def show_values(self, r: dict) -> None:
        speed, volts, current, kt = self.tiles
        speed.set(sig(r.get("no_load_speed_rpm")))
        volts.set(sig(r.get("vbus_V")))
        current.set(sig(r.get("no_load_current_A")))
        kt.set(sig((r.get("motor") or {}).get("kt_backemf_Nm_per_A"), 4))

    def show_chart(self, r: dict) -> None:
        s = r["series"]
        self.chart.show(s["t"], s["speed_rpm"], x_label="Time (s)", y_label="Output speed (rpm)", x_unit="s",
                        y_unit="rpm", fit_x=s["t"], fit_y=s["command_rpm"])


# --- Backlash ----------------------------------------------------------------------------------

class BacklashPanel(_ModelTestPanel):
    procedure_name = BacklashTest.name
    test_key = "backlash"
    sim_scenarios = ("locked",)
    sim_hint = "Simulator: the virtual gearbox has 0.2° of play and 1500 N·m/rad of stiffness."

    def __init__(self, worker, parent=None):
        super().__init__(worker, (
            "<b>Backlash and gearbox stiffness</b>: how far the output can move with no torque (play), and how "
            "much the gearbox twists under load. The motor winds the torque slowly up and down, both ways, while "
            "the output is held. The motor's encoder sees the play and the twist through the gearbox. (Isla "
            "Studio doesn't model these yet; they're exported for when it does.)"), (
            "<b>Set up:</b> clamp the output rigidly in both directions, as close to the output as you can: bolt "
            "the output flange to the bench, or clamp the arm between two stops with no gap. Whatever bends in the "
            "clamp counts as gearbox twist. The test stops if the output moves more than the angle below."),
            "Measured loop", "Gearbox under load", parent)
        self.torque = NumberField(0.01, 10000, 1, decimals=2, step=1, suffix="N·m")
        self.cycle = NumberField(2, 60, 8, decimals=0, step=1, suffix="s")
        self.movement = NumberField(0.5, 30, 5, decimals=1, step=0.5, suffix="°")
        self.max_info = label("", "muted")
        self.field_row(0, "Torque ±", self.torque)
        self.grid.addWidget(self.max_info, 0, 2, 1, 2)
        self.field_row(1, "One full cycle takes", self.cycle, "slower is gentler; two cycles are run")
        self.field_row(2, "Stop if the output moves more than", self.movement)
        self._finish_layout("Start backlash test", [
            StatTile("Backlash", "°"), StatTile("Stiffness", "N·m/rad"),
            StatTile("Loop width at zero torque", "°"), StatTile("Torque reached", "N·m")])

    def _limit(self) -> float:
        if not self.profile:
            return 1.0
        return self.profile.torque_model(self.kt_odrive).max_output_torque(self.profile.max_current_a)

    def defaults(self, profile: ActuatorProfile) -> None:
        self.torque.setValue(0.3 * profile.torque_model().max_output_torque(profile.max_current_a))

    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        super().set_profile(profile, kt_odrive)
        self.torque.setMaximum(self._limit())
        self.max_info.setText(f"max {sig(self._limit())} N·m with your {profile.max_current_a:g} A limit")

    def _start(self) -> None:
        self._launch(BacklashTest(BacklashTestParams(
            max_torque_out=self.torque.value(), cycle_s=self.cycle.value(), max_movement_deg=self.movement.value())))

    def show_values(self, r: dict) -> None:
        play, stiffness, width, reached = self.tiles
        play.set(sig(r.get("backlash_deg")))
        stiffness.set(sig(r.get("stiffness_Nm_per_rad")))
        width.set(sig(r.get("hysteresis_deg")))
        reached.set(sig(r.get("max_torque_Nm")))

    def show_chart(self, r: dict) -> None:
        s = r["series"]
        fit_x = fit_y = None
        if "stiffness_forward_Nm_per_rad" in r:
            top, center, half = r["max_torque_Nm"], r["center_deg"], r["backlash_deg"] / 2
            forward = np.linspace(0.3 * top, top, 20)
            backward = -forward[::-1]
            fit_x = np.concatenate([backward, [np.nan], forward])
            fit_y = np.concatenate([
                center - half + np.degrees(backward / r["stiffness_backward_Nm_per_rad"]), [np.nan],
                center + half + np.degrees(forward / r["stiffness_forward_Nm_per_rad"])])
        self.chart.show(s["torque_Nm"], s["angle_deg"], x_label="Output torque (N·m)", y_label="Output angle (°)",
                        x_unit="N·m", y_unit="°", fit_x=fit_x, fit_y=fit_y)


# --- Thermal -----------------------------------------------------------------------------------

class ThermalPanel(_ModelTestPanel):
    procedure_name = ThermalTest.name
    test_key = "thermal"
    sim_scenarios = ("wall", "locked")
    sim_hint = "Simulator: the virtual motor settles 0.2 °C per A² above 25 °C, with a 5-minute time constant."

    def __init__(self, worker, parent=None):
        super().__init__(worker, (
            "<b>Continuous current</b> sets the simulator's <code>continuous_current</code>, and how long the peak "
            "current can last. The motor holds a steady current against a blocked output while the app logs its "
            "temperature. A heating curve fitted to it gives the current at which the motor would settle exactly "
            "at your temperature limit."), (
            "<b>Set up:</b> as for the stall test: press the arm against a fixed object, or clamp the output. "
            "Start with a cold motor. Use the motor's temperature sensor (wire it and enable "
            "<code>motor_thermistor</code> in the ODrive): without it the app can only read the ODrive's own "
            "temperature, which describes the driver, not the motor. It takes 10 to 20 minutes. You can stop early "
            "and still get a result, but the longer it runs, the firmer the result."),
            "Temperature", "Heating curve", parent)
        self.current = NumberField(0.1, 120, 5, decimals=1, step=1, suffix="A")
        self.limit = NumberField(30, 150, 80, decimals=0, step=5, suffix="°C")
        self.duration = NumberField(1, 120, 15, decimals=0, step=5, suffix="min")
        self.direction = QComboBox()
        self.direction.addItem("Positive (+)", 1)
        self.direction.addItem("Negative (−)", -1)
        self.field_row(0, "Test current", self.current, "close to the current you expect to hold")
        self.field_row(1, "Stop at", self.limit, "the motor's temperature limit")
        self.field_row(2, "Run for at most", self.duration)
        self.field_row(3, "Direction", self.direction)
        self._finish_layout("Start thermal test", [
            StatTile("Continuous current", "A"), StatTile("Continuous torque", "N·m"),
            StatTile("Heating time constant", "min"), StatTile("Would settle at", "°C")])

    def defaults(self, profile: ActuatorProfile) -> None:
        self.current.setMaximum(profile.max_current_a)
        self.current.setValue(0.7 * profile.max_current_a)
        self.limit.setValue(profile.temp_limit_c)

    def _start(self) -> None:
        self._launch(ThermalTest(ThermalTestParams(
            current_a=self.current.value(), temp_limit_c=self.limit.value(),
            max_duration_s=self.duration.value() * 60.0, direction=int(self.direction.currentData()))))

    def show_values(self, r: dict) -> None:
        current, torque, tau, settle = self.tiles
        amps = r.get("continuous_current_A")
        current.set(sig(amps))
        k_out = self.profile.torque_model(self.kt_odrive).k_out if self.profile else math.nan
        torque.set(sig(amps * k_out) if amps is not None else "—")
        tau.set(sig(r["time_constant_s"] / 60.0) if "time_constant_s" in r else "—")
        settle.set(sig(r["start_temp_C"] + r["rise_C"]) if "rise_C" in r else "—")
        current.title.setText("Continuous current" + (" (driver, no motor sensor)" if r.get("sensor") == "driver" else ""))

    def show_chart(self, r: dict) -> None:
        s = r["series"]
        minutes = np.array(s["t"]) / 60.0
        fit_x = fit_y = None
        if "rise_C" in r and len(minutes):
            span = max(minutes[-1] * 1.3, 1.0)
            fit_x = np.linspace(minutes[0], span, 80)
            fit_y = r["start_temp_C"] + r["rise_C"] * (1 - np.exp(-(fit_x - minutes[0]) * 60.0 / r["time_constant_s"]))
        self.chart.show(minutes, s["temperature_C"], x_label="Time (min)", y_label="Temperature (°C)", x_unit="min",
                        y_unit="°C", fit_x=fit_x, fit_y=fit_y)


# --- Gear ratio --------------------------------------------------------------------------------

class GearRatioPanel(QWidget):
    """Count motor turns while the output is turned by hand. No motor power, so not a procedure."""

    measured = Signal(dict)
    test_key = "gear_ratio"

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.profile: ActuatorProfile | None = None
        self.connected = self.armed = self.busy = self.is_simulator = False
        self.latest_pos: float | None = None
        self.start_pos: float | None = None
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)
        root.addWidget(label(
            "<b>Gear ratio</b> sets the simulator's <code>gear_ratio</code>, and every value at the output depends "
            "on it. Turn the output by hand a whole number of turns; the app counts the motor's turns.",
            "secondary", wrap=True))
        root.addWidget(label(
            "<b>Set up:</b> motor off. Put a mark on the output and one on the housing beside it. Press Start, turn "
            "the output exactly the number of turns below until the marks line up again (more turns, more "
            "precise), then press Done. If the gearbox can't be turned by hand, use Manual control instead: "
            "Position mode, angle target +360°, and check that the marks line up after one turn.",
            "secondary", wrap=True))
        self.sim_note = label("", "muted", wrap=True)
        root.addWidget(self.sim_note)
        grid = form_grid()
        self.turns = NumberField(1, 100, 1, decimals=0, step=1, suffix="turns")
        grid.addWidget(label("Output turns"), 0, 0)
        grid.addWidget(self.turns, 0, 1)
        root.addLayout(grid)
        row = QHBoxLayout()
        self.start_button = QPushButton("Start counting")
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self._start)
        self.done_button = QPushButton("Done: work out the ratio")
        self.done_button.clicked.connect(self._done)
        self.hand_button = QPushButton("Turn the output one turn (simulated hand)")
        self.hand_button.clicked.connect(lambda: self.worker.turn_output_by_hand(1.0))
        for button in (self.start_button, self.done_button, self.hand_button):
            row.addWidget(button)
        row.addStretch(1)
        root.addLayout(row)
        self.live = label("", "secondary", wrap=True)
        root.addWidget(self.live)
        results = QGridLayout()
        results.setHorizontalSpacing(24)
        self.r_ratio = StatTile("Measured ratio", ": 1")
        self.r_setup = StatTile("In Setup", ": 1")
        self.r_diff = StatTile("Difference", "%")
        for col, tile in enumerate((self.r_ratio, self.r_setup, self.r_diff)):
            results.addWidget(tile, 0, col)
        root.addLayout(results)
        root.addStretch(1)
        self._update()

    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        self.profile = profile
        self.r_setup.set(sig(profile.gear_ratio, 6))
        stored = profile.model_results.get(self.test_key) or {}
        self._show(stored.get("ratio"))

    def set_connected(self, connected: bool) -> None:
        self.connected = connected
        if not connected:
            self.start_pos = self.latest_pos = None
        self._update()

    def set_simulator(self, is_simulator: bool, scenario: str) -> None:
        self.is_simulator = is_simulator and scenario in ("free", "lever")
        if is_simulator and not self.is_simulator:
            self.sim_note.setText(f"Simulator: choose “{SCENARIOS['free']}” on the Connect page for this check.")
        else:
            self.sim_note.setText("Simulator: press “Turn the output one turn” instead of turning it by hand."
                                  if is_simulator else "")
        self._update()

    def set_busy(self, busy: bool) -> None:
        self.busy = busy
        self._update()

    def set_armed(self, armed: bool) -> None:
        self.armed = armed
        self._update()

    def set_latest_position(self, motor_turns: float) -> None:
        self.latest_pos = motor_turns
        if self.start_pos is not None:
            self.live.setText(f"Motor turns since Start: {motor_turns - self.start_pos:.3f}")

    def _update(self) -> None:
        ready = self.connected and not self.busy
        self.start_button.setEnabled(ready)
        self.done_button.setEnabled(ready and self.start_pos is not None)
        self.hand_button.setVisible(self.is_simulator)
        self.hand_button.setEnabled(ready and not self.armed)
        if self.connected and self.armed:
            self.live.setText("Switch the motor off first: it would hold the output against your hand.")

    def _start(self) -> None:
        if self.armed:
            self.live.setText("Switch the motor off first: it would hold the output against your hand.")
            return
        if self.latest_pos is None:
            self.live.setText("No reading from the ODrive yet. Wait a moment and press Start again.")
            return
        self.start_pos = self.latest_pos
        self.live.setText("Counting. Turn the output now, then press Done.")
        self._update()

    def _done(self) -> None:
        if self.start_pos is None or self.latest_pos is None:
            return
        motor = abs(self.latest_pos - self.start_pos)
        output = self.turns.value()
        if motor < 0.5:
            self.live.setText("The motor hardly turned. Turn the output, then press Done.")
            return
        ratio = motor / output
        self.start_pos = None
        self.live.setText(f"✔ {motor:.3f} motor turns for {output:g} output turns.")
        self._show(ratio)
        self._update()
        self.measured.emit({"ratio": ratio, "output_turns": output, "motor_turns": motor})

    def _show(self, ratio: float | None) -> None:
        self.r_ratio.set(sig(ratio, 6))
        if ratio and self.profile:
            self.r_diff.set(f"{(ratio / self.profile.gear_ratio - 1.0) * 100.0:+.2f}")
        else:
            self.r_diff.set("—")


# --- Overview and export -----------------------------------------------------------------------

SOURCE_TEXT = {MEASURED: "✔ measured", CALCULATED: "✔ from measurements", SETUP: "▲ from Setup",
               MISSING: "○ not measured"}


class OverviewPanel(QWidget):
    values_changed = Signal(dict)   # profile fields edited here

    def __init__(self, settings: AppSettings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.profile: ActuatorProfile | None = None
        self._loading = False
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)
        root.addWidget(label(
            "<b>Simulation model.</b> Everything a robot simulator needs to know about this actuator, from the "
            "tests in the next tabs. Run each test once per actuator type, then export the model. Isla Studio "
            "reads the exported <code>actuator_type</code> block into robot.json; any simulator can use the values "
            "(the names are MuJoCo's where MuJoCo has them).", "secondary", wrap=True))

        root.addWidget(section_title("Your robot"))
        grid = form_grid()
        self.battery = NumberField(1, 100, 24, decimals=1, step=0.1, suffix="V")
        self.peak = NumberField(0, 200, 0, decimals=1, step=1, suffix="A")
        self.temp_limit = NumberField(30, 200, 80, decimals=0, step=5, suffix="°C")
        for row, (text, widget, hint) in enumerate((
                ("Battery voltage", self.battery, "nominal, e.g. 22.2 V for 6S. Sets the no-load and corner speed."),
                ("Peak current", self.peak, "the most the robot's drivers give for a moment; 0 = Max motor current."),
                ("Motor temperature limit", self.temp_limit, "sets the continuous current from the thermal test."))):
            grid.addWidget(label(text), row, 0)
            grid.addWidget(widget, row, 1)
            grid.addWidget(label(hint, "muted", wrap=True), row, 2, 1, 2)
        root.addLayout(grid)
        for widget in (self.battery, self.peak, self.temp_limit):
            widget.valueChanged.connect(self._edited)

        root.addWidget(section_title("Values"))
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Value", "Result", "Where it comes from", "Measured by"])
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setMinimumHeight(360)
        root.addWidget(self.table)

        row = QHBoxLayout()
        self.export_button = QPushButton("Export simulation model…")
        self.export_button.setObjectName("primary")
        self.export_button.clicked.connect(self._export)
        self.copy_button = QPushButton("Copy actuator_type for Isla Studio")
        self.copy_button.clicked.connect(self._copy)
        row.addWidget(self.export_button)
        row.addWidget(self.copy_button)
        row.addStretch(1)
        root.addLayout(row)
        self.status = label("", "secondary", wrap=True)
        root.addWidget(self.status)
        root.addStretch(1)

    def set_profile(self, profile: ActuatorProfile) -> None:
        self.profile = profile
        self._loading = True
        self.battery.setValue(profile.battery_voltage)
        self.peak.setValue(profile.peak_current_a)
        self.temp_limit.setValue(profile.temp_limit_c)
        self._loading = False
        self._render()

    def _edited(self, *_args) -> None:
        if self._loading or self.profile is None:
            return
        self.values_changed.emit({"battery_voltage": self.battery.value(), "peak_current_a": self.peak.value(),
                                  "temp_limit_c": self.temp_limit.value()})

    def _render(self) -> None:
        values = model_values(self.profile)
        self.table.setRowCount(len(values))
        for i, v in enumerate(values):
            if v.known:
                result = f"{sig(v.value, 4)} {v.unit}".replace(" : 1", ": 1")
                if v.unit == "rad/s":
                    result += f"  ({v.value * RPM_PER_RAD_S:.3g} rpm)"
            else:
                result = "—"
            where = SOURCE_TEXT[v.source] + (f": {v.note}" if v.note else "")
            for col, text in enumerate((f"{v.label}  ({v.key})", result, where, TESTS.get(v.test, v.test))):
                self.table.setItem(i, col, QTableWidgetItem(text))
        measured = sum(v.source in (MEASURED, CALCULATED) for v in values)
        self.status.setText(f"{measured} of {len(values)} values come from measurements. "
                            + ("Every value is measured." if measured == len(values)
                               else "Run the tests marked ○ (and ▲ for the datasheet ones) to fill in the rest."))

    def _export(self) -> None:
        if self.profile is None:
            return
        self.settings.data_path.mkdir(parents=True, exist_ok=True)
        name = "".join(c if c.isalnum() or c in "-_" else "-" for c in self.profile.name).strip("-") or "actuator"
        default = str(self.settings.data_path / f"{timestamp()}_{name}_simulation-model.json")
        path, _ = QFileDialog.getSaveFileName(self, "Export simulation model", default, "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(simulation_model(self.profile), f, indent=2, ensure_ascii=False, allow_nan=False)
        except OSError as exc:
            self.status.setText(f"▲ Could not save {path}: {exc}")
            return
        self.status.setText(f"✔ Saved {path}. Send it to whoever sets up the robot's simulation, or load it there.")

    def _copy(self) -> None:
        if self.profile is None:
            return
        block = simulation_model(self.profile)["actuator_type"]
        QGuiApplication.clipboard().setText(json.dumps(block, indent=2, ensure_ascii=False))
        count = len(block["measured"])
        self.status.setText(f"✔ Copied {count} measured values. Paste them into this actuator's entry under "
                            "actuator_types in robot.json." if count else
                            "▲ Copied, but nothing is measured yet: run the tests first.")


# --- The tab -----------------------------------------------------------------------------------

class ModelPage(QWidget):
    result_measured = Signal(str, dict)    # a result measured without a procedure (gear ratio)
    profile_values_changed = Signal(dict)  # robot values edited on the Overview

    def __init__(self, worker, settings: AppSettings, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        layout.addWidget(self.tabs)
        self.overview = OverviewPanel(settings)
        self.friction = FrictionPanel(worker)
        self.inertia = InertiaPanel(worker)
        self.speed = SpeedPanel(worker)
        self.backlash = BacklashPanel(worker)
        self.thermal = ThermalPanel(worker)
        self.gear = GearRatioPanel(worker)
        self.test_panels = [self.friction, self.inertia, self.speed, self.backlash, self.thermal]
        for widget, title in ((self.overview, "Overview and export"), (self.friction, "Friction"),
                              (self.inertia, "Inertia"), (self.speed, "No-load speed"),
                              (self.backlash, "Backlash"), (self.thermal, "Thermal"), (self.gear, "Gear ratio")):
            self.tabs.addTab(scrollable(widget), title)
        self.gear.measured.connect(lambda result: self.result_measured.emit("gear_ratio", result))
        self.overview.values_changed.connect(self.profile_values_changed.emit)

    def set_profile(self, profile: ActuatorProfile, kt_odrive: float | None = None) -> None:
        self.overview.set_profile(profile)
        for panel in (*self.test_panels, self.gear):
            panel.set_profile(profile, kt_odrive)

    def set_connected(self, connected: bool, is_simulator: bool = False, scenario: str = "") -> None:
        for panel in (*self.test_panels, self.gear):
            panel.set_connected(connected)
            panel.set_simulator(is_simulator, scenario)

    def set_busy(self, busy: bool) -> None:
        for panel in (*self.test_panels, self.gear):
            panel.set_busy(busy)

    def set_armed(self, armed: bool) -> None:
        self.gear.set_armed(armed)

    def set_latest_position(self, motor_turns: float) -> None:
        self.gear.set_latest_position(motor_turns)
