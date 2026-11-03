"""The simulation model: every measured value of one actuator, ready for a robot simulator.

``model_values`` turns the latest result of each test on a profile into the values a simulator's
actuator needs, each with where it came from. ``simulation_model`` packs them into a JSON file.
Its ``actuator_type`` block uses the field names of Isla Studio's robot.json (MuJoCo's names where
MuJoCo has the value); values nobody measured are left out and listed under ``not_measured`` so
they never replace better numbers with guesses.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import date, datetime

from . import APP_NAME, __version__
from .identification import MotorConstants, ThermalFit, corner_speed, torque_speed_curve, zero_torque_speed
from .odrive_api import MAX_PHASE_VOLTAGE_PER_BUS_VOLT
from .settings import ActuatorProfile

MEASURED, CALCULATED, SETUP, MISSING = "measured", "calculated", "from Setup", "not measured"

TESTS = {
    "calibration": "Calibration",
    "gear_ratio": "Gear ratio check",
    "friction": "Friction test",
    "inertia": "Inertia test",
    "speed": "Speed test",
    "backlash": "Backlash test",
    "thermal": "Thermal test",
}


@dataclass
class ModelValue:
    key: str                  # field name in the export (Isla Studio / MuJoCo where they have one)
    label: str
    value: float | None
    unit: str
    source: str               # MEASURED, CALCULATED, SETUP or MISSING
    test: str                 # the test that measures it
    note: str = ""

    @property
    def known(self) -> bool:
        return self.value is not None and math.isfinite(self.value)


def _motor(result: dict) -> MotorConstants | None:
    motor = result.get("motor") if result else None
    if not motor:
        return None
    return MotorConstants(motor["flux_Vs_per_rad"], motor["phase_resistance_ohm"], motor["phase_inductance_H"],
                          int(motor["pole_pairs"]))


def _thermal(result: dict) -> ThermalFit | None:
    if not result or "rise_C" not in result:
        return None
    return ThermalFit(result["test_current_A"], result["start_temp_C"], result["rise_C"], result["time_constant_s"],
                      result["duration_s"], result.get("sensor", "motor"))


def torque_per_amp(profile: ActuatorProfile) -> tuple[float, str]:
    if profile.has_calibration:
        return float(profile.calibration_k_out), MEASURED
    return profile.kt_motor * profile.gear_ratio * profile.efficiency_pct / 100.0, SETUP


def model_values(profile: ActuatorProfile) -> list[ModelValue]:
    r = profile.model_results
    volts, peak, limit = profile.battery_voltage, profile.robot_peak_current, profile.temp_limit_c
    values: list[ModelValue] = []

    ratio = r.get("gear_ratio", {}).get("ratio")
    gear = float(ratio) if ratio else profile.gear_ratio
    values.append(ModelValue("gear_ratio", "Gear ratio", gear, ": 1", MEASURED if ratio else SETUP, "gear_ratio"))

    k_out, k_source = torque_per_amp(profile)
    values.append(ModelValue("torque_constant", "Torque per amp at the output", k_out, "N·m/A", k_source,
                             "calibration", "" if k_source == MEASURED else "Kt × gear ratio × efficiency"))
    values.append(ModelValue("peak_torque", "Peak torque", k_out * peak, "N·m",
                             CALCULATED if k_source == MEASURED else SETUP, "calibration",
                             f"at {peak:g} A peak current"))

    thermal = _thermal(r.get("thermal", {}))
    if thermal:
        current = thermal.continuous_current(limit)
        note = f"motor at {limit:g} °C" if thermal.sensor == "motor" else f"driver (not motor) at {limit:g} °C"
        if thermal.extrapolated:
            note += "; short test, an estimate"
        values.append(ModelValue("continuous_current", "Continuous current", current, "A", MEASURED, "thermal", note))
        values.append(ModelValue("continuous_torque", "Continuous torque", k_out * current, "N·m", CALCULATED,
                                 "thermal"))
    else:
        values.append(ModelValue("continuous_current", "Continuous current", None, "A", MISSING, "thermal"))
        values.append(ModelValue("continuous_torque", "Continuous torque", None, "N·m", MISSING, "thermal"))

    motor = _motor(r.get("speed", {}))
    if motor:
        note = f"at {volts:g} V"
        if not r["speed"].get("resistance_measured", True):
            note += "; no phase resistance from the ODrive, 0 Ω assumed"
        values.append(ModelValue("no_load_speed", "No-load speed", zero_torque_speed(motor, volts) / gear, "rad/s",
                                 MEASURED, "speed", note))
        values.append(ModelValue("corner_speed", "Corner speed (full torque up to here)",
                                 corner_speed(motor, volts, peak) / gear, "rad/s", CALCULATED, "speed",
                                 f"at {volts:g} V and {peak:g} A"))
    else:
        values.append(ModelValue("no_load_speed", "No-load speed", None, "rad/s", MISSING, "speed"))
        values.append(ModelValue("corner_speed", "Corner speed (full torque up to here)", None, "rad/s", MISSING, "speed"))

    inertia = r.get("inertia", {}).get("inertia_kgm2")
    values.append(ModelValue("armature", "Reflected inertia (armature)", inertia, "kg·m²",
                             MEASURED if inertia is not None else MISSING, "inertia"))

    friction = r.get("friction", {})
    for key, label, field, unit in (("frictionloss", "Friction (Coulomb)", "coulomb_Nm", "N·m"),
                                    ("damping", "Damping (viscous friction)", "viscous_Nms_per_rad", "N·m·s/rad")):
        value = friction.get(field)
        values.append(ModelValue(key, label, value, unit, MEASURED if value is not None else MISSING, "friction"))

    backlash = r.get("backlash", {})
    play = backlash.get("backlash_deg")
    values.append(ModelValue("backlash", "Backlash", None if play is None else math.radians(play), "rad",
                             MEASURED if play is not None else MISSING, "backlash",
                             "" if play is None else f"{play:.3g}°"))
    stiffness = backlash.get("stiffness_Nm_per_rad")
    values.append(ModelValue("stiffness", "Gearbox stiffness", stiffness, "N·m/rad",
                             MEASURED if stiffness is not None else MISSING, "backlash"))
    return values


def record_result(profile: ActuatorProfile, test: str, result: dict) -> None:
    """Keep a finished test's numbers (not its samples) on the profile, dated."""
    summary = {k: v for k, v in result.items() if k not in ("series", "test")}
    summary["date"] = date.today().isoformat()
    profile.model_results[test] = _clean(summary)


def simulation_model(profile: ActuatorProfile) -> dict:
    """Everything measured about ``profile``'s actuator, as one JSON-ready dict."""
    values = model_values(profile)
    found = {v.key: v for v in values}
    exported = [v for v in values if v.known and v.source in (MEASURED, CALCULATED) and v.key != "continuous_torque"]
    actuator_type: dict = {v.key: float(f"{v.value:.6g}") for v in exported}
    actuator_type["measured"] = [v.key for v in exported]
    dates = {key: result.get("date", "") for key, result in profile.model_results.items()}
    if profile.has_calibration:
        dates["calibration"] = profile.calibration_date
    actuator_type["note"] = (f"Measured on an ODrive bench with {APP_NAME} {__version__}. "
                             + ", ".join(f"{TESTS[k]} {d}" for k, d in sorted(dates.items()) if k in TESTS and d) + ".")

    extra: dict = {}
    motor = _motor(profile.model_results.get("speed", {}))
    k_out = found["torque_constant"].value
    gear = found["gear_ratio"].value
    if motor:
        extra["motor"] = motor.to_dict()
        if profile.has_calibration:  # measured torque per amp over what the motor alone gives
            extra["gearbox_efficiency_from_back_emf"] = k_out / (motor.kt * gear)
        extra["torque_speed_curve"] = [
            {"speed_rad_s": round(w, 4), "torque_Nm": round(t, 4)}
            for w, t in torque_speed_curve(motor, profile.battery_voltage, profile.robot_peak_current, k_out, gear)]
    thermal = _thermal(profile.model_results.get("thermal", {}))
    if thermal:
        extra["thermal"] = {
            **thermal.to_dict(),
            "seconds_at_peak_current_from_cold": thermal.time_to_limit(profile.robot_peak_current, profile.temp_limit_c),
        }
    if found["backlash"].known:
        extra["backlash_deg"] = math.degrees(found["backlash"].value)

    return {
        "format": "odrive-actuator-bench/simulation-model",
        "version": 1,
        "actuator": profile.name,
        "created": datetime.now().isoformat(timespec="seconds"),
        "conditions": {
            "battery_voltage_V": profile.battery_voltage,
            "peak_current_A": profile.robot_peak_current,
            "motor_temp_limit_C": profile.temp_limit_c,
            "driver_phase_voltage_per_bus_volt": MAX_PHASE_VOLTAGE_PER_BUS_VOLT,
        },
        "actuator_type": actuator_type,
        "not_measured": [v.key for v in values if v.source in (SETUP, MISSING)],
        "values": [{"key": v.key, "label": v.label, "value": v.value if v.known else None, "unit": v.unit,
                    "source": v.source, "note": v.note} for v in values],
        "extra": _clean(extra),
        "tests": _clean(profile.model_results),
    }


def _clean(value):
    """JSON has no infinity or NaN."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    return value
