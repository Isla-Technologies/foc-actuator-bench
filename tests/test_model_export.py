"""The simulation model gathers the tests' results and never passes a guess off as a measurement."""

import json
import math

import pytest

from actuator_bench.identification import MotorConstants, corner_speed, zero_torque_speed
from actuator_bench.model_export import MEASURED, MISSING, SETUP, model_values, record_result, simulation_model
from actuator_bench.settings import ActuatorProfile, AppSettings, load_settings, save_settings


def test_an_unmeasured_actuator_exports_nothing_as_measured():
    profile = ActuatorProfile(kv=90, gear_ratio=36, efficiency_pct=90, max_current_a=22)
    values = {v.key: v for v in model_values(profile)}
    assert values["gear_ratio"].source == SETUP
    assert values["torque_constant"].value == pytest.approx(8.27 / 90 * 36 * 0.9)
    assert values["armature"].source == MISSING and not values["armature"].known
    model = simulation_model(profile)
    assert model["actuator_type"]["measured"] == []
    assert set(model["actuator_type"]) == {"measured", "note"}
    assert {"torque_constant", "armature", "no_load_speed", "frictionloss"} <= set(model["not_measured"])
    json.dumps(model, allow_nan=False)


def measured_profile() -> ActuatorProfile:
    profile = ActuatorProfile(name="Luma-1 macro", kv=90, gear_ratio=36, efficiency_pct=90, max_current_a=22,
                              battery_voltage=22.2, peak_current_a=30, temp_limit_c=90)
    profile.calibration_k_out = 2.9
    profile.calibration_date = "2026-11-03"
    motor = MotorConstants(flux=0.0613, resistance=0.12, inductance=6e-5, pole_pairs=14)
    results = {
        "gear_ratio": {"test": "gear_ratio", "ratio": 36.02},
        "friction": {"test": "friction", "coulomb_Nm": 1.4, "viscous_Nms_per_rad": 0.28, "series": {"t": [0, 1]}},
        "inertia": {"test": "inertia", "inertia_kgm2": 0.097},
        "speed": {"test": "speed", "motor": motor.to_dict(), "resistance_measured": True},
        "backlash": {"test": "backlash", "backlash_deg": 0.1, "stiffness_Nm_per_rad": 5800.0},
        "thermal": {"test": "thermal", "test_current_A": 15.0, "start_temp_C": 25.0, "rise_C": 40.0,
                    "time_constant_s": 600.0, "duration_s": 900.0, "sensor": "motor"},
    }
    for name, result in results.items():
        record_result(profile, name, result)
    return profile


def test_a_measured_actuator_exports_every_value():
    profile = measured_profile()
    assert "series" not in profile.model_results["friction"]
    assert profile.model_results["friction"]["date"]
    model = simulation_model(profile)
    a = model["actuator_type"]
    motor = MotorConstants(flux=0.0613, resistance=0.12, inductance=6e-5, pole_pairs=14)
    assert a["gear_ratio"] == pytest.approx(36.02)
    assert a["torque_constant"] == pytest.approx(2.9)
    assert a["peak_torque"] == pytest.approx(2.9 * 30)
    assert a["frictionloss"] == pytest.approx(1.4)
    assert a["damping"] == pytest.approx(0.28)
    assert a["armature"] == pytest.approx(0.097)
    assert a["no_load_speed"] == pytest.approx(zero_torque_speed(motor, 22.2) / 36.02, rel=1e-5)
    assert a["corner_speed"] == pytest.approx(corner_speed(motor, 22.2, 30) / 36.02, rel=1e-5)
    assert a["backlash"] == pytest.approx(math.radians(0.1), rel=1e-4)
    assert a["stiffness"] == pytest.approx(5800)
    # 65 °C of headroom at 15 A makes 40 °C, so heat (∝ I²) allows 15 × sqrt(65/40).
    assert a["continuous_current"] == pytest.approx(15 * math.sqrt(65 / 40), rel=1e-5)
    assert "continuous_torque" not in a
    assert model["not_measured"] == []
    assert model["extra"]["gearbox_efficiency_from_back_emf"] == pytest.approx(2.9 / (1.5 * 0.0613 * 36.02))
    curve = model["extra"]["torque_speed_curve"]
    assert curve[0]["torque_Nm"] == pytest.approx(2.9 * 30, rel=1e-3)
    assert curve[-1]["torque_Nm"] == pytest.approx(0, abs=1e-3)
    assert "Friction test" in a["note"] and "Calibration 2026-11-03" in a["note"]
    json.dumps(model, allow_nan=False)


def test_model_results_survive_saving(tmp_path):
    settings = AppSettings()
    settings.upsert_profile(measured_profile())
    save_settings(settings, tmp_path / "settings.json")
    loaded = load_settings(tmp_path / "settings.json").profile("Luma-1 macro")
    assert loaded.model_results["inertia"]["inertia_kgm2"] == pytest.approx(0.097)
    assert loaded.battery_voltage == 22.2
    assert {v.key: v.source for v in model_values(loaded)}["armature"] == MEASURED


def test_two_actuators_on_different_controllers_export_side_by_side(tmp_path):
    """One profile per actuator: an ODrive one and a SimpleFOC one, each with its own model."""
    settings = AppSettings()
    settings.upsert_profile(measured_profile())  # on an ODrive (the default)
    foc = ActuatorProfile(name="BE4108 on ESP32 FOC", controller="simplefoc", kv=380, pole_pairs=11, gear_ratio=1,
                          max_current_a=3, battery_voltage=24.0)
    motor = MotorConstants(flux=8.27 / 380 / 1.5, resistance=0.2, inductance=1e-4, pole_pairs=11)
    # SVPWM (1/√3 of the supply), but the sketch's voltage_limit of 11 V comes first at 24 V.
    record_result(foc, "speed", {"test": "speed", "motor": motor.to_dict(), "resistance_measured": True,
                                 "phase_voltage_per_bus_volt": 1 / math.sqrt(3), "phase_voltage_limit_V": 11.0})
    record_result(foc, "inertia", {"test": "inertia", "inertia_kgm2": 5.5e-5})
    settings.upsert_profile(foc)
    save_settings(settings, tmp_path / "settings.json")
    loaded = load_settings(tmp_path / "settings.json")

    odrive_model = simulation_model(loaded.profile("Luma-1 macro"))
    foc_model = simulation_model(loaded.profile("BE4108 on ESP32 FOC"))
    assert odrive_model["conditions"]["controller"].startswith("ODrive")
    assert foc_model["conditions"]["controller"].startswith("SimpleFOC")
    assert "SimpleFOC" in foc_model["actuator_type"]["note"]
    assert foc_model["actuator_type"]["armature"] == pytest.approx(5.5e-5)
    assert foc_model["actuator_type"]["no_load_speed"] == pytest.approx(11.0 / motor.flux, rel=1e-5)
    assert foc_model["conditions"]["driver_phase_voltage_per_bus_volt"] == pytest.approx(11.0 / 24.0)
    assert odrive_model["actuator_type"]["armature"] == pytest.approx(0.097)
    # On a 12 V battery the modulation, not the voltage limit, sets the top speed.
    foc12 = loaded.profile("BE4108 on ESP32 FOC")
    foc12.battery_voltage = 12.0
    assert simulation_model(foc12)["actuator_type"]["no_load_speed"] == pytest.approx(
        12.0 / math.sqrt(3) / motor.flux, rel=1e-5)
    json.dumps(foc_model, allow_nan=False)
