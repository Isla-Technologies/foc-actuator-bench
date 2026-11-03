"""The simulation-model tests find the simulated actuator's true values."""

import math

import pytest

from actuator_bench.backends.simulator import SimActuator
from actuator_bench.model_tests import (
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
from actuator_bench.odrive_api import MAX_PHASE_VOLTAGE_PER_BUS_VOLT, AxisState
from actuator_bench.samples import Limits
from actuator_bench.torque import TorqueModel


def true_model(actuator: SimActuator) -> TorqueModel:
    """What a perfect weight calibration would give, so the tests check the method, not the datasheet."""
    return TorqueModel(kt_motor=actuator.kt_nominal, gear_ratio=actuator.gear_ratio,
                       calibrated_k_out=actuator.true_k_out)


def free_rig(rig_factory, **actuator_values):
    actuator = SimActuator(scenario="free", **actuator_values)
    return rig_factory(actuator=actuator, model=true_model(actuator),
                       limits=Limits(max_current=15.0, max_motor_speed=40.0)), actuator


def test_friction_test_finds_coulomb_and_viscous_friction(rig_factory):
    rig, actuator = free_rig(rig_factory, coulomb_friction=0.3, viscous_friction=0.05)
    test = FrictionTest(FrictionTestParams(travel_deg=360.0, top_speed=8.0, levels=5))
    status = rig.run_procedure(test, timeout=120)
    assert status.success, status.message
    r = status.result
    assert r["coulomb_Nm"] == pytest.approx(0.3, rel=0.15)
    assert r["viscous_Nms_per_rad"] == pytest.approx(0.05, rel=0.2)
    assert len(r["levels"]) == 5
    # The output is back where it started and the motor is off.
    assert abs(rig.sim.read("axis0.encoder.pos_estimate")) / 9 * 360 < 30
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE


def test_friction_test_cancels_gravity_on_an_unbalanced_arm(rig_factory):
    actuator = SimActuator(scenario="lever", mass_kg=0.5, arm_length_m=0.2, coulomb_friction=0.3, viscous_friction=0.05)
    rig = rig_factory(actuator=actuator, model=true_model(actuator), limits=Limits(max_current=15.0, max_motor_speed=40.0))
    status = rig.run_procedure(FrictionTest(FrictionTestParams(travel_deg=60.0, top_speed=4.0, levels=4)), timeout=120)
    assert status.success, status.message
    assert status.result["coulomb_Nm"] == pytest.approx(0.3, rel=0.25)


def test_inertia_test_finds_the_reflected_inertia(rig_factory):
    rig, actuator = free_rig(rig_factory)
    truth = actuator.rotor_inertia * actuator.gear_ratio ** 2 + actuator.load_inertia
    status = rig.run_procedure(InertiaTest(InertiaTestParams(swing_deg=30.0, peak_speed=8.0, cycles=4)), timeout=120)
    assert status.success, status.message
    assert status.result["inertia_kgm2"] == pytest.approx(truth, rel=0.1)
    assert len(status.result["swings"]) == 3


def test_inertia_test_subtracts_what_is_attached(rig_factory):
    rig, actuator = free_rig(rig_factory)
    truth = actuator.rotor_inertia * actuator.gear_ratio ** 2 + actuator.load_inertia
    params = InertiaTestParams(swing_deg=30.0, peak_speed=8.0, cycles=4, attached_inertia=0.5 * truth)
    status = rig.run_procedure(InertiaTest(params), timeout=120)
    assert status.result["inertia_kgm2"] == pytest.approx(0.5 * truth, rel=0.2)


def test_speed_test_finds_the_no_load_speed_and_back_emf(rig_factory):
    rig, actuator = free_rig(rig_factory)
    status = rig.run_procedure(SpeedTest(SpeedTestParams(ramp_s=3.0, hold_s=0.5)), timeout=60)
    assert status.success, status.message
    r = status.result
    zero_torque = MAX_PHASE_VOLTAGE_PER_BUS_VOLT * 24.0 / actuator.flux / actuator.gear_ratio
    assert r["zero_torque_speed_rad_s"] == pytest.approx(zero_torque, rel=0.03)
    assert r["no_load_speed_rad_s"] == pytest.approx(zero_torque, rel=0.05)
    assert r["motor"]["kt_backemf_Nm_per_A"] == pytest.approx(actuator.kt_true, rel=0.03)
    assert r["phase_resistance_ohm"] == pytest.approx(actuator.phase_resistance)
    assert [run["direction"] for run in r["runs"]] == [1, -1]
    assert all(run["voltage_limited"] for run in r["runs"])


def test_speed_test_says_when_the_speed_limit_comes_first(rig_factory):
    actuator = SimActuator(scenario="free")
    rig = rig_factory(actuator=actuator, model=true_model(actuator), limits=Limits(max_current=15.0, max_motor_speed=10.0))
    status = rig.run_procedure(SpeedTest(SpeedTestParams(ramp_s=2.0, hold_s=0.5, both_directions=False)), timeout=60)
    assert not status.success
    assert "Raise Max output speed" in status.message


def test_speed_test_refuses_a_blocked_output(rig_factory):
    actuator = SimActuator(scenario="locked")
    rig = rig_factory(actuator=actuator, model=true_model(actuator), limits=Limits(max_current=5.0, max_motor_speed=40.0))
    status = rig.run_procedure(SpeedTest(SpeedTestParams(ramp_s=2.0)), timeout=60)
    assert not status.success
    assert "doesn't turn" in status.message
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE


def test_backlash_test_finds_play_and_stiffness(rig_factory):
    actuator = SimActuator(scenario="locked", backlash_deg=0.3, gear_stiffness=800.0)
    rig = rig_factory(actuator=actuator, model=true_model(actuator), limits=Limits(max_current=15.0, max_motor_speed=3.0))
    status = rig.run_procedure(BacklashTest(BacklashTestParams(max_torque_out=4.0, cycle_s=4.0)), timeout=60)
    assert status.success, status.message
    r = status.result
    assert r["stiffness_Nm_per_rad"] == pytest.approx(800.0, rel=0.1)
    assert r["backlash_deg"] == pytest.approx(0.3, abs=0.06)
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE


def test_backlash_test_stops_when_the_output_isnt_clamped(rig_factory):
    actuator = SimActuator(scenario="free")
    rig = rig_factory(actuator=actuator, model=true_model(actuator), limits=Limits(max_current=15.0, max_motor_speed=3.0))
    status = rig.run_procedure(BacklashTest(BacklashTestParams(max_torque_out=2.0, cycle_s=4.0)), timeout=30)
    assert not status.success
    assert "isn't clamped" in status.message
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE


def test_thermal_test_rates_the_continuous_current(rig_factory):
    actuator = SimActuator(scenario="wall", wall_angle_deg=1.0)
    rig = rig_factory(actuator=actuator, model=true_model(actuator), limits=Limits(max_current=15.0, max_motor_speed=3.0),
                      dt=0.1, step_s=0.01)
    params = ThermalTestParams(current_a=10.0, temp_limit_c=60.0, max_duration_s=900.0)
    status = rig.run_procedure(ThermalTest(params), timeout=1000)
    assert status.success, status.message
    r = status.result
    assert r["sensor"] == "motor"
    # Simulated motor: 0.2 °C per A² when settled, time constant 300 s.
    assert r["rise_C"] == pytest.approx(0.2 * 100, rel=0.1)
    assert r["time_constant_s"] == pytest.approx(300, rel=0.15)
    assert r["continuous_current_A"] == pytest.approx(math.sqrt((60 - 25) / 0.2), rel=0.1)
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE
