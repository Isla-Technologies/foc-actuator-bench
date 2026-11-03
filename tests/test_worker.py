"""End-to-end: worker + device layer + procedures against the simulated ODrive."""

import math

import numpy as np
import pytest

from actuator_bench.backends.serial_backend import SerialBackend
from actuator_bench.backends.sim_serial import SimulatedSerialPort
from actuator_bench.backends.simulator import SimActuator
from actuator_bench.calibration import fit_calibration
from actuator_bench.odrive_api import AxisState, ControlMode
from actuator_bench.procedures import (
    CalibrationSweep,
    CalibrationSweepParams,
    HoldTest,
    HoldTestParams,
    StallTest,
    StallTestParams,
)
from actuator_bench.samples import Limits


def serial_backend(sim):
    backend = SerialBackend("FAKE", opener=lambda _d: SimulatedSerialPort(sim), timeout=0.05)
    backend.sim = sim
    return backend


def test_connect_reports_device_and_readiness(rig_factory):
    rig = rig_factory()
    status = rig.listener.connection[-1]
    assert status.info.firmware == "0.5.6"
    assert status.info.firmware_supported
    assert status.readiness.can_arm
    assert status.readiness.kt_odrive == pytest.approx(0.0827)
    rig.run(0.5)
    assert len(rig.listener.column("t")) >= 45
    assert abs(rig.listener.column("torque_out").mean()) < 0.05
    assert rig.listener.slow[-1].vbus == pytest.approx(24.0, abs=0.5)


@pytest.mark.parametrize("backend_factory", [None, serial_backend], ids=["direct", "serial-protocol"])
def test_stall_test_reaches_target_against_fixed_object(rig_factory, backend_factory):
    rig = rig_factory(actuator=SimActuator(scenario="wall", wall_angle_deg=4.0), backend_factory=backend_factory)
    test = StallTest(StallTestParams(target_torque_out=3.0, ramp_time_s=1.0, hold_time_s=0.5, ramp_down_time_s=0.2))
    status = rig.run_procedure(test)
    assert status.success, status.message
    result = status.result
    assert result["peak_abs_torque_Nm"] == pytest.approx(3.0, rel=0.05)
    assert result["hold_mean_torque_Nm"] == pytest.approx(3.0, rel=0.03)
    assert result["max_movement_deg"] == pytest.approx(4.0, abs=0.5)
    assert not rig.worker.armed
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE
    # Settings changed for the test are restored when disconnecting.
    rig.worker.disconnect()
    rig.worker.step()
    assert rig.sim.read("axis0.controller.config.vel_limit") == 2.0
    assert math.isinf(rig.sim.read("axis0.motor.config.torque_lim"))
    assert rig.sim.read("axis0.controller.config.control_mode") == ControlMode.POSITION
    assert rig.sim.read("axis0.config.enable_watchdog") is False


def test_stall_test_aborts_when_arm_is_not_blocked(rig_factory):
    rig = rig_factory(actuator=SimActuator(scenario="free"))
    status = rig.run_procedure(StallTest(StallTestParams(target_torque_out=2.0, ramp_time_s=1.0)))
    assert not status.success
    assert "isn't blocked" in status.message
    assert not rig.worker.armed
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE


def test_stall_test_refuses_target_above_current_limit(rig_factory):
    rig = rig_factory(limits=Limits(max_current=2.0, max_motor_speed=2.0))
    status = rig.run_procedure(StallTest(StallTestParams(target_torque_out=5.0)))
    assert not status.success
    assert "current limit" in status.message
    assert not rig.worker.armed


def test_hold_test_measures_holding_torque_of_hanging_weight(rig_factory):
    actuator = SimActuator(scenario="lever", mass_kg=0.0, arm_mass_kg=0.0, arm_length_m=0.3)
    rig = rig_factory(actuator=actuator)
    test = HoldTest(HoldTestParams(max_torque_out=6.0, duration_s=3.0, max_deflection_deg=20))
    rig.worker.start_procedure(test)
    rig.worker.step()
    rig.run(0.5)
    rig.sim.update_actuator(mass_kg=1.0)  # hang 1 kg on the arm while it is held
    status = rig.run_procedure(test)
    assert status.success, status.message
    gravity = 1.0 * 9.80665 * 0.3
    # The estimate uses the nominal Kt and 100 % efficiency, the "real" actuator differs by 0.945x.
    expected_estimate = gravity / rig.sim.actuator.true_k_out * rig.model.k_out
    # Steady holding torque matches the load (within friction); catching the weight adds a short peak.
    friction = rig.sim.actuator.coulomb_friction / rig.sim.actuator.true_k_out * rig.model.k_out
    assert status.result["steady_torque_Nm"] == pytest.approx(expected_estimate, abs=friction + 0.05)
    assert expected_estimate * 0.9 < status.result["peak_abs_Nm"] < expected_estimate * 1.6
    assert status.result["max_deflection_deg"] < 10


def test_weight_calibration_recovers_true_output_constant(rig_factory):
    actuator = SimActuator(scenario="lever", mass_kg=0.0, arm_length_m=0.3)
    rig = rig_factory(actuator=actuator)
    points = []
    for mass in (0.0, 0.5, 1.0):
        rig.worker.configure_simulator(mass_kg=mass)
        sweep = CalibrationSweep(CalibrationSweepParams(mass_kg=mass, arm_length_m=0.3, horizontal_pos=0.0, cycles=2))
        status = rig.run_procedure(sweep)
        assert status.success, status.message
        assert rig.worker.armed  # keeps holding the arm between weights
        points.append(status.result["point"])
    fit = fit_calibration(points)
    assert fit.k_out_nm_per_a == pytest.approx(actuator.true_k_out, rel=0.02)
    assert fit.r_squared > 0.999
    # friction shows up as the up/down difference and is cancelled in the fit
    assert points[-1].iq_friction_a == pytest.approx(actuator.coulomb_friction / actuator.true_k_out, rel=0.3)


def test_manual_torque_mode_and_emergency_stop(rig_factory):
    rig = rig_factory(actuator=SimActuator(scenario="locked"))
    rig.worker.arm(ControlMode.TORQUE)
    rig.worker.set_output_torque(1.0)
    rig.run(1.0)
    assert rig.worker.armed
    assert rig.sim.read("axis0.config.enable_watchdog") is True
    torque = rig.listener.column("torque_out")[-60:].mean()
    cap = rig.model.output_torque(3.0 * 0.16 / 0.0827)  # standstill cap of the torque-mode speed limiter
    assert torque == pytest.approx(min(1.0, cap), rel=0.05)
    rig.worker.estop()
    rig.run(0.05)
    assert not rig.worker.armed
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE
    assert any("STOP" in text for _, text in rig.listener.messages)


def test_manual_setpoints_are_clamped_to_limits(rig_factory):
    rig = rig_factory(actuator=SimActuator(scenario="locked"), limits=Limits(max_current=5.0, max_motor_speed=50.0))
    rig.worker.arm(ControlMode.TORQUE)
    rig.worker.set_output_torque(100.0)
    rig.run(0.3)
    assert rig.sim.read("axis0.motor.current_control.Iq_setpoint") == pytest.approx(5.0, rel=0.01)


def test_watchdog_stops_motor_if_app_freezes(rig_factory):
    rig = rig_factory(actuator=SimActuator(scenario="locked"))
    rig.worker.arm(ControlMode.TORQUE)
    rig.run(0.2)
    rig.clock.advance(2.0)  # the app stops talking for 2 s
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE
    rig.run(0.5)
    assert not rig.worker.armed
    assert any("WATCHDOG_TIMER_EXPIRED" in text for _, text in rig.listener.messages)


def test_arming_with_errors_is_refused(rig_factory):
    rig = rig_factory()
    rig.sim.inject_error(0, "motor", 0x8)
    rig.worker.arm(ControlMode.POSITION)
    rig.worker.step()
    assert not rig.worker.armed
    assert any("DRV_FAULT" in text for _, text in rig.listener.messages)
    rig.worker.clear_errors()
    rig.worker.step()
    assert rig.listener.readiness[-1].can_arm


def test_snapshot_contains_config(rig_factory):
    rig = rig_factory()
    rig.worker.request_snapshot()
    rig.worker.step()
    snap = rig.listener.snapshots[-1]
    assert snap["connected"]
    assert snap["config"]["axis0.motor.config.torque_constant"] == pytest.approx(0.0827)
    assert snap["info"]["firmware"] == "0.5.6"


def test_connection_loss_is_reported(rig_factory):
    rig = rig_factory(backend_factory=serial_backend)
    rig.backend._ser.close()
    rig.run(0.2)
    assert rig.listener.connection[-1].state == "disconnected"
    assert "Connection lost" in rig.listener.connection[-1].message
    assert not rig.worker.connected


def test_sample_rate_is_measured(rig_factory):
    rig = rig_factory()
    rig.run(2.5)
    assert rig.listener.slow[-1].sample_rate_hz == pytest.approx(100, rel=0.05)
    t = rig.listener.column("t")
    assert np.all(np.diff(t) > 0)
