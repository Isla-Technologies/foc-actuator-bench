"""The simulated ODrive behaves like firmware 0.5.x where the app depends on it."""

import pytest

from actuator_bench.backends.simulator import ManualClock, SimActuator, SimulatedODrive
from actuator_bench.odrive_api import AxisState, ControlMode


def make(scenario="locked", **kw):
    clock = ManualClock()
    sim = SimulatedODrive(SimActuator(scenario=scenario, **kw), clock=clock)
    return sim, clock


def run(sim, clock, seconds, dt=0.01):
    for _ in range(int(seconds / dt)):
        clock.advance(dt)
        sim.read("axis0.encoder.pos_estimate")


def arm_torque_mode(sim, torque, vel_limit=100.0, limiter=True):
    sim.write("axis0.controller.config.control_mode", int(ControlMode.TORQUE))
    sim.write("axis0.controller.config.vel_limit", vel_limit)
    sim.write("axis0.controller.config.enable_torque_mode_vel_limit", limiter)
    sim.write("axis0.controller.input_torque", torque)
    sim.write("axis0.requested_state", int(AxisState.CLOSED_LOOP_CONTROL))


def test_identity_and_defaults():
    sim, _ = make()
    assert sim.read("fw_version_minor") == 5
    assert sim.read("hw_version_variant") == 56
    assert sim.read("axis0.motor.config.torque_constant") == pytest.approx(0.0827)
    assert sim.read("axis0.current_state") == AxisState.IDLE
    assert sim.read("axis1.motor.is_calibrated") is False


def test_current_follows_torque_command_on_locked_output():
    sim, clock = make()
    arm_torque_mode(sim, 0.5)
    run(sim, clock, 0.1)
    assert sim.read("axis0.motor.current_control.Iq_setpoint") == pytest.approx(0.5 / 0.0827, rel=1e-3)
    iq = sum(sim.read("axis0.motor.current_control.Iq_measured") for _ in range(200)) / 200
    assert iq == pytest.approx(0.5 / 0.0827, rel=0.02)
    assert sim.true_output_torque() == pytest.approx(sim.actuator.true_k_out * 0.5 / 0.0827, rel=1e-3)


def test_torque_mode_speed_limiter_caps_standstill_torque_like_firmware():
    sim, clock = make()
    arm_torque_mode(sim, 1.0, vel_limit=1.0)
    run(sim, clock, 0.1)
    # Tmax = (vel_limit - vel) * vel_gain = 1.0 * 0.16
    assert sim.read("axis0.motor.current_control.Iq_setpoint") == pytest.approx(0.16 / 0.0827, rel=1e-3)


def test_overspeed_error_when_output_runs_free():
    sim, clock = make(scenario="free")
    arm_torque_mode(sim, 0.3, vel_limit=2.0, limiter=False)
    run(sim, clock, 3.0)
    assert sim.read("axis0.current_state") == AxisState.IDLE
    assert sim.read("axis0.controller.error") & 0x1
    assert sim.read("axis0.error") & 0x200


def test_requesting_closed_loop_without_calibration_fails():
    sim, _ = make()
    sim.write("axis1.requested_state", int(AxisState.CLOSED_LOOP_CONTROL))
    assert sim.read("axis1.current_state") == AxisState.IDLE
    assert sim.read("axis1.error") & 0x1


def test_calibration_sequence_completes():
    sim, clock = make(scenario="free")
    sim.axes[0].motor_calibrated = False
    sim.write("axis0.requested_state", int(AxisState.FULL_CALIBRATION_SEQUENCE))
    assert sim.read("axis0.current_state") == AxisState.FULL_CALIBRATION_SEQUENCE
    run(sim, clock, 3.5)
    assert sim.read("axis0.current_state") == AxisState.IDLE
    assert sim.read("axis0.motor.is_calibrated") is True


def test_feeding_before_setting_the_timeout_expires_immediately():
    # Firmware: watchdog_feed() loads the countdown from the current timeout (default 0).
    sim, clock = make()
    sim.call("axis0.watchdog_feed")
    sim.write("axis0.config.watchdog_timeout", 0.5)
    sim.write("axis0.config.enable_watchdog", True)
    run(sim, clock, 0.01)
    assert sim.read("axis0.error") & 0x800


def test_watchdog_expires_when_not_fed():
    sim, clock = make()
    sim.write("axis0.config.watchdog_timeout", 0.5)
    sim.call("axis0.watchdog_feed")
    sim.write("axis0.config.enable_watchdog", True)
    arm_torque_mode(sim, 0.1)
    run(sim, clock, 0.3)
    assert sim.read("axis0.current_state") == AxisState.CLOSED_LOOP_CONTROL
    run(sim, clock, 0.5)
    assert sim.read("axis0.current_state") == AxisState.IDLE
    assert sim.read("axis0.error") & 0x800


def test_entering_closed_loop_holds_current_position():
    sim, clock = make(scenario="free")
    sim.axes[0].pos = 3.25
    sim.write("axis0.controller.config.control_mode", int(ControlMode.POSITION))
    sim.write("axis0.controller.input_pos", 0.0)  # stale target must not cause a jump
    sim.write("axis0.requested_state", int(AxisState.CLOSED_LOOP_CONTROL))
    run(sim, clock, 0.5)
    assert sim.read("axis0.encoder.pos_estimate") == pytest.approx(3.25, abs=0.01)


def test_clear_errors():
    sim, _ = make()
    sim.inject_error(0, "motor", 0x8)
    assert sim.read("axis0.motor.error") == 0x8
    sim.call("clear_errors")
    assert sim.read("axis0.motor.error") == 0
    assert sim.read("axis0.error") == 0


def test_back_emf_caps_the_speed_at_the_no_load_speed():
    from actuator_bench.odrive_api import MAX_PHASE_VOLTAGE_PER_BUS_VOLT

    sim, clock = make(scenario="free")
    sim.write("axis0.controller.config.control_mode", int(ControlMode.VELOCITY))
    sim.write("axis0.controller.config.vel_limit", 60.0)
    sim.write("axis0.controller.input_vel", 50.0)
    sim.write("axis0.requested_state", int(AxisState.CLOSED_LOOP_CONTROL))
    run(sim, clock, 3.0)
    no_load = MAX_PHASE_VOLTAGE_PER_BUS_VOLT * 24.0 / sim.actuator.flux / (2 * 3.141592653589793)
    assert sim.read("axis0.current_state") == AxisState.CLOSED_LOOP_CONTROL
    assert sim.read("axis0.encoder.vel_estimate") == pytest.approx(no_load, rel=0.03)
    # The controller asks for more current than the voltage lets through.
    assert sim.read("axis0.motor.current_control.Iq_setpoint") > 3 * sim.read("axis0.motor.current_control.Iq_measured")


def test_clamped_output_shows_backlash_then_stiffness():
    sim, clock = make(backlash_deg=0.4, gear_stiffness=1000.0)
    angles = {}
    for torque in (0.3, -0.3):
        arm_torque_mode(sim, torque, vel_limit=50.0)
        run(sim, clock, 0.5)
        angles[torque] = sim.read("axis0.encoder.pos_estimate") / 9 * 360
    output_torque = 0.3 * sim.actuator.true_k_out / sim.actuator.kt_nominal
    twist_deg = output_torque / 1000.0 * 180 / 3.141592653589793
    assert angles[0.3] - angles[-0.3] == pytest.approx(0.4 + 2 * twist_deg, abs=0.03)


def test_output_can_be_turned_by_hand_while_the_motor_is_off():
    sim, clock = make(scenario="free")
    sim.turn_output_by_hand(2.0)
    assert sim.read("axis0.encoder.pos_estimate") == pytest.approx(2.0 * 9)
    assert sim.read("axis0.motor.config.phase_resistance") == pytest.approx(0.06)
    assert sim.read("axis0.motor.motor_thermistor.config.enabled") is True
