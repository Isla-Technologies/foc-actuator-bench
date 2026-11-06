"""End-to-end against a simulated SimpleFOC board: Commander framing, the controller, the worker,
the procedures and the simulation-model tests, with and without the bench firmware."""

import math

import pytest

from actuator_bench.backends.sim_simplefoc import SimulatedSimpleFOCBoard
from actuator_bench.backends.simulator import SimActuator
from actuator_bench.calibration import fit_calibration
from actuator_bench.control import ControlMode
from actuator_bench.controllers.commander import CommanderLink, format_number, parse_number, reply_prefix
from actuator_bench.controllers.simplefoc import SimpleFOCMotor
from actuator_bench.model_tests import FrictionTest, FrictionTestParams, SpeedTest, SpeedTestParams, ThermalTest, \
    ThermalTestParams
from actuator_bench.odrive_api import AxisState
from actuator_bench.procedures import CalibrationSweep, CalibrationSweepParams, StallTest, StallTestParams
from actuator_bench.samples import Limits
from actuator_bench.torque import TorqueModel


def board_factory(**board_options):
    """A connected SimpleFOCMotor talking Commander to a simulated board (the same physics as the
    simulated ODrive). ``board_options`` go to SimulatedSimpleFOCBoard (bench=False: a plain sketch)."""
    letter = board_options.pop("controller_letter", "M")

    def make(sim):
        boards = []

        def opener(_port, _baud):
            boards.append(SimulatedSimpleFOCBoard(sim, **board_options))
            return boards[-1]

        link = CommanderLink("SIM", 115200, timeout=0.05, boot_wait_s=0.2, opener=opener, kind="simulator")
        link.connect()
        device = SimpleFOCMotor(link, letter)
        device.board = boards[-1]
        return device

    return make


def foc_rig(rig_factory, actuator=None, **kwargs):
    board_options = kwargs.pop("board", {})
    return rig_factory(actuator=actuator or SimActuator(), device_factory=board_factory(**board_options), **kwargs)


# --- Commander framing -------------------------------------------------------------------------

@pytest.mark.parametrize("command, prefix", [
    ("MMG7", "MMG"), ("ME1", "ME"), ("ME", "ME"), ("MC2", "MC"), ("MLC5", "MLC"), ("MT", "MT"), ("MWT", "MWT"),
    ("M-1.5", "M"), ("M0", "M"), ("BI", "BI"), ("BW0.5", "BW"), ("BK", "BK"), ("#5", "#"), ("@3", "@"),
])
def test_reply_prefix_matches_commander(command, prefix):
    assert reply_prefix(command) == prefix


def test_numbers_fit_commander():
    assert format_number(0.0) == "0"
    assert format_number(1e-9) == "0"
    assert float(format_number(1.5)) == 1.5
    assert float(format_number(-0.0123456789)) == pytest.approx(-0.0123456789, rel=1e-6)
    assert len(format_number(123456789.123)) <= 12
    with pytest.raises(ValueError):
        format_number(math.nan)
    assert parse_number("0.12345") == pytest.approx(0.12345)
    assert math.isnan(parse_number("nan")) and math.isnan(parse_number("ovf"))
    assert parse_number("torque") is None


def test_link_skips_stray_output_and_reads_errors(rig_factory):
    rig = foc_rig(rig_factory)
    link, board = rig.device.link, rig.device.board
    board.inject_line("MOT: Align sensor.")  # looks like a reply to a target command, isn't one
    board.inject_line("debug: 42")
    assert parse_number(link.ask("M0.25")) == pytest.approx(0.25)
    assert link.ask("MZZ") is None  # Commander answers err to an unknown sub-command
    assert "MOT: Align sensor." in rig.device.device_messages


# --- Connecting ----------------------------------------------------------------------------------

def test_connect_reports_the_board_and_its_checks(rig_factory):
    rig = foc_rig(rig_factory)
    status = rig.listener.connection[-1]
    assert status.info.family == "simplefoc"
    assert status.info.board == "SimpleFOC simulated ESP32 FOC"
    assert "bench firmware" in status.info.firmware
    assert status.info.capabilities.watchdog and not status.info.capabilities.torque_mode_speed_limit
    r = status.readiness
    assert r.can_arm
    assert r.current_lim == pytest.approx(20.0)
    assert any(kind == "ok" and "FOC current" in text for kind, text in r.checks)
    assert rig.device.motor_parameters().pole_pairs == rig.sim.actuator.pole_pairs
    rig.run(0.5)
    assert rig.listener.slow[-1].vbus == pytest.approx(24.0, abs=0.5)
    assert math.isfinite(rig.listener.slow[-1].fet_temp)


def test_missing_motor_command_is_explained(rig_factory):
    rig = foc_rig(rig_factory, board={"motor_letter": "A"}, expect_connected=False)
    status = rig.listener.connection[-1]
    assert status.state == "failed"
    assert "no Commander command 'M'" in status.message and "A (motor)" in status.message


def test_voltage_mode_sketch_is_refused(rig_factory):
    rig = foc_rig(rig_factory, board={"torque_control": 0})
    readiness = rig.listener.connection[-1].readiness
    assert not readiness.can_arm
    assert "voltage" in readiness.blocked
    rig.worker.arm(ControlMode.TORQUE)
    rig.run(0.1)
    assert not rig.worker.armed
    assert any("doesn't control current" in text for _, text in rig.listener.messages)


# --- Measuring -----------------------------------------------------------------------------------

def test_stall_test_and_settings_restored(rig_factory):
    rig = foc_rig(rig_factory, SimActuator(scenario="wall", wall_angle_deg=4.0))
    test = StallTest(StallTestParams(target_torque_out=3.0, ramp_time_s=1.0, hold_time_s=0.5, ramp_down_time_s=0.2))
    status = rig.run_procedure(test)
    assert status.success, status.message
    assert status.result["peak_abs_torque_Nm"] == pytest.approx(3.0, rel=0.05)
    assert status.result["hold_mean_torque_Nm"] == pytest.approx(3.0, rel=0.03)
    assert not rig.worker.armed
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE
    board = rig.device.board
    rig.worker.disconnect()
    rig.worker.step()
    assert board.current_limit == pytest.approx(2.0)   # SimpleFOC's default, as it was before connecting
    assert board.velocity_limit == pytest.approx(20.0)
    assert board.verbose == 2 and board.decimals == 3


def test_weight_calibration_recovers_true_output_constant(rig_factory):
    actuator = SimActuator(scenario="lever", mass_kg=0.0, arm_length_m=0.3)
    rig = foc_rig(rig_factory, actuator)
    points = []
    for mass in (0.0, 0.5, 1.0):
        rig.worker.configure_simulator(mass_kg=mass)
        sweep = CalibrationSweep(CalibrationSweepParams(mass_kg=mass, arm_length_m=0.3, horizontal_pos=0.0, cycles=2))
        status = rig.run_procedure(sweep)
        assert status.success, status.message
        points.append(status.result["point"])
    assert fit_calibration(points).k_out_nm_per_a == pytest.approx(actuator.true_k_out, rel=0.02)


def test_watchdog_stops_motor_if_app_freezes(rig_factory):
    rig = foc_rig(rig_factory, SimActuator(scenario="locked"))
    rig.worker.arm(ControlMode.TORQUE)
    rig.run(0.2)
    assert rig.worker.armed and rig.device.watchdog_on
    rig.clock.advance(2.0)  # the app stops talking for 2 s
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE
    rig.run(0.5)
    assert not rig.worker.armed
    assert any("WATCHDOG_EXPIRED" in text for _, text in rig.listener.messages)
    rig.worker.clear_errors()
    rig.run(0.3)  # the status is read every 0.2 s
    assert not rig.listener.slow[-1].errors


def test_bench_firmware_trips_a_motor_that_runs_away(rig_factory):
    # SimpleFOC doesn't limit the speed in torque mode; the bench firmware trips at 1.2 x the limit.
    rig = foc_rig(rig_factory, SimActuator(scenario="free"), limits=Limits(max_current=10.0, max_motor_speed=3.0))
    rig.worker.arm(ControlMode.TORQUE)
    rig.worker.set_output_torque(5.0)
    rig.run(1.0)
    assert not rig.worker.armed
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE
    assert any("OVERSPEED" in text for _, text in rig.listener.messages)
    assert abs(rig.listener.column("vel")).max() < 1.2 * 3.0 + 0.1 + 0.3


def test_app_switches_off_a_runaway_motor_on_a_plain_sketch(rig_factory):
    # Without the bench firmware only the app watches the speed, a sample at a time.
    rig = foc_rig(rig_factory, SimActuator(scenario="free"), board={"bench": False},
                  limits=Limits(max_current=10.0, max_motor_speed=3.0))
    rig.worker.arm(ControlMode.TORQUE)
    rig.worker.set_output_torque(5.0)
    rig.run(1.0)
    assert not rig.worker.armed
    assert rig.sim.read("axis0.current_state") == AxisState.IDLE
    assert any("Overspeed" in text for _, text in rig.listener.messages)
    speed = abs(rig.listener.column("vel"))
    first = int((speed > 1.2 * 3.0 + 0.1).argmax())
    # Off by the second sample over the limit: from then on it only coasts down.
    assert int(speed.argmax()) <= first + 2
    assert (speed[first + 2:][1:] <= speed[first + 2:][:-1] + 0.05).all()


# --- A plain SimpleFOC sketch (no bench firmware) --------------------------------------------------

@pytest.mark.parametrize("get_all", [True, False], ids=["MG7", "MG-one-by-one"])
def test_plain_sketch_measures_a_stall(rig_factory, get_all):
    actuator = SimActuator(scenario="wall", wall_angle_deg=4.0)
    rig = foc_rig(rig_factory, actuator, board={"bench": False, "has_get_all": get_all})
    status = rig.listener.connection[-1]
    assert status.info.firmware == "SimpleFOC sketch (Commander)"
    assert not status.info.capabilities.watchdog
    assert any(kind == "warn" and "no watchdog" in text for kind, text in status.readiness.checks)
    test = StallTest(StallTestParams(target_torque_out=2.0, ramp_time_s=1.0, hold_time_s=0.5, ramp_down_time_s=0.2))
    result = rig.run_procedure(test)
    assert result.success, result.message
    assert result.result["peak_abs_torque_Nm"] == pytest.approx(2.0, rel=0.05)
    assert math.isnan(rig.listener.slow[-1].vbus)


def test_plain_sketch_cant_run_the_thermal_test(rig_factory):
    rig = foc_rig(rig_factory, SimActuator(scenario="wall"), board={"bench": False})
    rig.run(0.3)
    status = rig.run_procedure(ThermalTest(ThermalTestParams(current_a=5.0)))
    assert not status.success
    assert "doesn't report a temperature" in status.message
    assert not rig.worker.armed


# --- Simulation-model tests ------------------------------------------------------------------------

def true_model(actuator: SimActuator) -> TorqueModel:
    return TorqueModel(kt_motor=actuator.kt_nominal, gear_ratio=actuator.gear_ratio,
                       calibrated_k_out=actuator.true_k_out)


def test_speed_test_uses_the_boards_voltage_limit(rig_factory):
    actuator = SimActuator(scenario="free")
    rig = foc_rig(rig_factory, actuator, model=true_model(actuator),
                  limits=Limits(max_current=15.0, max_motor_speed=40.0))
    status = rig.run_procedure(SpeedTest(SpeedTestParams(ramp_s=3.0, hold_s=0.5)), timeout=60)
    assert status.success, status.message
    r = status.result
    # SVPWM on 24 V could give 13.9 V per phase; the 11 V voltage limit comes first.
    assert r["phase_voltage_limit_V"] == pytest.approx(11.0)
    assert r["phase_voltage_per_bus_volt"] == pytest.approx(1 / math.sqrt(3))
    assert r["zero_torque_speed_rad_s"] == pytest.approx(11.0 / actuator.flux / actuator.gear_ratio, rel=0.03)
    assert r["motor"]["kt_backemf_Nm_per_A"] == pytest.approx(actuator.kt_true, rel=0.03)
    assert r["pole_pairs"] == actuator.pole_pairs and r["pole_pairs_source"] == "controller"


def test_speed_test_takes_motor_values_from_setup(rig_factory):
    # A plain sketch knows R and L (MR, MI) but not the pole pairs: those come from Setup.
    actuator = SimActuator(scenario="free", pole_pairs=11)
    rig = foc_rig(rig_factory, actuator, model=true_model(actuator), board={"bench": False},
                  limits=Limits(max_current=15.0, max_motor_speed=40.0))
    test = SpeedTest(SpeedTestParams(ramp_s=3.0, hold_s=0.5, both_directions=False, pole_pairs=11))
    status = rig.run_procedure(test, timeout=60)
    assert status.result["pole_pairs"] == 11 and status.result["pole_pairs_source"] == "Setup"
    # No supply voltage from a plain sketch: the back-EMF can't be worked out, and it says so.
    assert not status.success
    assert "doesn't report its supply voltage" in status.message


def test_friction_test(rig_factory):
    actuator = SimActuator(scenario="free", coulomb_friction=0.3, viscous_friction=0.05)
    rig = foc_rig(rig_factory, actuator, model=true_model(actuator),
                  limits=Limits(max_current=15.0, max_motor_speed=40.0))
    status = rig.run_procedure(FrictionTest(FrictionTestParams(travel_deg=360.0, top_speed=8.0, levels=5)), timeout=120)
    assert status.success, status.message
    assert status.result["coulomb_Nm"] == pytest.approx(0.3, rel=0.15)
    assert status.result["viscous_Nms_per_rad"] == pytest.approx(0.05, rel=0.2)
