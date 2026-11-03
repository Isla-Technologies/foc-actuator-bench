import pytest

from actuator_bench.odrive_api import AxisPaths, AxisState, decode_errors, state_label


def test_decode_known_flags():
    flags = decode_errors("axis", 0x800 | 0x40)
    assert [f.name for f in flags] == ["MOTOR_FAILED", "WATCHDOG_TIMER_EXPIRED"]
    assert all(f.explanation for f in flags)


def test_decode_unknown_bits_and_zero():
    assert decode_errors("motor", 0) == []
    assert decode_errors("motor", None) == []
    flags = decode_errors("controller", 0x1 | 0x10000)
    assert [f.name for f in flags] == ["OVERSPEED", "UNKNOWN_0x10000"]


def test_large_motor_error_bits():
    flags = decode_errors("motor", 0x800000000)
    assert flags[0].name == "UNBALANCED_PHASES"


def test_axis_paths():
    p = AxisPaths(1)
    assert p.iq_measured == "axis1.motor.current_control.Iq_measured"
    assert p.pos_estimate == "axis1.encoder.pos_estimate"
    assert p.torque_constant == "axis1.motor.config.torque_constant"
    assert p.fast[0] == p.iq_measured
    assert set(p.error_registers) == {"axis", "motor", "encoder", "controller"}
    with pytest.raises(ValueError):
        AxisPaths(2)


def test_state_labels():
    assert "Closed loop" in state_label(AxisState.CLOSED_LOOP_CONTROL)
    assert state_label(None) == "Unknown"
    assert state_label(99) == "State 99"
