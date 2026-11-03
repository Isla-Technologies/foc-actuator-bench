import math

import numpy as np
import pytest

from actuator_bench.torque import (
    TORQUE_UNITS,
    LowPass,
    TorqueModel,
    TorqueStats,
    convert_torque,
    kt_from_kv,
    kv_from_kt,
)


def test_kt_from_kv_matches_odrive_formula():
    assert kt_from_kv(100) == pytest.approx(0.0827)
    assert kt_from_kv(270) == pytest.approx(8.27 / 270)
    assert kv_from_kt(kt_from_kv(150)) == pytest.approx(150)
    with pytest.raises(ValueError):
        kt_from_kv(0)


def test_output_torque_includes_gear_ratio_and_efficiency():
    model = TorqueModel(kt_motor=0.08, gear_ratio=9.0, efficiency=0.9)
    assert model.motor_torque(10.0) == pytest.approx(0.8)
    assert model.output_torque(10.0) == pytest.approx(0.08 * 9 * 0.9 * 10)
    assert model.max_output_torque(5.0) == pytest.approx(0.08 * 9 * 0.9 * 5)


def test_calibration_overrides_nominal_constant():
    model = TorqueModel(kt_motor=0.08, gear_ratio=9.0, efficiency=1.0, calibrated_k_out=0.65)
    assert model.is_calibrated
    assert model.output_torque(2.0) == pytest.approx(1.3)
    assert model.current_for_output_torque(1.3) == pytest.approx(2.0)


def test_command_scaling_compensates_odrive_torque_constant():
    # The ODrive turns input_torque into current with ITS torque_constant (here the 0.04 default).
    model = TorqueModel(kt_motor=0.0827, gear_ratio=6.0, efficiency=1.0, kt_odrive=0.04)
    target = 3.0
    input_torque = model.odrive_input_torque(target)
    current = input_torque / 0.04
    assert model.output_torque(current) == pytest.approx(target)
    assert model.odrive_torque_limit(10.0) == pytest.approx(0.4)


def test_motor_to_output_conversions():
    model = TorqueModel(kt_motor=0.1, gear_ratio=10.0)
    assert model.output_angle_deg(10.0) == pytest.approx(360.0)
    assert model.motor_turns_from_output_deg(36.0) == pytest.approx(1.0)
    assert model.output_speed_rpm(10.0) == pytest.approx(60.0)
    assert model.motor_turns_per_s_from_output_rpm(60.0) == pytest.approx(10.0)
    assert model.mechanical_power(10.0, 1.0) == pytest.approx(1.0 * 2 * math.pi)


def test_invalid_models_are_rejected():
    with pytest.raises(ValueError):
        TorqueModel(kt_motor=0.0)
    with pytest.raises(ValueError):
        TorqueModel(kt_motor=0.1, gear_ratio=0)
    with pytest.raises(ValueError):
        TorqueModel(kt_motor=0.1, efficiency=1.5)


def test_unit_conversion():
    assert convert_torque(1.0, "N·m") == 1.0
    assert convert_torque(1.0, "kgf·cm") == pytest.approx(10.197, rel=1e-4)
    assert convert_torque(1.0, "lbf·in") == pytest.approx(8.8507, rel=1e-4)
    assert set(TORQUE_UNITS) == {"N·m", "kgf·cm", "lbf·in"}


def test_low_pass_converges_and_handles_gaps():
    lp = LowPass(0.05)
    t = np.arange(0, 1, 0.01)
    y = lp.apply(t, np.ones_like(t))
    assert y[0] == 1.0
    lp.reset()
    y = lp.apply(t, np.where(t < 0.5, 0.0, 1.0))
    assert y[49] == 0.0 and 0.0 < y[52] < 1.0 and y[-1] == pytest.approx(1.0, abs=1e-3)


def test_stats_peak_mean_rms():
    stats = TorqueStats()
    stats.add(np.array([0.0, 1.0, 2.0, 3.0]), np.array([1.0, -3.0, 2.0, np.nan]))
    assert stats.count == 3
    assert stats.peak_pos == 2.0
    assert stats.peak_neg == -3.0
    assert stats.peak_abs == 3.0
    assert stats.mean == pytest.approx(0.0)
    assert stats.rms == pytest.approx(math.sqrt(14 / 3))
    assert stats.duration == 3.0
    stats.reset()
    assert stats.count == 0 and stats.rms == 0.0
