import numpy as np
import pytest

from actuator_bench.calibration import G, CalibrationPoint, analyze_sweep, fit_calibration


def synthetic_sweep(k_out, mass, arm, friction_a=0.4, offset_a=0.2, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    angle = np.concatenate([np.linspace(-10, 10, 200), np.linspace(10, -10, 200)])
    direction = np.concatenate([np.ones(200), -np.ones(200)])
    speed = direction * 10.0
    iq = mass * G * arm / k_out + offset_a + direction * friction_a + rng.normal(0, noise, angle.size)
    return angle, speed, iq, direction


def test_analyze_sweep_cancels_friction():
    angle, speed, iq, direction = synthetic_sweep(0.7, 1.0, 0.3)
    point = analyze_sweep(angle, speed, iq, direction, mass_kg=1.0, arm_length_m=0.3)
    assert point.torque_ref_nm == pytest.approx(G * 0.3)
    assert point.iq_static_a == pytest.approx(G * 0.3 / 0.7 + 0.2)
    assert point.iq_friction_a == pytest.approx(0.4)


def test_analyze_sweep_needs_samples_in_both_directions():
    angle, speed, iq, direction = synthetic_sweep(0.7, 1.0, 0.3)
    with pytest.raises(ValueError):
        analyze_sweep(angle[:200], speed[:200], iq[:200], direction[:200], mass_kg=1.0, arm_length_m=0.3)


def test_fit_recovers_constant_and_offset():
    points = []
    for mass in (0.0, 0.5, 1.0, 2.0):
        angle, speed, iq, direction = synthetic_sweep(0.7, mass, 0.25, noise=0.05, seed=int(mass * 10))
        points.append(analyze_sweep(angle, speed, iq, direction, mass_kg=mass, arm_length_m=0.25))
    fit = fit_calibration(points)
    assert fit.k_out_nm_per_a == pytest.approx(0.7, rel=0.01)
    assert fit.offset_a == pytest.approx(0.2, abs=0.02)
    assert fit.r_squared > 0.999
    assert fit.implied_efficiency(kt_motor=0.0827, gear_ratio=9) == pytest.approx(0.7 / (0.0827 * 9), rel=0.01)


def test_fit_needs_two_masses():
    p = CalibrationPoint(1.0, 0.3, G * 0.3, 4.0, 0.1, 4.1, 3.9, 10, 10)
    with pytest.raises(ValueError):
        fit_calibration([p, p])
