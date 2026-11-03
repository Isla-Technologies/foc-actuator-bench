"""The fits recover known values from synthetic measurements."""

import math

import numpy as np
import pytest

from actuator_bench.identification import (
    FrictionLevel,
    MotorConstants,
    analyze_swing,
    corner_speed,
    fit_backlash,
    fit_friction,
    fit_inertia,
    fit_thermal,
    max_current_at,
    motor_from_no_load,
    phasor,
    torque_speed_curve,
    zero_torque_speed,
)
from actuator_bench.odrive_api import MAX_PHASE_VOLTAGE_PER_BUS_VOLT


def test_friction_fit_separates_coulomb_viscous_and_gravity():
    coulomb, viscous, gravity = 0.8, 0.12, 2.0
    levels = [FrictionLevel(s, gravity + coulomb + viscous * s, gravity - coulomb - viscous * s, 50, 50)
              for s in (0.2, 0.5, 1.0, 2.0, 4.0)]
    fit = fit_friction(levels)
    assert fit.coulomb == pytest.approx(coulomb)
    assert fit.viscous == pytest.approx(viscous)
    assert fit.bias == pytest.approx(gravity)
    assert fit.r_squared == pytest.approx(1.0)


def test_friction_that_falls_with_speed_never_gives_negative_damping():
    levels = [FrictionLevel(s, 1.0 - 0.1 * s, -(1.0 - 0.1 * s), 20, 20) for s in (0.5, 1.0, 2.0)]
    fit = fit_friction(levels)
    assert fit.viscous == 0.0
    assert fit.coulomb == pytest.approx(np.mean([1.0 - 0.1 * s for s in (0.5, 1.0, 2.0)]))


def test_friction_needs_two_speeds():
    with pytest.raises(ValueError):
        fit_friction([FrictionLevel(1.0, 1.0, -1.0, 10, 10)])


def test_phasor_on_irregular_samples():
    rng = np.random.default_rng(1)
    t = np.sort(rng.uniform(0, 4, 400))
    x = 2.0 * np.cos(2 * np.pi * 1.5 * t + 0.4) + 0.3
    assert phasor(t, x, 1.5) == pytest.approx(2.0 * np.exp(0.4j), abs=1e-9)


def swing(frequency, inertia, spring, coulomb, viscous, amplitude=0.3, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    w = 2 * np.pi * frequency
    t = np.sort(rng.uniform(0, 4 / frequency, 600))
    angle = amplitude * np.sin(w * t)
    speed = amplitude * w * np.cos(w * t)
    accel = -amplitude * w * w * np.sin(w * t)
    torque = inertia * accel + spring * angle + coulomb * np.sign(speed) + viscous * speed
    return t, speed + rng.normal(0, noise * amplitude * w, t.size), torque + rng.normal(0, noise, t.size)


def test_inertia_fit_ignores_friction_and_removes_a_spring():
    swings = [analyze_swing(*swing(f, 0.1, 3.0, 1.5, 0.2, noise=0.05, seed=i), f) for i, f in enumerate((1.0, 1.5, 2.25))]
    fit = fit_inertia(swings)
    assert fit.inertia == pytest.approx(0.1, rel=0.03)
    assert fit.spring == pytest.approx(3.0, rel=0.15)
    assert fit.spread < 0.05
    # One frequency alone can't tell a spring from inertia; without a spring it still finds the inertia.
    alone = fit_inertia([analyze_swing(*swing(2.0, 0.05, 0.0, 1.0, 0.0), 2.0)])
    assert alone.inertia == pytest.approx(0.05, rel=0.02)


def test_motor_constants_round_trip():
    motor = MotorConstants(flux=0.06, resistance=0.1, inductance=4e-5, pole_pairs=14)
    vbus = 22.2
    v = MAX_PHASE_VOLTAGE_PER_BUS_VOLT * vbus
    assert zero_torque_speed(motor, vbus) == pytest.approx(v / 0.06)
    # A motor at no load with 1 A of friction current tops out where the voltage runs out.
    speed = 0.0
    for _ in range(100):  # fixed point: (R I + flux w)^2 + (pp w L I)^2 = V^2
        speed = (math.sqrt(v * v - (14 * speed * 4e-5 * 1.0) ** 2) - 0.1) / 0.06
    measured = motor_from_no_load(speed, 1.0, vbus, 0.1, 4e-5, 14)
    assert measured.flux == pytest.approx(0.06, rel=1e-6)
    assert measured.kt == pytest.approx(0.09)
    # The corner speed at 20 A is where 20 A is the most the voltage allows.
    corner = corner_speed(motor, vbus, 20.0)
    assert max_current_at(motor, vbus, corner) == pytest.approx(20.0, rel=1e-6)
    assert 0 < corner < zero_torque_speed(motor, vbus)
    curve = torque_speed_curve(motor, vbus, 20.0, k_out=3.0, gear_ratio=36.0)
    assert curve[0] == (0.0, pytest.approx(60.0))
    assert curve[-1][1] == pytest.approx(0.0, abs=1e-6)
    assert curve[-1][0] == pytest.approx(v / 0.06 / 36.0)


def test_corner_speed_is_zero_when_resistance_eats_the_voltage():
    motor = MotorConstants(flux=0.06, resistance=1.0, inductance=0.0, pole_pairs=7)
    assert corner_speed(motor, 10.0, 20.0) == 0.0


def test_backlash_and_stiffness_from_a_torque_cycle():
    play, stiffness = math.radians(0.3), 2500.0
    torque = 20.0 * (2 / np.pi) * np.arcsin(np.sin(np.linspace(0, 4 * np.pi, 2000)))
    angle = np.where(torque >= 0, play / 2, -play / 2) + torque / stiffness
    fit = fit_backlash(angle, torque)
    assert math.degrees(fit.backlash) == pytest.approx(0.3, rel=1e-3)
    assert fit.stiffness == pytest.approx(stiffness, rel=1e-3)
    assert fit.max_torque == pytest.approx(20.0, rel=1e-3)


def test_backlash_refuses_an_output_that_isnt_clamped():
    torque = np.linspace(-5, 5, 200)
    with pytest.raises(ValueError):
        fit_backlash(np.zeros_like(torque) - torque, torque)


def test_thermal_fit_and_ratings():
    t = np.arange(0, 900, 2.0)
    temp = 25.0 + 40.0 * (1 - np.exp(-t / 300.0))
    fit = fit_thermal(t, temp, current=10.0)
    assert fit.rise == pytest.approx(40.0, rel=0.01)
    assert fit.time_constant == pytest.approx(300.0, rel=0.02)
    assert not fit.extrapolated
    # 20 °C of headroom at 10 A gives 40 °C; heat grows with current squared.
    assert fit.continuous_current(45.0, ambient=25.0) == pytest.approx(10.0 * math.sqrt(0.5), rel=0.01)
    assert math.isinf(fit.time_to_limit(5.0, 80.0, ambient=25.0))
    assert fit.time_to_limit(20.0, 80.0, ambient=25.0) == pytest.approx(-300 * math.log(1 - 55 / 160), rel=0.02)


def test_thermal_fit_flags_a_short_test():
    t = np.arange(0, 60, 1.0)
    fit = fit_thermal(t, 25.0 + 40.0 * (1 - np.exp(-t / 300.0)), current=10.0)
    assert fit.extrapolated
    assert fit.heating_rate == pytest.approx(40 / 300, rel=0.05)
    with pytest.raises(ValueError):
        fit_thermal(t, np.full_like(t, 25.0), current=10.0)


def test_friction_fit_says_how_sure_it_is():
    rng = np.random.default_rng(4)
    levels = []
    for s in (0.2, 0.5, 1.0, 2.0, 4.0):
        f = 0.8 + 0.12 * s
        levels.append(FrictionLevel(s, f + rng.normal(0, 0.02), -f + rng.normal(0, 0.02), 40, 40, 0.02, 0.02))
    fit = fit_friction(levels)
    assert 0 < fit.coulomb_error < 0.05
    assert abs(fit.coulomb - 0.8) < 3 * fit.coulomb_error
    assert abs(fit.viscous - 0.12) < 3 * fit.viscous_error
