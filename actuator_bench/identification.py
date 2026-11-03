"""Fits that turn bench measurements into the values a robot simulation needs.

All values are at the actuator's output (after the gearbox) unless a name says motor.

* Friction: the output turns at a steady speed one way, then back over the same angle. Half the
  difference of the two torques is friction (gravity and current-sensor offsets cancel), fitted as
  Coulomb plus viscous: MuJoCo's joint ``frictionloss`` and ``damping``.
* Inertia: the output swings sinusoidally at a few frequencies. Over whole cycles, the torque in
  phase with the speed is friction, and the torque in quadrature is inertia (and any spring, such
  as gravity on an unbalanced output): Im(T / Ω) = ω J − K / ω. No differentiation of noisy speed
  is needed. J is MuJoCo's joint ``armature``.
* Speed: the no-load speed gives the motor's back-EMF constant; with the phase resistance and
  inductance the ODrive measured, that gives the torque-speed limit at any battery voltage.
* Backlash: torque cycled on a clamped output. The loaded parts of the loop give the gearbox
  stiffness; the gap between them at zero torque is the backlash.
* Thermal: temperature at constant current fitted to T0 + ΔT (1 − e^(−t/τ)). The continuous
  current is the one whose temperature settles at the limit.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

import numpy as np

from .odrive_api import MAX_PHASE_VOLTAGE_PER_BUS_VOLT
from .torque import KT_PER_KV

TWO_PI = 2.0 * math.pi


# --- Friction ----------------------------------------------------------------------------------

@dataclass
class FrictionLevel:
    speed: float              # output rad/s
    torque_forward: float     # mean torque moving forward [N*m]
    torque_backward: float    # mean torque moving backward [N*m]
    samples_forward: int
    samples_backward: int

    @property
    def friction(self) -> float:
        return (self.torque_forward - self.torque_backward) / 2.0

    @property
    def bias(self) -> float:
        """Torque that doesn't depend on the direction (gravity on the output, current offset)."""
        return (self.torque_forward + self.torque_backward) / 2.0

    def to_dict(self) -> dict:
        return {**asdict(self), "friction": self.friction, "bias": self.bias}


@dataclass
class FrictionFit:
    coulomb: float            # N*m: friction at any speed (MuJoCo frictionloss)
    viscous: float            # N*m per rad/s (MuJoCo damping)
    bias: float               # N*m, cancelled out
    r_squared: float
    levels: list[FrictionLevel] = field(default_factory=list)

    def friction_at(self, speed: float) -> float:
        return self.coulomb + self.viscous * abs(speed)

    def to_dict(self) -> dict:
        return {"coulomb_Nm": self.coulomb, "viscous_Nms_per_rad": self.viscous, "bias_Nm": self.bias,
                "r_squared": self.r_squared, "levels": [level.to_dict() for level in self.levels]}


def fit_friction(levels: list[FrictionLevel]) -> FrictionFit:
    """Coulomb + viscous friction through the measured speeds (viscous never below zero)."""
    if len({round(level.speed, 9) for level in levels}) < 2:
        raise ValueError("Friction needs at least two different speeds with steady samples.")
    speed = np.array([level.speed for level in levels])
    friction = np.array([level.friction for level in levels])
    viscous, coulomb = np.polyfit(speed, friction, 1)
    if viscous < 0:  # friction that falls with speed: keep the average, a simulator can't use negative damping
        viscous, coulomb = 0.0, float(friction.mean())
    predicted = coulomb + viscous * speed
    ss_res = float(((friction - predicted) ** 2).sum())
    ss_tot = float(((friction - friction.mean()) ** 2).sum())
    return FrictionFit(
        coulomb=float(coulomb),
        viscous=float(viscous),
        bias=float(np.mean([level.bias for level in levels])),
        r_squared=1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0,
        levels=list(levels),
    )


# --- Inertia -----------------------------------------------------------------------------------

def phasor(t: np.ndarray, x: np.ndarray, frequency: float, harmonics: int = 9) -> complex:
    """Complex amplitude X of x(t) ≈ Re(X e^(jωt)) + offset, by least squares (any sample spacing).

    Harmonics up to ``harmonics`` are fitted alongside, so friction's square-wave shape can't leak
    into the fundamental when the samples aren't evenly spaced.
    """
    w = TWO_PI * frequency
    t = np.asarray(t, dtype=float)
    columns = [np.cos(w * t), np.sin(w * t), np.ones_like(t)]
    for k in range(2, harmonics + 1):
        columns += [np.cos(k * w * t), np.sin(k * w * t)]
    coefficients, *_ = np.linalg.lstsq(np.column_stack(columns), np.asarray(x, dtype=float), rcond=None)
    return complex(coefficients[0], -coefficients[1])


@dataclass
class SwingResult:
    frequency: float          # Hz
    speed_amplitude: float    # output rad/s
    torque_amplitude: float   # N*m
    apparent_inertia: float   # Im(T/Ω)/ω: inertia plus any spring's share at this frequency
    damping: float            # Re(T/Ω): friction as an equivalent damping at this amplitude
    samples: int


@dataclass
class InertiaFit:
    inertia: float            # kg*m^2 at the output, everything that turns with the motor
    spring: float             # N*m/rad: position-proportional load (gravity on the output, cables)
    damping: float            # N*m*s/rad: friction expressed as damping at the test amplitude
    spread: float             # relative scatter of the swings around the fit
    swings: list[SwingResult] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {"inertia_kgm2": self.inertia, "spring_Nm_per_rad": self.spring,
                "damping_equivalent_Nms_per_rad": self.damping, "spread": self.spread,
                "swings": [asdict(s) for s in self.swings]}


def analyze_swing(t: np.ndarray, speed: np.ndarray, torque: np.ndarray, frequency: float,
                  torque_t: np.ndarray | None = None) -> SwingResult:
    """One frequency's swing. ``torque_t`` dates the torque samples when they differ from ``t``: a
    torque held between samples belongs to the middle of that interval."""
    speed_phasor = phasor(t, speed, frequency)
    if abs(speed_phasor) < 1e-9:
        raise ValueError("The output didn't move during the swing.")
    torque_phasor = phasor(t if torque_t is None else torque_t, torque, frequency)
    ratio = torque_phasor / speed_phasor
    w = TWO_PI * frequency
    return SwingResult(frequency, abs(speed_phasor), abs(torque_phasor), ratio.imag / w, ratio.real, len(t))


def fit_inertia(swings: list[SwingResult]) -> InertiaFit:
    """Im(T/Ω) = ωJ − K/ω across the frequencies (one frequency: K is taken as zero)."""
    if not swings:
        raise ValueError("No usable swings.")
    w = np.array([TWO_PI * s.frequency for s in swings])
    im = np.array([s.apparent_inertia for s in swings]) * w
    if len({round(s.frequency, 6) for s in swings}) >= 2:
        (inertia, spring), *_ = np.linalg.lstsq(np.column_stack([w, -1.0 / w]), im, rcond=None)
    else:
        inertia, spring = float(np.mean(im / w)), 0.0
    predicted = w * inertia - spring / w
    scale = max(abs(float(inertia)), 1e-12)
    spread = float(np.sqrt(np.mean(((im - predicted) / w) ** 2))) / scale
    return InertiaFit(float(inertia), float(spring), float(np.mean([s.damping for s in swings])), spread, list(swings))


# --- Motor speed -------------------------------------------------------------------------------

@dataclass
class MotorConstants:
    """The motor's electrical model, motor side: V_q = R I + flux ω, V_d = −pp ω L I (phase peak)."""

    flux: float               # back-EMF per motor rad/s [V*s/rad]
    resistance: float         # phase resistance [ohm]
    inductance: float         # phase inductance [H]
    pole_pairs: int

    @property
    def kt(self) -> float:
        """Torque per amp of Iq at the motor (the ODrive's torque_constant convention)."""
        return 1.5 * self.flux

    @property
    def kv(self) -> float:
        return KT_PER_KV / self.kt

    def to_dict(self) -> dict:
        return {"flux_Vs_per_rad": self.flux, "phase_resistance_ohm": self.resistance,
                "phase_inductance_H": self.inductance, "pole_pairs": self.pole_pairs,
                "kt_backemf_Nm_per_A": self.kt, "kv_backemf_rpm_per_V": self.kv}


def phase_voltage(vbus: float, ratio: float = MAX_PHASE_VOLTAGE_PER_BUS_VOLT) -> float:
    """Largest phase voltage (peak) a driver gets from this bus voltage."""
    return ratio * vbus


def motor_from_no_load(motor_speed: float, current: float, vbus: float, resistance: float,
                       inductance: float, pole_pairs: int,
                       ratio: float = MAX_PHASE_VOLTAGE_PER_BUS_VOLT) -> MotorConstants:
    """Back-EMF constant from the speed the motor tops out at (``motor_speed`` in rad/s)."""
    if motor_speed <= 0 or vbus <= 0:
        raise ValueError("Need a positive no-load speed and bus voltage.")
    v = phase_voltage(vbus, ratio)
    current = abs(current)
    cross = pole_pairs * motor_speed * inductance * current
    flux = (math.sqrt(max(v * v - cross * cross, 0.0)) - resistance * current) / motor_speed
    if flux <= 0:
        raise ValueError("The measured resistance and current leave no voltage for the back-EMF.")
    return MotorConstants(flux, resistance, inductance, int(pole_pairs))


def zero_torque_speed(motor: MotorConstants, vbus: float, ratio: float = MAX_PHASE_VOLTAGE_PER_BUS_VOLT) -> float:
    """Motor rad/s where the motor can no longer make any torque."""
    return phase_voltage(vbus, ratio) / motor.flux


def corner_speed(motor: MotorConstants, vbus: float, current: float,
                 ratio: float = MAX_PHASE_VOLTAGE_PER_BUS_VOLT) -> float:
    """Highest motor rad/s at which the motor still gets ``current``: (R I + flux ω)² + (pp ω L I)² = V²."""
    v = phase_voltage(vbus, ratio)
    i = abs(current)
    a = motor.flux ** 2 + (motor.pole_pairs * motor.inductance * i) ** 2
    b = 2.0 * motor.resistance * i * motor.flux
    c = (motor.resistance * i) ** 2 - v * v
    if c >= 0:
        return 0.0
    return (-b + math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a)


def max_current_at(motor: MotorConstants, vbus: float, motor_speed: float,
                   ratio: float = MAX_PHASE_VOLTAGE_PER_BUS_VOLT) -> float:
    """Most current the bus voltage lets through while turning at ``motor_speed`` (rad/s, driving)."""
    v = phase_voltage(vbus, ratio)
    w = abs(motor_speed)
    a = motor.resistance ** 2 + (motor.pole_pairs * w * motor.inductance) ** 2
    b = 2.0 * motor.resistance * motor.flux * w
    c = (motor.flux * w) ** 2 - v * v
    if c >= 0:
        return 0.0
    if a < 1e-18:
        return math.inf
    return (-b + math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a)


def torque_speed_curve(motor: MotorConstants, vbus: float, current_limit: float, k_out: float,
                       gear_ratio: float, points: int = 11,
                       ratio: float = MAX_PHASE_VOLTAGE_PER_BUS_VOLT) -> list[tuple[float, float]]:
    """(output rad/s, most output N*m) from standstill to the zero-torque speed."""
    top = zero_torque_speed(motor, vbus, ratio)
    curve = []
    for i in range(points):
        w = top * i / (points - 1)
        current = min(current_limit, max_current_at(motor, vbus, w, ratio))
        curve.append((w / gear_ratio, k_out * current))
    return curve


# --- Backlash and stiffness --------------------------------------------------------------------

@dataclass
class BacklashFit:
    backlash: float           # rad at the output: play between the two loaded directions
    stiffness: float          # N*m/rad at the output, both directions averaged
    stiffness_forward: float
    stiffness_backward: float
    hysteresis: float         # rad: raw width of the loop around zero torque
    max_torque: float         # N*m reached

    def to_dict(self) -> dict:
        return {"backlash_deg": math.degrees(self.backlash), "stiffness_Nm_per_rad": self.stiffness,
                "stiffness_forward_Nm_per_rad": self.stiffness_forward,
                "stiffness_backward_Nm_per_rad": self.stiffness_backward,
                "hysteresis_deg": math.degrees(self.hysteresis), "max_torque_Nm": self.max_torque}


def fit_backlash(angle: np.ndarray, torque: np.ndarray, rising: np.ndarray | None = None,
                 band: tuple[float, float] = (0.4, 0.85)) -> BacklashFit:
    """``angle`` (output rad) against ``torque`` over full ± torque cycles on a clamped output.

    Friction makes the loop's rising and falling halves run apart, and right after each reversal
    the motor sticks while the torque changes. So each half is fitted on its own, inside ``band``
    (shares of the largest torque) to stay clear of the reversals, and the two are averaged: their
    middle line is where the gearbox sits without friction. ``rising`` says which samples had
    increasing torque (from the command, if known).
    """
    angle = np.asarray(angle, dtype=float)
    torque = np.asarray(torque, dtype=float)
    top = float(np.max(np.abs(torque))) if torque.size else 0.0
    if rising is None:
        rising = np.gradient(np.convolve(torque, np.ones(5) / 5, mode="same")) > 0
    rising = np.asarray(rising, dtype=bool)
    low, high = band[0] * top, band[1] * top
    slopes, intercepts = [], []
    for sign in (1.0, -1.0):
        loaded = (sign * torque > low) & (sign * torque < high)
        side_slopes, side_intercepts = [], []
        for half in (rising, ~rising):
            use = loaded & half
            if use.sum() < 5:
                continue
            slope, intercept = np.polyfit(torque[use], angle[use], 1)
            side_slopes.append(slope)
            side_intercepts.append(intercept)
        if not side_slopes:
            raise ValueError("Not enough samples under load in both directions.")
        slope = float(np.mean(side_slopes))
        if slope <= 0:
            raise ValueError("The angle didn't follow the torque: is the output really clamped?")
        slopes.append(slope)
        intercepts.append(float(np.mean(side_intercepts)))
    near_zero = np.abs(torque) < 0.05 * top
    hysteresis = float(np.ptp(angle[near_zero])) if near_zero.sum() >= 2 else math.nan
    return BacklashFit(
        backlash=max(0.0, intercepts[0] - intercepts[1]),
        stiffness=2.0 / (slopes[0] + slopes[1]),
        stiffness_forward=1.0 / slopes[0],
        stiffness_backward=1.0 / slopes[1],
        hysteresis=hysteresis,
        max_torque=top,
    )


# --- Thermal -----------------------------------------------------------------------------------

@dataclass
class ThermalFit:
    current: float            # A held during the test
    start_temp: float         # °C
    rise: float               # °C the temperature would settle above the start at this current
    time_constant: float      # s
    duration: float           # s of data
    sensor: str               # "motor" or "driver"

    @property
    def extrapolated(self) -> bool:
        """The test ended before one time constant: the settled temperature is a long extrapolation."""
        return self.duration < self.time_constant

    @property
    def heating_rate(self) -> float:
        """°C per second at the start of the test."""
        return self.rise / self.time_constant

    def continuous_current(self, limit_temp: float, ambient: float | None = None) -> float:
        """Current whose temperature settles exactly at ``limit_temp`` (heat grows with current²)."""
        ambient = self.start_temp if ambient is None else ambient
        if self.rise <= 0 or limit_temp <= ambient:
            return 0.0
        return self.current * math.sqrt((limit_temp - ambient) / self.rise)

    def time_to_limit(self, current: float, limit_temp: float, ambient: float | None = None) -> float:
        """Seconds from ``ambient`` to ``limit_temp`` at ``current`` (inf if it never gets there)."""
        ambient = self.start_temp if ambient is None else ambient
        rise = self.rise * (current / self.current) ** 2
        if rise <= limit_temp - ambient:
            return math.inf
        return -self.time_constant * math.log(1.0 - (limit_temp - ambient) / rise)

    def to_dict(self) -> dict:
        return {"test_current_A": self.current, "start_temp_C": self.start_temp, "rise_C": self.rise,
                "time_constant_s": self.time_constant, "duration_s": self.duration, "sensor": self.sensor,
                "heating_rate_C_per_s": self.heating_rate, "extrapolated": self.extrapolated}


def fit_thermal(t: np.ndarray, temp: np.ndarray, current: float, sensor: str = "motor") -> ThermalFit:
    """First-order heating curve through the samples (τ by search, start and rise by least squares)."""
    t = np.asarray(t, dtype=float)
    temp = np.asarray(temp, dtype=float)
    keep = np.isfinite(t) & np.isfinite(temp)
    t, temp = t[keep], temp[keep]
    if t.size < 10 or t[-1] - t[0] < 20.0:
        raise ValueError("Need at least 20 s of temperature readings.")
    t = t - t[0]
    if float(np.ptp(temp)) < 1.0:
        raise ValueError("The temperature hardly changed. Hold a higher current or run longer.")
    best = None
    for tau in np.geomspace(5.0, 50.0 * max(t[-1], 1.0), 400):
        x = 1.0 - np.exp(-t / tau)
        basis = np.column_stack([np.ones_like(x), x])
        (start, rise), *_ = np.linalg.lstsq(basis, temp, rcond=None)
        if rise <= 0:
            continue
        error = float(((basis @ np.array([start, rise]) - temp) ** 2).sum())
        if best is None or error < best[0]:
            best = (error, float(start), float(rise), float(tau))
    if best is None:
        raise ValueError("The temperature didn't rise.")
    _, start, rise, tau = best
    return ThermalFit(current=abs(current), start_temp=start, rise=rise, time_constant=tau,
                      duration=float(t[-1]), sensor=sensor)
