"""Torque math: from measured motor current to output torque, plus units, filters and statistics.

The ODrive measures current, not torque. Torque is estimated as::

    motor torque  = Kt * Iq                     (Kt ~= 8.27 / KV  [N*m/A])
    output torque = motor torque * gear ratio * efficiency

or, after a known-weight calibration, ``output torque = K_cal * Iq`` where K_cal (output N*m per
amp) already includes the real gear ratio, gearbox losses and any error in Kt.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Torque constant of a 3-phase BLDC motor from its KV: (3/2) / (sqrt(3) * 2*pi/60) = 8.27.
KT_PER_KV = 8.27

# Multiply N*m by these factors to convert.
TORQUE_UNITS = {
    "N·m": 1.0,
    "kgf·cm": 1.0 / (9.80665 * 0.01),
    "lbf·in": 1.0 / (4.4482216152605 * 0.0254),
}


def kt_from_kv(kv_rpm_per_volt: float) -> float:
    if kv_rpm_per_volt <= 0:
        raise ValueError("KV must be positive")
    return KT_PER_KV / kv_rpm_per_volt


def kv_from_kt(kt_nm_per_amp: float) -> float:
    if kt_nm_per_amp <= 0:
        raise ValueError("Kt must be positive")
    return KT_PER_KV / kt_nm_per_amp


def convert_torque(value_nm, unit: str):
    return value_nm * TORQUE_UNITS[unit]


@dataclass
class TorqueModel:
    """Converts between motor current and torque, and between motor and output units.

    Positions/velocities from the ODrive are motor-side (turns, turns/s); output-side values are
    divided by the gear ratio.
    """

    kt_motor: float                         # N*m/A, used for measurement
    gear_ratio: float = 1.0
    efficiency: float = 1.0                 # 0..1
    calibrated_k_out: float | None = None   # output N*m per amp, from weight calibration
    kt_odrive: float | None = None          # torque_constant configured inside the ODrive

    def __post_init__(self) -> None:
        if self.kt_motor <= 0:
            raise ValueError("Kt must be positive")
        if self.gear_ratio <= 0:
            raise ValueError("Gear ratio must be positive")
        if not 0 < self.efficiency <= 1:
            raise ValueError("Efficiency must be in (0, 1]")

    @property
    def is_calibrated(self) -> bool:
        return bool(self.calibrated_k_out)

    @property
    def k_out(self) -> float:
        """Output torque per amp of Iq [N*m/A]."""
        if self.calibrated_k_out:
            return abs(self.calibrated_k_out)
        return self.kt_motor * self.gear_ratio * self.efficiency

    # current <-> torque
    def motor_torque(self, iq):
        return self.kt_motor * iq

    def output_torque(self, iq):
        return self.k_out * iq

    def current_for_output_torque(self, torque_out):
        return torque_out / self.k_out

    def max_output_torque(self, max_current: float) -> float:
        return self.k_out * max_current

    # commands for the ODrive (which uses its own torque_constant to turn torque into current)
    def odrive_input_torque(self, torque_out: float) -> float:
        """``controller.input_torque`` value that makes the ODrive deliver ``torque_out``."""
        return self.current_for_output_torque(torque_out) * self._kt_device

    def odrive_torque_limit(self, max_current: float) -> float:
        """``motor.config.torque_lim`` value equivalent to ``max_current`` amps."""
        return max_current * self._kt_device

    @property
    def _kt_device(self) -> float:
        return self.kt_odrive if self.kt_odrive and self.kt_odrive > 0 else self.kt_motor

    # motor side <-> output side
    def output_angle_deg(self, motor_turns):
        return motor_turns / self.gear_ratio * 360.0

    def motor_turns_from_output_deg(self, deg):
        return deg / 360.0 * self.gear_ratio

    def output_speed_rpm(self, motor_turns_per_s):
        return motor_turns_per_s / self.gear_ratio * 60.0

    def motor_turns_per_s_from_output_rpm(self, rpm):
        return rpm / 60.0 * self.gear_ratio

    def output_speed_deg_s(self, motor_turns_per_s):
        return motor_turns_per_s / self.gear_ratio * 360.0

    def mechanical_power(self, iq, motor_turns_per_s):
        """Shaft power [W] (motor torque times motor angular speed)."""
        return self.motor_torque(iq) * motor_turns_per_s * 2.0 * math.pi


class LowPass:
    """First-order low-pass filter for irregularly spaced samples."""

    def __init__(self, time_constant_s: float):
        self.time_constant_s = max(0.0, time_constant_s)
        self._y: float | None = None
        self._t: float | None = None

    def reset(self) -> None:
        self._y = None
        self._t = None

    def apply(self, t: np.ndarray, x: np.ndarray) -> np.ndarray:
        out = np.empty(len(x), dtype=float)
        tau = self.time_constant_s
        y, t_prev = self._y, self._t
        for i in range(len(x)):
            xi = float(x[i])
            if y is None or not math.isfinite(y) or tau <= 0:
                y = xi
            else:
                dt = max(0.0, float(t[i]) - t_prev)
                alpha = 1.0 - math.exp(-dt / tau)
                y += alpha * (xi - y)
            t_prev = float(t[i])
            out[i] = y
        self._y, self._t = y, t_prev
        return out


class TorqueStats:
    """Running peak / mean / RMS of a torque signal since the last reset."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.count = 0
        self._sum = 0.0
        self._sum_sq = 0.0
        self.peak_pos = 0.0
        self.peak_neg = 0.0
        self.t_first: float | None = None
        self.t_last: float | None = None

    def add(self, t: np.ndarray, values: np.ndarray) -> None:
        values = np.asarray(values, dtype=float)
        finite = np.isfinite(values)
        if not finite.any():
            return
        v = values[finite]
        self.count += len(v)
        self._sum += float(v.sum())
        self._sum_sq += float((v * v).sum())
        self.peak_pos = max(self.peak_pos, float(v.max()))
        self.peak_neg = min(self.peak_neg, float(v.min()))
        if len(t):
            if self.t_first is None:
                self.t_first = float(t[0])
            self.t_last = float(t[-1])

    @property
    def peak_abs(self) -> float:
        return max(self.peak_pos, -self.peak_neg)

    @property
    def mean(self) -> float:
        return self._sum / self.count if self.count else 0.0

    @property
    def rms(self) -> float:
        return math.sqrt(self._sum_sq / self.count) if self.count else 0.0

    @property
    def duration(self) -> float:
        if self.t_first is None or self.t_last is None:
            return 0.0
        return self.t_last - self.t_first

    def summary(self) -> dict:
        return {
            "samples": self.count,
            "duration_s": round(self.duration, 3),
            "peak_positive_Nm": self.peak_pos,
            "peak_negative_Nm": self.peak_neg,
            "peak_abs_Nm": self.peak_abs,
            "mean_Nm": self.mean,
            "rms_Nm": self.rms,
        }
