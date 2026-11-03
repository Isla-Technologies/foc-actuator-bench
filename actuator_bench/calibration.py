"""Known-weight calibration: measure the real output torque per amp of the actuator.

A lever arm of length r carries a mass m, so gravity applies a known torque m*g*r at the output
when the arm is horizontal. The actuator sweeps the arm slowly up and down through horizontal;
averaging the current needed going up and going down cancels friction. Repeating this for a few
masses and fitting a straight line gives K_cal = output torque per amp of Iq, which includes the
real gear ratio, gearbox losses and the true motor torque constant. The line's offset absorbs the
arm's own weight.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

G = 9.80665


@dataclass
class CalibrationPoint:
    mass_kg: float
    arm_length_m: float
    torque_ref_nm: float      # m * g * r
    iq_static_a: float        # average of the up and down currents (friction cancelled)
    iq_friction_a: float      # half the difference: current spent on friction
    iq_up_a: float
    iq_down_a: float
    samples_up: int
    samples_down: int

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class CalibrationFit:
    k_out_nm_per_a: float     # output torque per amp of Iq
    offset_a: float           # current at zero added mass (arm weight, bias)
    r_squared: float
    max_residual_nm: float
    points: list[CalibrationPoint]

    def implied_efficiency(self, kt_motor: float, gear_ratio: float) -> float:
        """K_cal divided by the ideal value Kt * N (lumps gearbox losses and Kt error together)."""
        return self.k_out_nm_per_a / (kt_motor * gear_ratio)

    def to_dict(self) -> dict:
        data = asdict(self)
        data["points"] = [p.to_dict() for p in self.points]
        return data


def analyze_sweep(
    angle_deg: np.ndarray,
    speed_deg_s: np.ndarray,
    iq: np.ndarray,
    direction: np.ndarray,
    *,
    mass_kg: float,
    arm_length_m: float,
    window_deg: float = 3.0,
    min_speed_fraction: float = 0.3,
    sweep_speed_deg_s: float | None = None,
) -> CalibrationPoint:
    """Average the current near horizontal while moving up and while moving down.

    ``angle_deg`` is the output angle relative to horizontal, ``direction`` the commanded
    direction (+1 up, -1 down, 0 not sweeping) for each sample.
    """
    angle_deg = np.asarray(angle_deg, dtype=float)
    speed_deg_s = np.asarray(speed_deg_s, dtype=float)
    iq = np.asarray(iq, dtype=float)
    direction = np.asarray(direction, dtype=float)
    if sweep_speed_deg_s is None:
        moving = np.abs(speed_deg_s[direction != 0])
        sweep_speed_deg_s = float(np.median(moving)) if moving.size else 0.0
    min_speed = min_speed_fraction * abs(sweep_speed_deg_s)

    near = np.abs(angle_deg) <= window_deg
    up = near & (direction > 0) & (speed_deg_s > min_speed)
    down = near & (direction < 0) & (speed_deg_s < -min_speed)
    if up.sum() < 3 or down.sum() < 3:
        raise ValueError(
            "Not enough steady samples near horizontal "
            f"(up: {int(up.sum())}, down: {int(down.sum())}). Use a slower sweep, more cycles or a wider window."
        )
    iq_up = float(iq[up].mean())
    iq_down = float(iq[down].mean())
    return CalibrationPoint(
        mass_kg=mass_kg,
        arm_length_m=arm_length_m,
        torque_ref_nm=mass_kg * G * arm_length_m,
        iq_static_a=(iq_up + iq_down) / 2.0,
        iq_friction_a=(iq_up - iq_down) / 2.0,
        iq_up_a=iq_up,
        iq_down_a=iq_down,
        samples_up=int(up.sum()),
        samples_down=int(down.sum()),
    )


def fit_calibration(points: list[CalibrationPoint]) -> CalibrationFit:
    """Straight-line fit of current against reference torque: Iq = torque / K_cal + offset."""
    masses = {round(p.mass_kg, 6) for p in points}
    if len(masses) < 2:
        raise ValueError("Measure at least two different masses (for example 0 kg and 1 kg).")
    torque = np.array([p.torque_ref_nm for p in points])
    current = np.array([p.iq_static_a for p in points])
    slope, offset = np.polyfit(torque, current, 1)
    if not math.isfinite(slope) or abs(slope) < 1e-9:
        raise ValueError("The current did not change with the mass; check the setup.")
    k_out = 1.0 / abs(slope)
    predicted = slope * torque + offset
    ss_res = float(((current - predicted) ** 2).sum())
    ss_tot = float(((current - current.mean()) ** 2).sum())
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    residual_nm = np.abs(current - predicted) * k_out
    return CalibrationFit(
        k_out_nm_per_a=float(k_out),
        offset_a=float(offset),
        r_squared=float(r_squared),
        max_residual_nm=float(residual_nm.max()),
        points=list(points),
    )
