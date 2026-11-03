"""Automated measurements, run sample by sample inside the acquisition worker.

* StallTest: the arm is pressed against a fixed object; torque is ramped up to a target, held
  briefly and ramped down. Reports the peak (stall) torque. Aborts if the arm moves.
* HoldTest: the motor holds its position while a load is applied; reports the peak holding torque.
* CalibrationSweep: one point of the known-weight calibration (see calibration.py).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .calibration import analyze_sweep
from .device import SlowReading
from .odrive_api import AxisState, ControlMode
from .samples import Limits, Sample
from .torque import TorqueModel, TorqueStats


class ProcedureError(Exception):
    """A procedure can't start (bad parameters, limits, state)."""


@dataclass
class ProcedureStatus:
    name: str
    phase: str = "starting"
    progress: float = 0.0
    message: str = ""
    done: bool = False
    success: bool = False
    result: dict | None = None


class ProcedureContext(Protocol):
    model: TorqueModel
    limits: Limits
    vel_gain: float | None
    latest_slow: SlowReading | None
    armed: bool
    armed_mode: ControlMode | None

    def arm(self, mode: ControlMode, *, vel_limit: float | None = None, max_current: float | None = None,
            torque_mode_vel_limit: bool = True) -> None: ...

    def set_torque_out(self, torque_out: float) -> None: ...

    def set_position_turns(self, turns: float, vel_ff: float = 0.0) -> None: ...

    def describe_errors(self) -> str: ...


def _fet_temp(ctx: ProcedureContext) -> float:
    return ctx.latest_slow.fet_temp if ctx.latest_slow else math.nan


class Procedure:
    name = "Procedure"
    keep_armed = False  # stay in closed loop when finished (calibration keeps holding the arm)

    def __init__(self) -> None:
        self.status = ProcedureStatus(self.name)
        self.t_start: float | None = None

    def begin(self, ctx: ProcedureContext, sample: Sample) -> None:
        self.t_start = sample.t
        self.start(ctx, sample)

    def elapsed(self, sample: Sample) -> float:
        return 0.0 if self.t_start is None else sample.t - self.t_start

    def start(self, ctx: ProcedureContext, sample: Sample) -> None:
        raise NotImplementedError

    def update(self, ctx: ProcedureContext, sample: Sample) -> None:
        raise NotImplementedError

    def stop(self, ctx: ProcedureContext) -> None:
        """The user pressed Stop on the procedure (not the emergency STOP)."""
        self.abort(ctx, "Stopped by user.")

    def abort(self, ctx: ProcedureContext, reason: str) -> None:
        self.finish(False, reason, self.result(ctx))

    def finish(self, success: bool, message: str, result: dict | None = None) -> None:
        self.status.done = True
        self.status.success = success
        self.status.message = message
        self.status.result = result
        if success:
            self.status.progress = 1.0

    def result(self, ctx: ProcedureContext) -> dict | None:
        return None

    @staticmethod
    def check_running(ctx: ProcedureContext, sample: Sample) -> str | None:
        state = int(sample.state) if sample.state is not None else -1
        if state >= 0 and state != AxisState.CLOSED_LOOP_CONTROL:  # negative = not known yet
            return "The ODrive switched the motor off." + ctx.describe_errors()
        return None


# --- Stall test --------------------------------------------------------------------------------

@dataclass
class StallTestParams:
    target_torque_out: float          # N*m at the output; the sign picks the direction
    ramp_time_s: float = 3.0
    hold_time_s: float = 1.0
    ramp_down_time_s: float = 0.5
    max_movement_deg: float = 10.0    # output degrees; more means the arm isn't blocked


class StallTest(Procedure):
    name = "Stall test"

    def __init__(self, params: StallTestParams):
        super().__init__()
        self.p = params
        self.series: dict[str, list[float]] = {"t": [], "torque_cmd": [], "torque_meas": [], "iq": []}
        self._smooth: deque[float] = deque(maxlen=5)
        self.peak = 0.0
        self.peak_current = 0.0
        self.peak_cmd = 0.0
        self.max_movement = 0.0
        self.hold_torque: list[float] = []
        self.hold_current: list[float] = []
        self.pos0 = 0.0
        self.fet_start = math.nan
        self.vel_limit_used = math.nan
        self._last_cmd = 0.0

    @property
    def total_time(self) -> float:
        return self.p.ramp_time_s + self.p.hold_time_s + self.p.ramp_down_time_s

    def start(self, ctx: ProcedureContext, sample: Sample) -> None:
        p = self.p
        if p.target_torque_out == 0 or not math.isfinite(p.target_torque_out):
            raise ProcedureError("Enter a target torque.")
        if p.ramp_time_s <= 0 or p.hold_time_s < 0 or p.ramp_down_time_s < 0:
            raise ProcedureError("Ramp time must be positive and hold/ramp-down times not negative.")
        if p.max_movement_deg <= 0:
            raise ProcedureError("Allowed movement must be positive.")
        target = abs(p.target_torque_out)
        current = ctx.model.current_for_output_torque(target)
        if current > ctx.limits.max_current * 1.0001:
            raise ProcedureError(
                f"{target:.3g} N·m needs {current:.1f} A, above your {ctx.limits.max_current:.1f} A "
                "current limit. Lower the target or raise the limit in Setup."
            )
        self.pos0 = sample.pos
        self.fet_start = _fet_temp(ctx)
        # In torque mode the ODrive caps torque at standstill to vel_limit * vel_gain, so raise the
        # speed limit enough for the target. The movement check protects against a free arm.
        odrive_target = ctx.model.odrive_input_torque(target)
        vel_limit = ctx.limits.max_motor_speed
        limiter = True
        if ctx.vel_gain and ctx.vel_gain > 0:
            vel_limit = max(vel_limit, 1.3 * odrive_target / ctx.vel_gain)
        else:
            limiter = False
        self.vel_limit_used = vel_limit
        ctx.arm(ControlMode.TORQUE, vel_limit=vel_limit,
                max_current=min(ctx.limits.max_current, current * 1.2 + 0.5),
                torque_mode_vel_limit=limiter)
        self.status.phase = "ramping up"

    def command_at(self, t: float) -> float:
        p = self.p
        if t < p.ramp_time_s:
            return p.target_torque_out * t / p.ramp_time_s
        t -= p.ramp_time_s
        if t < p.hold_time_s:
            return p.target_torque_out
        t -= p.hold_time_s
        if t < p.ramp_down_time_s:
            return p.target_torque_out * (1.0 - t / p.ramp_down_time_s)
        return 0.0

    def phase_at(self, t: float) -> str:
        if t < self.p.ramp_time_s:
            return "ramping up"
        if t < self.p.ramp_time_s + self.p.hold_time_s:
            return "holding"
        return "ramping down"

    def update(self, ctx: ProcedureContext, sample: Sample) -> None:
        t = self.elapsed(sample)
        movement = abs(ctx.model.output_angle_deg(sample.pos - self.pos0))
        self.max_movement = max(self.max_movement, movement)
        s = self.series
        s["t"].append(t)
        s["torque_cmd"].append(self._last_cmd)
        s["torque_meas"].append(sample.torque_out)
        s["iq"].append(sample.iq)

        if movement > self.p.max_movement_deg:
            self.abort(ctx, f"The arm moved {movement:.1f}°, so it isn't blocked. "
                            "Press it firmly against a fixed object and try again.")
            return
        problem = self.check_running(ctx, sample)
        if problem:
            self.abort(ctx, problem)
            return

        self._smooth.append(sample.torque_out)
        smooth = sum(self._smooth) / len(self._smooth)
        if abs(smooth) > abs(self.peak):
            self.peak, self.peak_current, self.peak_cmd = smooth, sample.iq, self._last_cmd
        if self.phase_at(t) == "holding" and t - self.p.ramp_time_s > 0.1:
            self.hold_torque.append(sample.torque_out)
            self.hold_current.append(sample.iq)

        if t >= self.total_time:
            ctx.set_torque_out(0.0)
            self.finish(True, "Stall test complete.", self.result(ctx))
            return
        self._last_cmd = self.command_at(t)
        ctx.set_torque_out(self._last_cmd)
        self.status.phase = self.phase_at(t)
        self.status.progress = min(1.0, t / self.total_time)

    def abort(self, ctx: ProcedureContext, reason: str) -> None:
        try:
            ctx.set_torque_out(0.0)
        finally:
            super().abort(ctx, reason)

    def result(self, ctx: ProcedureContext) -> dict:
        hold = np.array(self.hold_torque)
        return {
            "test": "stall",
            "target_torque_Nm": self.p.target_torque_out,
            "peak_torque_Nm": self.peak,
            "peak_abs_torque_Nm": abs(self.peak),
            "current_at_peak_A": self.peak_current,
            "commanded_at_peak_Nm": self.peak_cmd,
            "hold_mean_torque_Nm": float(hold.mean()) if hold.size else math.nan,
            "hold_mean_current_A": float(np.mean(self.hold_current)) if self.hold_current else math.nan,
            "max_movement_deg": self.max_movement,
            "fet_temp_start_C": self.fet_start,
            "fet_temp_end_C": _fet_temp(ctx),
            "speed_limit_used_motor_turns_s": self.vel_limit_used,
            "ramp_time_s": self.p.ramp_time_s,
            "hold_time_s": self.p.hold_time_s,
            "series": self.series,
        }


# --- Hold test ---------------------------------------------------------------------------------

@dataclass
class HoldTestParams:
    max_torque_out: float             # N*m: torque limit while holding
    duration_s: float = 10.0          # 0 = until the user presses Stop
    max_deflection_deg: float = 30.0  # output degrees


class HoldTest(Procedure):
    name = "Hold test"

    def __init__(self, params: HoldTestParams):
        super().__init__()
        self.p = params
        self.stats = TorqueStats()
        self.series: dict[str, list[float]] = {"t": [], "torque_meas": [], "deflection_deg": []}
        self._smooth: deque[float] = deque(maxlen=5)
        self.peak_pos = 0.0
        self.peak_neg = 0.0
        self.max_deflection = 0.0
        self.pos0 = 0.0
        self.fet_start = math.nan

    def start(self, ctx: ProcedureContext, sample: Sample) -> None:
        p = self.p
        if p.max_torque_out <= 0:
            raise ProcedureError("Enter a positive torque limit.")
        if p.duration_s < 0 or p.max_deflection_deg <= 0:
            raise ProcedureError("Duration can't be negative and the allowed deflection must be positive.")
        current = ctx.model.current_for_output_torque(p.max_torque_out)
        if current > ctx.limits.max_current * 1.0001:
            raise ProcedureError(
                f"{p.max_torque_out:.3g} N·m needs {current:.1f} A, above your "
                f"{ctx.limits.max_current:.1f} A current limit."
            )
        self.pos0 = sample.pos
        self.fet_start = _fet_temp(ctx)
        ctx.arm(ControlMode.POSITION, vel_limit=ctx.limits.max_motor_speed, max_current=current)
        self.status.phase = "holding"

    def update(self, ctx: ProcedureContext, sample: Sample) -> None:
        t = self.elapsed(sample)
        deflection = ctx.model.output_angle_deg(sample.pos - self.pos0)
        self.max_deflection = max(self.max_deflection, abs(deflection))
        self.series["t"].append(t)
        self.series["torque_meas"].append(sample.torque_out)
        self.series["deflection_deg"].append(deflection)
        self.stats.add(np.array([sample.t]), np.array([sample.torque_out]))

        if abs(deflection) > self.p.max_deflection_deg:
            self.abort(ctx, f"The output was pushed {abs(deflection):.1f}° away, beyond the "
                            f"{self.p.max_deflection_deg:.0f}° allowed. The load is stronger than the torque limit.")
            return
        problem = self.check_running(ctx, sample)
        if problem:
            self.abort(ctx, problem)
            return

        self._smooth.append(sample.torque_out)
        smooth = sum(self._smooth) / len(self._smooth)
        self.peak_pos = max(self.peak_pos, smooth)
        self.peak_neg = min(self.peak_neg, smooth)
        if self.p.duration_s > 0:
            self.status.progress = min(1.0, t / self.p.duration_s)
            if t >= self.p.duration_s:
                self.finish(True, "Hold test complete.", self.result(ctx))
        else:
            self.status.progress = 0.0

    def stop(self, ctx: ProcedureContext) -> None:
        self.finish(True, "Stopped by user.", self.result(ctx))

    def steady_torque(self, window_s: float = 1.0) -> float:
        """Mean torque over the last ``window_s``: the steady holding torque without catch-up peaks."""
        t = np.array(self.series["t"])
        if not t.size:
            return math.nan
        torque = np.array(self.series["torque_meas"])
        return float(torque[t >= t[-1] - window_s].mean())

    def result(self, ctx: ProcedureContext) -> dict:
        return {
            "test": "hold",
            "torque_limit_Nm": self.p.max_torque_out,
            "steady_torque_Nm": self.steady_torque(),
            "peak_positive_Nm": self.peak_pos,
            "peak_negative_Nm": self.peak_neg,
            "peak_abs_Nm": max(self.peak_pos, -self.peak_neg),
            "mean_Nm": self.stats.mean,
            "rms_Nm": self.stats.rms,
            "max_deflection_deg": self.max_deflection,
            "duration_s": self.stats.duration,
            "fet_temp_start_C": self.fet_start,
            "fet_temp_end_C": _fet_temp(ctx),
            "series": self.series,
        }


# --- Calibration sweep ------------------------------------------------------------------------

@dataclass
class CalibrationSweepParams:
    mass_kg: float
    arm_length_m: float
    horizontal_pos: float             # motor turns where the arm is horizontal
    amplitude_deg: float = 10.0
    speed_deg_s: float = 10.0
    cycles: int = 2
    window_deg: float = 3.0
    settle_s: float = 0.5


class CalibrationSweep(Procedure):
    name = "Calibration sweep"
    keep_armed = True

    def __init__(self, params: CalibrationSweepParams):
        super().__init__()
        self.p = params
        self.angle: list[float] = []
        self.speed: list[float] = []
        self.iq: list[float] = []
        self.direction: list[int] = []
        self.x0 = 0.0
        self.move_time = 0.0
        self.sweep_time = 0.0

    def start(self, ctx: ProcedureContext, sample: Sample) -> None:
        p = self.p
        if p.arm_length_m <= 0 or p.mass_kg < 0:
            raise ProcedureError("Arm length must be positive and the mass not negative.")
        if p.speed_deg_s <= 0 or p.cycles < 1 or p.amplitude_deg <= p.window_deg:
            raise ProcedureError("Speed must be positive, at least one cycle, and the amplitude larger than the window.")
        self.x0 = ctx.model.output_angle_deg(sample.pos - p.horizontal_pos)
        if abs(self.x0) > 90:
            raise ProcedureError("The arm is more than 90° from the horizontal position you set. Set horizontal again.")
        self.move_time = abs(self.x0) / p.speed_deg_s
        self.sweep_time = p.cycles * 4.0 * p.amplitude_deg / p.speed_deg_s
        sweep_motor_speed = ctx.model.motor_turns_from_output_deg(p.speed_deg_s)
        if 2.0 * sweep_motor_speed > ctx.limits.max_motor_speed:
            raise ProcedureError("The sweep speed is too close to your speed limit. Use a slower sweep "
                                 "or raise the speed limit in Setup.")
        ctx.arm(ControlMode.POSITION, vel_limit=ctx.limits.max_motor_speed, max_current=ctx.limits.max_current)
        self.status.phase = "moving to horizontal"

    @property
    def total_time(self) -> float:
        return self.move_time + self.sweep_time + self.p.settle_s

    def reference(self, t: float) -> tuple[float, float, int, str]:
        """Target angle [deg from horizontal], speed [deg/s], sweep direction and phase name."""
        p = self.p
        v, a = p.speed_deg_s, p.amplitude_deg
        if t < self.move_time:
            step = -math.copysign(v, self.x0)
            return self.x0 + step * t, step, 0, "moving to horizontal"
        t -= self.move_time
        if t < self.sweep_time:
            tau = t % (4.0 * a / v)
            if tau < a / v:
                return v * tau, v, 1, "sweeping"
            if tau < 3.0 * a / v:
                return a - v * (tau - a / v), -v, -1, "sweeping"
            return -a + v * (tau - 3.0 * a / v), v, 1, "sweeping"
        return 0.0, 0.0, 0, "settling"

    def update(self, ctx: ProcedureContext, sample: Sample) -> None:
        t = self.elapsed(sample)
        problem = self.check_running(ctx, sample)
        if problem:
            self.abort(ctx, problem)
            return
        target, speed, direction, phase = self.reference(t)
        angle = ctx.model.output_angle_deg(sample.pos - self.p.horizontal_pos)
        if direction:
            self.angle.append(angle)
            self.speed.append(ctx.model.output_speed_deg_s(sample.vel))
            self.iq.append(sample.iq)
            self.direction.append(direction)
        if abs(angle - target) > 15.0:
            self.abort(ctx, "The arm can't follow the sweep. Is the weight too heavy for the current limit?")
            return
        if t >= self.total_time:
            self._complete(ctx)
            return
        turns = self.p.horizontal_pos + ctx.model.motor_turns_from_output_deg(target)
        ctx.set_position_turns(turns, ctx.model.motor_turns_from_output_deg(speed))
        self.status.phase = phase
        self.status.progress = min(1.0, t / self.total_time)

    def _complete(self, ctx: ProcedureContext) -> None:
        ctx.set_position_turns(self.p.horizontal_pos, 0.0)
        try:
            point = analyze_sweep(
                np.array(self.angle), np.array(self.speed), np.array(self.iq), np.array(self.direction),
                mass_kg=self.p.mass_kg, arm_length_m=self.p.arm_length_m,
                window_deg=self.p.window_deg, sweep_speed_deg_s=self.p.speed_deg_s,
            )
        except ValueError as exc:
            self.finish(False, str(exc), None)
            return
        self.finish(True, f"Measured {point.mass_kg:g} kg: {point.iq_static_a:.3f} A.", {
            "test": "calibration_point",
            "point": point,
        })

    def stop(self, ctx: ProcedureContext) -> None:
        self.finish(False, "Stopped by user (the motor keeps holding the arm).", None)
