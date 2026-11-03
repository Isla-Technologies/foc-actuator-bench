"""Tests that measure what a robot simulation needs to know about the actuator.

They run sample by sample inside the acquisition worker, like the stall and hold tests, and leave
the analysis to identification.py. All results are at the output, after the gearbox.

* FrictionTest: steady speeds back and forth over the same angle → Coulomb and viscous friction.
* InertiaTest: sinusoidal swings at three frequencies → the inertia the motor and gearbox add.
* SpeedTest: the speed is raised until the motor runs out of voltage → no-load speed, back-EMF.
* BacklashTest: torque cycled both ways on a clamped output → backlash and gearbox stiffness.
* ThermalTest: constant current on a blocked output → heating curve and continuous current.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .identification import (
    FrictionFit,
    FrictionLevel,
    InertiaFit,
    analyze_swing,
    fit_backlash,
    fit_friction,
    fit_inertia,
    fit_thermal,
    motor_from_no_load,
    zero_torque_speed,
)
from .odrive_api import ControlMode
from .procedures import Procedure, ProcedureContext, ProcedureError, arm_for_torque
from .samples import Sample

TWO_PI = 2.0 * math.pi


def out_angle(ctx: ProcedureContext, motor_turns: float) -> float:
    """Motor turns → output radians."""
    return motor_turns / ctx.model.gear_ratio * TWO_PI


def out_speed(ctx: ProcedureContext, motor_turns_per_s: float) -> float:
    """Motor turns/s → output rad/s."""
    return motor_turns_per_s / ctx.model.gear_ratio * TWO_PI


def motor_turns(ctx: ProcedureContext, output_rad: float) -> float:
    """Output radians (or rad/s) → motor turns (or turns/s)."""
    return output_rad * ctx.model.gear_ratio / TWO_PI


def _mean(values) -> float:
    return float(np.mean(values)) if len(values) else math.nan


def _standard_error(values) -> float:
    """How far the mean of ``values`` may be off from noise alone."""
    return float(np.std(values, ddof=1) / math.sqrt(len(values))) if len(values) > 1 else math.nan


# --- Friction ----------------------------------------------------------------------------------

@dataclass
class FrictionTestParams:
    travel_deg: float = 180.0      # output degrees covered by each pass
    top_speed: float = 3.0         # output rad/s of the fastest pass
    levels: int = 6                # speeds, spread from top/12 to top
    max_pass_s: float = 4.0        # slow passes stop after this long, before the full travel
    settle_s: float = 0.3          # skipped at the start of each pass


class FrictionTest(Procedure):
    name = "Friction test"

    def __init__(self, params: FrictionTestParams):
        super().__init__()
        self.p = params
        self.speeds: list[float] = []
        self.levels: list[FrictionLevel] = []
        self.skipped: list[float] = []
        self.fit: FrictionFit | None = None
        self.series: dict[str, list[float]] = {"t": [], "speed_rad_s": [], "torque_Nm": []}
        self.pos0 = 0.0
        self._level = 0
        self._dir = 1
        self._pass_t0 = 0.0
        self._torques: dict[int, list[float]] = {1: [], -1: []}
        self._pause_until: float | None = None
        self._total = 1.0

    def start(self, ctx: ProcedureContext, sample: Sample) -> None:
        p = self.p
        if p.travel_deg <= 0 or p.top_speed <= 0 or p.levels < 2 or p.max_pass_s <= p.settle_s:
            raise ProcedureError("Travel and speed must be positive, with at least two speeds.")
        travel = math.radians(p.travel_deg)
        top = min(p.top_speed, 0.95 * out_speed(ctx, ctx.limits.max_motor_speed), travel / (p.settle_s + 0.4))
        self.speeds = [top * (1.0 / 12.0) ** (1.0 - i / (p.levels - 1)) for i in range(p.levels)]
        self._total = sum(2.0 * min(travel / s, p.max_pass_s) + 0.3 for s in self.speeds)
        self.pos0 = sample.pos
        ctx.arm(ControlMode.VELOCITY, vel_limit=ctx.limits.max_motor_speed, max_current=ctx.limits.max_current)
        self._begin_pass(ctx, sample, 1)

    def _begin_pass(self, ctx: ProcedureContext, sample: Sample, direction: int) -> None:
        self._dir = direction
        self._pass_t0 = self.elapsed(sample)
        self._torques[direction] = []
        ctx.set_velocity_turns(direction * motor_turns(ctx, self.speeds[self._level]))
        rpm = self.speeds[self._level] * 60.0 / TWO_PI
        self.status.phase = f"speed {self._level + 1}/{len(self.speeds)}: {rpm:.3g} rpm {'forward' if direction > 0 else 'back'}"

    def update(self, ctx: ProcedureContext, sample: Sample) -> None:
        t = self.elapsed(sample)
        problem = self.check_running(ctx, sample)
        if problem:
            self.abort(ctx, problem)
            return
        angle = out_angle(ctx, sample.pos - self.pos0)
        if abs(angle) > math.radians(self.p.travel_deg + 30.0):
            self.abort(ctx, f"The output went {math.degrees(abs(angle)):.0f}° from the start, past the travel you "
                            "allowed. Is something turning it?")
            return
        speed_now = out_speed(ctx, sample.vel)
        self.series["t"].append(t)
        self.series["speed_rad_s"].append(speed_now)
        self.series["torque_Nm"].append(sample.torque_out)
        self.status.progress = min(0.99, t / self._total)

        if self._pause_until is not None:
            if t < self._pause_until:
                return
            self._pause_until = None
            if self._level >= len(self.speeds):
                self._complete(ctx)
            else:
                self._begin_pass(ctx, sample, 1)
            return

        p = self.p
        speed = self.speeds[self._level]
        t_pass = t - self._pass_t0
        if t_pass >= p.settle_s and abs(speed_now - self._dir * speed) <= 0.05 * speed + 0.01:
            self._torques[self._dir].append(sample.torque_out)
        if self._dir > 0:
            if angle >= math.radians(p.travel_deg) or t_pass >= p.max_pass_s:
                self._begin_pass(ctx, sample, -1)
        elif angle <= 0.0 or t_pass >= p.max_pass_s + 2.0:
            self._end_level(ctx, t)

    def _end_level(self, ctx: ProcedureContext, t: float) -> None:
        forward, backward = self._torques[1], self._torques[-1]
        speed = self.speeds[self._level]
        if len(forward) >= 5 and len(backward) >= 5:
            self.levels.append(FrictionLevel(speed, _mean(forward), _mean(backward), len(forward), len(backward),
                                             _standard_error(forward), _standard_error(backward)))
        else:
            self.skipped.append(speed)
        self._level += 1
        ctx.set_velocity_turns(0.0)
        self._pause_until = t + 0.3

    def _complete(self, ctx: ProcedureContext) -> None:
        ctx.set_velocity_turns(0.0)
        try:
            self.fit = fit_friction(self.levels)
        except ValueError as exc:
            self.finish(False, f"{exc} Allow more travel, or lower the top speed.", self.result(ctx))
            return
        f = self.fit
        self.finish(True, f"Friction {f.coulomb:.3g} N·m plus {f.viscous:.3g} N·m per rad/s.", self.result(ctx))

    def abort(self, ctx: ProcedureContext, reason: str) -> None:
        try:
            ctx.set_velocity_turns(0.0)
        finally:
            super().abort(ctx, reason)

    def result(self, ctx: ProcedureContext) -> dict:
        result = {"test": "friction", "travel_deg": self.p.travel_deg, "speeds_rad_s": self.speeds,
                  "skipped_speeds_rad_s": self.skipped, "series": self.series}
        if self.fit:
            result.update(self.fit.to_dict())
        else:
            result["levels"] = [level.to_dict() for level in self.levels]
        return result


# --- Inertia -----------------------------------------------------------------------------------

FREQUENCY_STEPS = (1.0, 1.5, 2.25)
MAX_ADAPT_CYCLES = 10


@dataclass
class InertiaTestParams:
    swing_deg: float = 20.0         # how far the output should swing either side of the start
    peak_speed: float = 2.0         # output rad/s in the middle of every swing
    cycles: int = 4                 # measured cycles per frequency
    attached_inertia: float = 0.0   # kg*m^2 of anything bolted to the output, subtracted
    max_torque_share: float = 0.6   # of the torque the current limit allows


class InertiaTest(Procedure):
    """A sinusoidal torque at three frequencies; how far the output swings gives its inertia.

    Torque mode: the torque is what the app commands, constant between samples, so the samples see
    all of it. (Position targets sent at the sample rate would move in steps, and the torque kicks
    from those steps fall between samples and bias the result.) The amplitude is adjusted each cycle
    until the output swings about as far as asked, then held for the measured cycles. A weak
    centring spring and damper keep the output from wandering; they are part of the measured torque.
    """

    name = "Inertia test"

    def __init__(self, params: InertiaTestParams):
        super().__init__()
        self.p = params
        self.segments: list[tuple[float, float]] = []   # (frequency Hz, target swing rad)
        self.data: list[dict[str, list[float]]] = []
        self.fit: InertiaFit | None = None
        self.notes: list[str] = []
        self.series: dict[str, list[float]] = {"t": [], "angle_deg": [], "torque_cmd_Nm": [], "torque_Nm": []}
        self.pos0 = 0.0
        self.cap = 0.0
        self._index = 0
        self._t0 = 0.0
        self._cycle = 0
        self._stage = "fade-in"
        self._amplitude = 0.0       # N*m of the sinusoid
        self._adapted = 0
        self._changed = False
        self._measured = 0
        self._angles: list[float] = []
        self._last_t: float | None = None
        self._total = 1.0

    def start(self, ctx: ProcedureContext, sample: Sample) -> None:
        p = self.p
        if p.swing_deg <= 0 or p.peak_speed <= 0 or p.cycles < 2 or p.attached_inertia < 0 \
                or not 0 < p.max_torque_share <= 0.8:
            raise ProcedureError("Swing and speed must be positive, with at least two cycles.")
        swing = math.radians(p.swing_deg)
        speed = min(p.peak_speed, 0.8 * out_speed(ctx, ctx.limits.max_motor_speed))
        base = speed / (TWO_PI * swing)
        self.segments = [(base * k, swing / k) for k in FREQUENCY_STEPS]
        self.data = [{"t": [], "t_torque": [], "speed": [], "torque": []} for _ in self.segments]
        self._total = sum((p.cycles + 5) / frequency for frequency, _ in self.segments)
        full = ctx.model.max_output_torque(ctx.limits.max_current)
        self.cap = p.max_torque_share * full
        self._amplitude = 0.03 * full
        self.pos0 = sample.pos
        arm_for_torque(ctx, 1.2 * self.cap, top_speed=1.5 * motor_turns(ctx, speed))
        self._begin_segment(sample, 0)

    def _begin_segment(self, sample: Sample, index: int) -> None:
        self._index = index
        self._t0 = self.elapsed(sample)
        self._cycle = 0
        self._stage = "fade-in"
        self._adapted = 0
        self._changed = False
        self._measured = 0
        self._angles = []

    def update(self, ctx: ProcedureContext, sample: Sample) -> None:
        t = self.elapsed(sample)
        problem = self.check_running(ctx, sample)
        if problem:
            self.abort(ctx, problem)
            return
        angle = out_angle(ctx, sample.pos - self.pos0)
        if abs(angle) > 2.0 * math.radians(self.p.swing_deg) + math.radians(10.0):
            self.abort(ctx, f"The output swung {math.degrees(abs(angle)):.0f}°, much further than asked. "
                            "Try again with a smaller swing.")
            return
        frequency, _ = self.segments[self._index]
        if int((t - self._t0) * frequency) > self._cycle:
            self._end_cycle(ctx, sample)
            if self.status.done:
                return
            frequency, _ = self.segments[self._index]
        self._angles.append(angle)
        if self._stage == "measuring":
            data = self.data[self._index]
            data["t"].append(t)
            data["t_torque"].append(t if self._last_t is None else (t + self._last_t) / 2.0)
            data["speed"].append(out_speed(ctx, sample.vel))
            data["torque"].append(sample.torque_out)
        self._last_t = t
        command = self._command(t, angle, out_speed(ctx, sample.vel))
        ctx.set_torque_out(command)
        s = self.series
        s["t"].append(t)
        s["angle_deg"].append(math.degrees(angle))
        s["torque_cmd_Nm"].append(command)
        s["torque_Nm"].append(sample.torque_out)
        self.status.phase = f"{self._stage} at {frequency:.2g} Hz"
        self.status.progress = min(0.99, t / self._total)

    def _command(self, t: float, angle: float, speed: float) -> float:
        """The sinusoid (faded in and out at the ends of each frequency), centred by a weak spring and
        damper. A sinusoid started from rest pushes the output off to one side; with little friction
        it would wander away in a slow swing, which the damper settles."""
        frequency, target = self.segments[self._index]
        x = (t - self._t0) * frequency - self._cycle
        envelope = 1.0
        if self._stage == "fade-in":
            envelope = 3 * x * x - 2 * x ** 3
        elif self._stage == "fade-out":
            envelope = 1.0 - (3 * x * x - 2 * x ** 3)
        spring = 0.1 * self._amplitude / target
        damper = 0.3 * self._amplitude / (TWO_PI * frequency * target)
        sinusoid = envelope * self._amplitude * math.sin(TWO_PI * frequency * (t - self._t0))
        return sinusoid - spring * angle - damper * speed

    def _end_cycle(self, ctx: ProcedureContext, sample: Sample) -> None:
        frequency, target = self.segments[self._index]
        swing = (max(self._angles) - min(self._angles)) / 2.0 if self._angles else 0.0
        self._angles = []
        self._cycle += 1
        if self._stage == "fade-in":
            self._stage = "adapting"
        elif self._stage == "adapting":
            ratio = target / max(swing, 1e-9)
            if 0.8 <= ratio <= 1.25 and not self._changed:
                self._stage = "measuring"
            elif 0.8 <= ratio <= 1.25:
                self._changed = False  # one more cycle at this torque, so the swing has settled
            elif self._adapted >= MAX_ADAPT_CYCLES or (self._amplitude >= self.cap and ratio > 1.0):
                if swing < 0.3 * target:
                    ctx.set_torque_out(0.0)
                    self.finish(False, f"The output barely swings, even with {self._amplitude:.3g} N·m. Is it "
                                       "blocked, or loaded by something?", self.result(ctx))
                    return
                self.notes.append(f"{frequency:.2g} Hz: swung {math.degrees(swing):.0f}° instead of "
                                  f"{math.degrees(target):.0f}°.")
                self._stage = "measuring"
            else:
                factor = 2.0 if swing < 0.2 * target else min(max(ratio, 0.6), 1.5)
                self._amplitude = min(self._amplitude * factor, self.cap)
                self._adapted += 1
                self._changed = True
        elif self._stage == "measuring":
            self._measured += 1
            if self._measured >= self.p.cycles:
                self._stage = "fade-out"
        elif self._index + 1 < len(self.segments):
            following = self.segments[self._index + 1][0]
            self._amplitude = min(self._amplitude * following / frequency, self.cap)
            self._begin_segment(sample, self._index + 1)
        else:
            ctx.set_torque_out(0.0)
            self._complete(ctx)

    def _complete(self, ctx: ProcedureContext) -> None:
        swings = []
        for (frequency, _), data in zip(self.segments, self.data):
            n = len(data["t"])
            if n < 20:
                self.notes.append(f"{frequency:.2g} Hz: only {n} samples. Raise the sample rate in Setup.")
                continue
            if n < 12 * self.p.cycles:
                self.notes.append(f"{frequency:.2g} Hz: under 12 samples a cycle; a higher sample rate is more precise.")
            swings.append(analyze_swing(np.array(data["t"]), np.array(data["speed"]), np.array(data["torque"]),
                                        frequency, torque_t=np.array(data["t_torque"])))
        if not swings:
            self.finish(False, "No usable swing. " + " ".join(self.notes), self.result(ctx))
            return
        fit = fit_inertia(swings)
        fit.inertia -= self.p.attached_inertia
        self.fit = fit
        if fit.spread > 0.15:
            self.notes.append("The frequencies disagree by more than 15 %: check that nothing rubs or hangs on the output.")
        self.finish(True, f"Inertia {fit.inertia:.4g} kg·m² at the output.", self.result(ctx))

    def abort(self, ctx: ProcedureContext, reason: str) -> None:
        try:
            ctx.set_torque_out(0.0)
        finally:
            super().abort(ctx, reason)

    def result(self, ctx: ProcedureContext) -> dict:
        result = {"test": "inertia", "swing_deg": self.p.swing_deg, "attached_inertia_kgm2": self.p.attached_inertia,
                  "notes": self.notes, "series": self.series}
        if self.fit:
            result.update(self.fit.to_dict())
        return result


# --- Speed -------------------------------------------------------------------------------------

@dataclass
class SpeedTestParams:
    ramp_s: float = 6.0             # from standstill to the speed limit
    hold_s: float = 1.0             # averaged at the top speed
    both_directions: bool = True


class SpeedTest(Procedure):
    name = "Speed test"

    def __init__(self, params: SpeedTestParams):
        super().__init__()
        self.p = params
        self.directions: list[int] = []
        self.runs: list[dict] = []
        self.constants: dict = {}
        self.series: dict[str, list[float]] = {"t": [], "command_rpm": [], "speed_rpm": [], "current_A": []}
        self.result_values: dict = {}
        self._index = 0
        self._phase = "speeding up"
        self._phase_t0 = 0.0
        self._command = 0.0
        self._hold = 0.0
        self._history: deque[tuple[float, float]] = deque()
        self._collect: dict[str, list[float]] = {}
        self._limit = 1.0
        self._rate = 1.0

    def start(self, ctx: ProcedureContext, sample: Sample) -> None:
        p = self.p
        if p.ramp_s <= 0 or p.hold_s <= 0:
            raise ProcedureError("Ramp and hold times must be positive.")
        paths = ctx.paths
        resistance, inductance, pole_pairs = ctx.read_optional(
            [paths.phase_resistance, paths.phase_inductance, paths.pole_pairs])
        self.constants = {
            "phase_resistance_ohm": float(resistance) if resistance else 0.0,
            "phase_inductance_H": float(inductance) if inductance else 0.0,
            "pole_pairs": int(pole_pairs) if pole_pairs else 7,
            "resistance_measured": bool(resistance),
        }
        self.directions = [1, -1] if p.both_directions else [1]
        self._limit = ctx.limits.max_motor_speed
        self._rate = self._limit / p.ramp_s
        ctx.arm(ControlMode.VELOCITY, vel_limit=self._limit, max_current=ctx.limits.max_current)
        self._set_phase("speeding up", sample)

    def _set_phase(self, phase: str, sample: Sample) -> None:
        self._phase = phase
        self._phase_t0 = self.elapsed(sample)
        direction = "forward" if self.directions[self._index] > 0 else "backward"
        self.status.phase = f"{phase} ({direction})"

    def update(self, ctx: ProcedureContext, sample: Sample) -> None:
        t = self.elapsed(sample)
        problem = self.check_running(ctx, sample)
        if problem:
            self.abort(ctx, problem)
            return
        direction = self.directions[self._index]
        speed = abs(sample.vel)
        t_phase = t - self._phase_t0
        to_rpm = 60.0 / ctx.model.gear_ratio
        self.series["t"].append(t)
        self.series["command_rpm"].append(direction * self._command * to_rpm)
        self.series["speed_rpm"].append(sample.vel * to_rpm)
        self.series["current_A"].append(sample.iq)
        self.status.progress = min(0.99, (self._index + min(1.0, t_phase / (self.p.ramp_s + 3.0))) / len(self.directions))

        if self._phase == "speeding up":
            self._command = min(self._rate * t_phase, self._limit)
            ctx.set_velocity_turns(direction * self._command)
            history = self._history
            history.append((t, speed))
            while history and history[0][0] < t - 0.5:
                history.popleft()
            if self._command > 0.2 * self._limit and speed < 0.05 * self._command \
                    and abs(sample.iq) > 0.8 * ctx.limits.max_current:
                self.abort(ctx, "The output doesn't turn. It must spin freely, with nothing attached, for this test.")
                return
            span = t - history[0][0]
            lagging = self._command - speed > 0.05 * self._command + 0.05
            flat = span >= 0.4 and speed - history[0][1] < 0.25 * self._rate * span
            if (lagging and flat) or self._command >= self._limit:
                self._hold = self._command
                self._collect = {"speed": [], "current": [], "vbus": []}
                self._set_phase("measuring", sample)
        elif self._phase == "measuring":
            ctx.set_velocity_turns(direction * self._hold)
            if t_phase >= 0.3:
                self._collect["speed"].append(speed)
                self._collect["current"].append(abs(sample.iq))
                self._collect["vbus"].append(sample.vbus)
            if t_phase >= 0.3 + self.p.hold_s:
                top = _mean(self._collect["speed"])
                self.runs.append({
                    "direction": direction,
                    "speed_motor_turns_s": top,
                    "current_A": _mean(self._collect["current"]),
                    "vbus_V": _mean([v for v in self._collect["vbus"] if math.isfinite(v)]),
                    "voltage_limited": top < 0.95 * self._hold,
                })
                self._set_phase("slowing down", sample)
        else:  # slowing down
            self._command = self._hold * max(0.0, 1.0 - t_phase / 1.5)
            ctx.set_velocity_turns(direction * self._command)
            if t_phase >= 2.0:
                self._index += 1
                self._history.clear()
                if self._index >= len(self.directions):
                    self._complete(ctx)
                else:
                    self._command = 0.0
                    self._set_phase("speeding up", sample)

    def _complete(self, ctx: ProcedureContext) -> None:
        ctx.set_velocity_turns(0.0)
        limited = [r for r in self.runs if r["voltage_limited"] and math.isfinite(r["vbus_V"])]
        gear = ctx.model.gear_ratio
        if not limited:
            top = max((r["speed_motor_turns_s"] for r in self.runs), default=0.0)
            self.finish(False, f"The motor reached your speed limit ({top / gear * 60:.4g} rpm at the output) before it "
                               "ran out of voltage, so the no-load speed is higher. Raise Max output speed in Setup "
                               "and run again.", self.result(ctx))
            return
        motor_speed = _mean([r["speed_motor_turns_s"] for r in limited]) * TWO_PI
        current = _mean([r["current_A"] for r in limited])
        vbus = _mean([r["vbus_V"] for r in limited])
        c = self.constants
        try:
            motor = motor_from_no_load(motor_speed, current, vbus, c["phase_resistance_ohm"],
                                       c["phase_inductance_H"], c["pole_pairs"])
        except ValueError as exc:
            self.finish(False, str(exc), self.result(ctx))
            return
        self.result_values = {
            "no_load_speed_rad_s": motor_speed / gear,
            "no_load_speed_rpm": motor_speed / gear * 60.0 / TWO_PI,
            "no_load_current_A": current,
            "vbus_V": vbus,
            "zero_torque_speed_rad_s": zero_torque_speed(motor, vbus) / gear,
            "motor": motor.to_dict(),
        }
        note = "" if c["resistance_measured"] else " (the ODrive didn't report a phase resistance, so 0 Ω was used)"
        self.finish(True, f"No-load speed {motor_speed / gear:.3g} rad/s at {vbus:.1f} V; back-EMF Kt "
                          f"{motor.kt:.4g} N·m/A{note}.", self.result(ctx))

    def abort(self, ctx: ProcedureContext, reason: str) -> None:
        try:
            ctx.set_velocity_turns(0.0)
        finally:
            super().abort(ctx, reason)

    def result(self, ctx: ProcedureContext) -> dict:
        return {"test": "speed", "runs": self.runs, **self.constants, **self.result_values, "series": self.series}


# --- Backlash and stiffness --------------------------------------------------------------------

@dataclass
class BacklashTestParams:
    max_torque_out: float           # N*m, applied both ways
    cycle_s: float = 8.0            # one full +T → −T → +T cycle
    cycles: int = 2
    max_movement_deg: float = 5.0   # output degrees; more means the output isn't clamped


class BacklashTest(Procedure):
    name = "Backlash test"

    def __init__(self, params: BacklashTestParams):
        super().__init__()
        self.p = params
        self.series: dict[str, list[float]] = {"t": [], "torque_cmd_Nm": [], "torque_Nm": [], "angle_deg": []}
        self.values: dict = {}
        self.pos0 = 0.0

    @property
    def total_time(self) -> float:
        return self.p.cycle_s * self.p.cycles

    def start(self, ctx: ProcedureContext, sample: Sample) -> None:
        p = self.p
        if p.max_torque_out <= 0 or p.cycle_s <= 0 or p.cycles < 1 or p.max_movement_deg <= 0:
            raise ProcedureError("Torque, cycle time and allowed movement must be positive.")
        self.pos0 = sample.pos
        arm_for_torque(ctx, p.max_torque_out)
        self.status.phase = "winding up"

    def command_at(self, t: float) -> float:
        if t >= self.total_time:
            return 0.0
        return self.p.max_torque_out * (2.0 / math.pi) * math.asin(math.sin(TWO_PI * t / self.p.cycle_s))

    def update(self, ctx: ProcedureContext, sample: Sample) -> None:
        t = self.elapsed(sample)
        angle = out_angle(ctx, sample.pos - self.pos0)
        if abs(math.degrees(angle)) > self.p.max_movement_deg:
            self.abort(ctx, f"The output moved {abs(math.degrees(angle)):.1f}°, so it isn't clamped. Clamp or bolt "
                            "the output so it can't turn either way, and try again.")
            return
        problem = self.check_running(ctx, sample)
        if problem:
            self.abort(ctx, problem)
            return
        command = self.command_at(t)
        s = self.series
        s["t"].append(t)
        s["torque_cmd_Nm"].append(command)
        s["torque_Nm"].append(sample.torque_out)
        s["angle_deg"].append(math.degrees(angle))
        if t >= self.total_time:
            ctx.set_torque_out(0.0)
            self._complete(ctx)
            return
        ctx.set_torque_out(command)
        self.status.progress = min(0.99, t / self.total_time)

    def _complete(self, ctx: ProcedureContext) -> None:
        t = np.array(self.series["t"])
        use = t >= self.p.cycle_s / 4.0  # after the first wind-up from rest
        rising = np.gradient(np.array(self.series["torque_cmd_Nm"])) > 0
        try:
            fit = fit_backlash(np.radians(np.array(self.series["angle_deg"])[use]),
                               np.array(self.series["torque_Nm"])[use], rising[use])
        except ValueError as exc:
            self.finish(False, str(exc), self.result(ctx))
            return
        self.values = fit.to_dict()
        self.finish(True, f"Backlash {math.degrees(fit.backlash):.3g}°, stiffness {fit.stiffness:.4g} N·m/rad.",
                    self.result(ctx))

    def abort(self, ctx: ProcedureContext, reason: str) -> None:
        try:
            ctx.set_torque_out(0.0)
        finally:
            super().abort(ctx, reason)

    def result(self, ctx: ProcedureContext) -> dict:
        return {"test": "backlash", "torque_amplitude_Nm": self.p.max_torque_out, **self.values, "series": self.series}


# --- Thermal -----------------------------------------------------------------------------------

@dataclass
class ThermalTestParams:
    current_a: float                # held for the whole test
    temp_limit_c: float = 80.0      # the test stops when the sensor reaches this
    max_duration_s: float = 900.0
    direction: int = 1
    max_movement_deg: float = 10.0


class ThermalTest(Procedure):
    name = "Thermal test"

    def __init__(self, params: ThermalTestParams):
        super().__init__()
        self.p = params
        self.sensor = "motor"
        self.series: dict[str, list[float]] = {"t": [], "temperature_C": [], "current_A": []}
        self.values: dict = {}
        self.pos0 = 0.0
        self.target = 0.0
        self._last_record = -math.inf

    def _temperature(self, ctx: ProcedureContext) -> float:
        slow = ctx.latest_slow
        if slow is None:
            return math.nan
        return slow.motor_temp if self.sensor == "motor" else slow.fet_temp

    def start(self, ctx: ProcedureContext, sample: Sample) -> None:
        p = self.p
        if not 0 < p.current_a <= ctx.limits.max_current * 1.0001:
            raise ProcedureError(f"Choose a test current between 0 and your {ctx.limits.max_current:g} A limit.")
        if p.max_duration_s < 30 or p.max_movement_deg <= 0:
            raise ProcedureError("Run for at least 30 s, with a positive allowed movement.")
        self.sensor = "motor" if ctx.motor_thermistor else "driver"
        start_temp = self._temperature(ctx)
        if math.isfinite(start_temp) and start_temp >= p.temp_limit_c - 5.0:
            raise ProcedureError(f"The {self.sensor} is already at {start_temp:.0f} °C. Let it cool down first.")
        self.pos0 = sample.pos
        self.target = ctx.model.output_torque(p.current_a) * (1 if p.direction >= 0 else -1)
        arm_for_torque(ctx, self.target)
        self.status.phase = "heating"

    def update(self, ctx: ProcedureContext, sample: Sample) -> None:
        t = self.elapsed(sample)
        movement = abs(math.degrees(out_angle(ctx, sample.pos - self.pos0)))
        if movement > self.p.max_movement_deg:
            self.abort(ctx, f"The arm moved {movement:.1f}°, so it isn't blocked. Press it firmly against a fixed "
                            "object, or clamp the output, and try again.")
            return
        problem = self.check_running(ctx, sample)
        if problem:
            self.abort(ctx, problem)
            return
        ctx.set_torque_out(self.target * min(1.0, t / 1.0))
        temp = self._temperature(ctx)
        if t >= 1.5 and t - self._last_record >= 0.5 and math.isfinite(temp):
            self._last_record = t
            self.series["t"].append(t)
            self.series["temperature_C"].append(temp)
            self.series["current_A"].append(abs(sample.iq))
        first = self.series["temperature_C"][0] if self.series["temperature_C"] else temp
        limit, duration = self.p.temp_limit_c, self.p.max_duration_s
        heat = (temp - first) / max(limit - first, 1e-6) if math.isfinite(temp) and math.isfinite(first) else 0.0
        self.status.progress = min(0.99, max(t / duration, heat))
        self.status.phase = f"heating: {temp:.1f} °C" if math.isfinite(temp) else "heating"
        if math.isfinite(temp) and temp >= limit:
            self._complete(ctx, f"Reached {limit:g} °C after {t:.0f} s.")
        elif t >= duration:
            self._complete(ctx, f"Ran for {duration:.0f} s.")

    def stop(self, ctx: ProcedureContext) -> None:
        self._complete(ctx, "Stopped by user.")

    def _complete(self, ctx: ProcedureContext, how: str) -> None:
        ctx.set_torque_out(0.0)
        s = self.series
        try:
            fit = fit_thermal(np.array(s["t"]), np.array(s["temperature_C"]), _mean(s["current_A"]), self.sensor)
        except ValueError as exc:
            self.finish(False, f"{how} {exc}", self.result(ctx))
            return
        self.values = {**fit.to_dict(), "temp_limit_C": self.p.temp_limit_c,
                       "continuous_current_A": fit.continuous_current(self.p.temp_limit_c)}
        warning = " The test was shorter than the heating time constant, so run it longer for a firmer rating." \
            if fit.extrapolated else ""
        self.finish(True, f"{how} Continuous current about {self.values['continuous_current_A']:.3g} A at "
                          f"{self.p.temp_limit_c:g} °C.{warning}", self.result(ctx))

    def abort(self, ctx: ProcedureContext, reason: str) -> None:
        try:
            ctx.set_torque_out(0.0)
        finally:
            super().abort(ctx, reason)

    def result(self, ctx: ProcedureContext) -> dict:
        return {"test": "thermal", "sensor": self.sensor, "temp_limit_C": self.p.temp_limit_c,
                **self.values, "series": self.series}
