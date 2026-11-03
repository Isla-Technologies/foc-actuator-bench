"""A simulated ODrive v3.6 (firmware 0.5.x) with an actuator attached.

Used for development, demos and automated tests when no hardware is available. It mimics the
firmware's property tree, state machine, position/velocity/torque cascade (including the
torque-mode velocity limiter and overspeed check), limits, watchdog and errors closely enough to
exercise the whole app. The simulated actuator deliberately differs from its nominal datasheet
values (Kt, gearbox efficiency) so the weight calibration has something real to find.
"""

from __future__ import annotations

import math
import random
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, fields

from ..odrive_api import CALIBRATION_STATES, AxisState, ControlMode, InputMode
from ..torque import KT_PER_KV
from .base import Backend, PropertyUnavailable

G = 9.80665
TWO_PI = 2.0 * math.pi

SCENARIOS = {
    "free": "Nothing attached (output turns freely)",
    "wall": "Arm against a fixed object (stall test)",
    "lever": "Arm with a hanging weight (gravity load)",
    "locked": "Output clamped (cannot move)",
}


@dataclass
class SimActuator:
    """Physical truth of the simulated actuator."""

    kv: float = 100.0                 # motor KV [rpm/V]
    gear_ratio: float = 9.0
    true_kt_factor: float = 1.05      # real Kt = 8.27/KV * factor
    true_efficiency: float = 0.90     # real gearbox efficiency
    rotor_inertia: float = 1.2e-4     # kg*m^2, motor side
    load_inertia: float = 5e-4        # kg*m^2, output side (hub, coupling)
    coulomb_friction: float = 0.15    # N*m at the output
    viscous_friction: float = 0.02    # N*m per rad/s at the output
    current_noise: float = 0.08       # A rms noise on Iq_measured
    phase_resistance: float = 0.06    # ohm
    supply_voltage: float = 24.0
    supply_resistance: float = 0.05   # ohm, causes bus sag under load
    odrive_kt_configured: bool = True  # False: torque_constant left at the 0.04 factory default
    current_lim: float = 20.0         # ODrive motor.config.current_lim [A]
    scenario: str = "wall"
    wall_angle_deg: float = 2.0       # fixed objects at +/- this output angle (wall scenario)
    arm_length_m: float = 0.30
    arm_mass_kg: float = 0.10         # the arm itself (centre of mass at half length)
    mass_kg: float = 0.0              # weight hanging at the end of the arm (lever scenario)

    @property
    def kt_nominal(self) -> float:
        return KT_PER_KV / self.kv

    @property
    def kt_true(self) -> float:
        return self.kt_nominal * self.true_kt_factor

    @property
    def true_k_out(self) -> float:
        """Real output torque per amp: what a perfect calibration should find."""
        return self.kt_true * self.gear_ratio * self.true_efficiency

    @property
    def has_arm(self) -> bool:
        return self.scenario in ("wall", "lever")


class ManualClock:
    """Deterministic clock for tests: time only moves when ``advance`` is called."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


@dataclass
class _Prop:
    get: Callable[[], object]
    set: Callable[[object], None] | None
    kind: type


_AXIS_CONFIG_DEFAULTS: list[tuple[str, object, type]] = [
    ("config.enable_watchdog", False, bool),
    ("config.watchdog_timeout", 0.0, float),
    ("motor.config.torque_constant", 0.04, float),  # factory default; axis0 gets the actuator's value
    ("motor.config.current_lim", 10.0, float),
    ("motor.config.torque_lim", math.inf, float),
    ("motor.config.pole_pairs", 7, int),
    ("motor.config.motor_type", 0, int),
    ("motor.config.requested_current_range", 60.0, float),
    ("motor.config.calibration_current", 10.0, float),
    ("motor.current_control.I_measured_report_filter_k", 1.0, float),
    ("motor.motor_thermistor.config.enabled", False, bool),
    ("encoder.config.cpr", 8192, int),
    ("controller.config.control_mode", int(ControlMode.POSITION), int),
    ("controller.config.input_mode", int(InputMode.PASSTHROUGH), int),
    ("controller.config.vel_limit", 2.0, float),
    ("controller.config.vel_limit_tolerance", 1.2, float),
    ("controller.config.enable_vel_limit", True, bool),
    ("controller.config.enable_torque_mode_vel_limit", True, bool),
    ("controller.config.enable_overspeed_error", True, bool),
    ("controller.config.pos_gain", 20.0, float),
    ("controller.config.vel_gain", 0.16, float),
    ("controller.config.vel_integrator_gain", 0.32, float),
    ("controller.config.vel_integrator_limit", math.inf, float),
    ("controller.config.vel_ramp_rate", 1.0, float),
    ("controller.config.torque_ramp_rate", 0.01, float),
]

# error bits used by the simulator (values from odrive_api tables)
_AXIS_INVALID_STATE = 0x1
_AXIS_MOTOR_FAILED = 0x40
_AXIS_ENCODER_FAILED = 0x100
_AXIS_CONTROLLER_FAILED = 0x200
_AXIS_WATCHDOG_EXPIRED = 0x800
_MOTOR_FET_OVER_TEMP = 0x40000
_ENCODER_NO_RESPONSE = 0x4
_CONTROLLER_OVERSPEED = 0x1
_SYSTEM_UNDER_VOLTAGE = 0x2

CALIBRATION_DURATION_S = 3.0
FET_TRIP_TEMPERATURE = 120.0


class _SimAxis:
    def __init__(self, index: int, has_motor: bool):
        self.index = index
        self.has_motor = has_motor
        self.state = int(AxisState.IDLE)
        self.error = 0
        self.motor_error = 0
        self.encoder_error = 0
        self.controller_error = 0
        self.motor_calibrated = has_motor
        self.encoder_ready = has_motor
        self.pos = 0.0          # motor turns
        self.vel = 0.0          # motor turns/s
        self.iq = 0.0
        self.iq_set = 0.0
        self.iq_report = 0.0
        self.vel_integrator = 0.0
        self.pos_setpoint = 0.0
        self.vel_setpoint = 0.0
        self.torque_setpoint = 0.0
        self.input_pos = 0.0
        self.input_vel = 0.0
        self.input_torque = 0.0
        self.fet_temp = 25.0
        self.motor_temp = 25.0
        self.wd_deadline = 0.0     # like the firmware: feed() loads a countdown from the current timeout
        self.calibration_end: float | None = None
        self.i_bus = 0.0
        self.mechanical_power = 0.0
        self.electrical_power = 0.0
        self.cfg: dict[str, object] = {key: default for key, default, _ in _AXIS_CONFIG_DEFAULTS}

    @property
    def any_error(self) -> bool:
        return bool(self.error or self.motor_error or self.encoder_error or self.controller_error)


class SimulatedODrive:
    """Property-level model of an ODrive v3.6-56V running firmware 0.5.6."""

    HW = (3, 6, 56)
    FW = (0, 5, 6)
    SERIAL = 0x3348373D3432

    def __init__(
        self,
        actuator: SimActuator | None = None,
        *,
        clock: Callable[[], float] | None = None,
        seed: int | None = 1,
        step_s: float = 0.001,
    ):
        self.actuator = actuator or SimActuator()
        self._clock = clock or time.perf_counter
        self._rng = random.Random(seed)
        self._step_s = step_s
        self._lock = threading.RLock()
        self._t = self._clock()
        self.system_error = 0
        self.vbus = self.actuator.supply_voltage
        self.ibus = 0.0
        self.saved_configuration = False
        self.axes = [_SimAxis(0, has_motor=True), _SimAxis(1, has_motor=False)]
        self._apply_actuator_config(self.axes[0])
        self._props: dict[str, _Prop] = {}
        self._funcs: dict[str, Callable[[], object]] = {}
        self._build_registry()

    # --- public API ------------------------------------------------------------------------
    def read(self, path: str):
        with self._lock:
            self._advance()
            prop = self._props.get(path)
            if prop is None:
                raise PropertyUnavailable(path, "no such property")
            return _coerce(prop.get(), prop.kind)

    def write(self, path: str, value) -> None:
        with self._lock:
            self._advance()
            prop = self._props.get(path)
            if prop is None:
                raise PropertyUnavailable(path, "no such property")
            if prop.set is None:
                raise PropertyUnavailable(path, "read-only")
            prop.set(_coerce(value, prop.kind))

    def call(self, path: str):
        with self._lock:
            self._advance()
            func = self._funcs.get(path)
            if func is None:
                raise PropertyUnavailable(path, "no such function")
            return func()

    def property_kind(self, path: str) -> type | None:
        prop = self._props.get(path)
        return prop.kind if prop else None

    def is_writable(self, path: str) -> bool:
        prop = self._props.get(path)
        return bool(prop and prop.set)

    def has_function(self, path: str) -> bool:
        return path in self._funcs

    def update_actuator(self, **changes) -> None:
        """Change the simulated hardware/scenario on the fly (e.g. hang a different mass)."""
        names = {f.name for f in fields(SimActuator)}
        with self._lock:
            self._advance()
            for key, value in changes.items():
                if key not in names:
                    raise AttributeError(f"SimActuator has no field {key!r}")
                setattr(self.actuator, key, value)
            if "current_lim" in changes:
                self.axes[0].cfg["motor.config.current_lim"] = float(self.actuator.current_lim)

    def true_output_torque(self, axis: int = 0) -> float:
        a = self.actuator
        return a.kt_true * self.axes[axis].iq * a.gear_ratio * a.true_efficiency

    def inject_error(self, axis: int, component: str, bit: int) -> None:
        """Raise an error as if the firmware had detected it (for testing the UI)."""
        with self._lock:
            ax = self.axes[axis]
            if component == "system":
                self.system_error |= bit
                return
            attr = {"axis": "error", "motor": "motor_error", "encoder": "encoder_error",
                    "controller": "controller_error"}[component]
            setattr(ax, attr, getattr(ax, attr) | bit)
            if component == "motor":
                ax.error |= _AXIS_MOTOR_FAILED
            elif component == "encoder":
                ax.error |= _AXIS_ENCODER_FAILED
            elif component == "controller":
                ax.error |= _AXIS_CONTROLLER_FAILED
            self._go_idle(ax)

    # --- setup -----------------------------------------------------------------------------
    def _apply_actuator_config(self, ax: _SimAxis) -> None:
        a = self.actuator
        ax.cfg["motor.config.torque_constant"] = a.kt_nominal if a.odrive_kt_configured else 0.04
        ax.cfg["motor.config.current_lim"] = float(a.current_lim)

    def _build_registry(self) -> None:
        reg = self._register
        reg("vbus_voltage", lambda: self.vbus + self._rng.gauss(0.0, 0.02))
        reg("ibus", lambda: self.ibus)
        reg("error", lambda: self.system_error, self._set_system_error, int)
        reg("fw_version_major", lambda: self.FW[0], kind=int)
        reg("fw_version_minor", lambda: self.FW[1], kind=int)
        reg("fw_version_revision", lambda: self.FW[2], kind=int)
        reg("fw_version_unreleased", lambda: 0, kind=int)
        reg("hw_version_major", lambda: self.HW[0], kind=int)
        reg("hw_version_minor", lambda: self.HW[1], kind=int)
        reg("hw_version_variant", lambda: self.HW[2], kind=int)
        reg("serial_number", lambda: self.SERIAL, kind=int)
        system_cfg = {"config.usb_cdc_protocol": (3, int), "config.enable_uart_a": (True, bool),
                      "config.brake_resistance": (2.0, float),
                      "config.dc_bus_overvoltage_trip_level": (59.92, float),
                      "config.dc_bus_undervoltage_trip_level": (8.0, float)}
        store = {k: v for k, (v, _) in system_cfg.items()}
        for key, (_, kind) in system_cfg.items():
            reg(key, lambda k=key: store[k], lambda v, k=key: store.__setitem__(k, v), kind)
        self._funcs["clear_errors"] = self._clear_errors
        self._funcs["save_configuration"] = self._save_configuration
        self._funcs["erase_configuration"] = lambda: None
        self._funcs["reboot"] = self._reboot

        for ax in self.axes:
            self._register_axis(ax)

    def _register_axis(self, ax: _SimAxis) -> None:
        p = f"axis{ax.index}."
        reg = self._register
        reg(p + "current_state", lambda: ax.state, kind=int)
        reg(p + "requested_state", lambda: 0, lambda v: self._request_state(ax, int(v)), int)
        reg(p + "error", lambda: ax.error, lambda v: setattr(ax, "error", int(v)), int)
        reg(p + "motor.error", lambda: ax.motor_error, lambda v: setattr(ax, "motor_error", int(v)), int)
        reg(p + "encoder.error", lambda: ax.encoder_error, lambda v: setattr(ax, "encoder_error", int(v)), int)
        reg(p + "controller.error", lambda: ax.controller_error,
            lambda v: setattr(ax, "controller_error", int(v)), int)
        reg(p + "motor.is_calibrated", lambda: ax.motor_calibrated, kind=bool)
        reg(p + "encoder.is_ready", lambda: ax.encoder_ready, kind=bool)
        reg(p + "motor.current_control.Iq_measured", lambda: self._iq_measured(ax))
        reg(p + "motor.current_control.Iq_setpoint", lambda: ax.iq_set)
        reg(p + "motor.current_control.Id_measured",
            lambda: self._rng.gauss(0.0, self.actuator.current_noise) if ax.has_motor else 0.0)
        reg(p + "motor.I_bus", lambda: ax.i_bus)
        reg(p + "motor.fet_thermistor.temperature", lambda: ax.fet_temp)
        reg(p + "motor.motor_thermistor.temperature",
            lambda: ax.motor_temp if ax.cfg["motor.motor_thermistor.config.enabled"] else 0.0)
        reg(p + "encoder.pos_estimate", lambda: self._pos_reported(ax))
        reg(p + "encoder.vel_estimate",
            lambda: ax.vel + (self._rng.gauss(0.0, 0.003) if ax.has_motor else 0.0))
        reg(p + "controller.input_pos", lambda: ax.input_pos, lambda v: setattr(ax, "input_pos", v))
        reg(p + "controller.input_vel", lambda: ax.input_vel, lambda v: setattr(ax, "input_vel", v))
        reg(p + "controller.input_torque", lambda: ax.input_torque,
            lambda v: setattr(ax, "input_torque", v))
        reg(p + "controller.mechanical_power", lambda: ax.mechanical_power)
        reg(p + "controller.electrical_power", lambda: ax.electrical_power)
        for key, _, kind in _AXIS_CONFIG_DEFAULTS:
            reg(p + key, lambda k=key: ax.cfg[k], lambda v, k=key: ax.cfg.__setitem__(k, v), kind)
        self._funcs[p + "watchdog_feed"] = lambda: setattr(
            ax, "wd_deadline", self._t + float(ax.cfg["config.watchdog_timeout"]))

    def _register(self, path, getter, setter=None, kind: type = float) -> None:
        self._props[path] = _Prop(getter, setter, kind)

    # --- reported values -------------------------------------------------------------------
    def _iq_measured(self, ax: _SimAxis) -> float:
        if not ax.has_motor:
            return 0.0
        noise = self.actuator.current_noise
        if ax.state != AxisState.CLOSED_LOOP_CONTROL:
            noise *= 0.25  # with the power stage off only sensor noise remains
        raw = ax.iq + self._rng.gauss(0.0, noise)
        k = float(ax.cfg["motor.current_control.I_measured_report_filter_k"])
        ax.iq_report = raw if k >= 1.0 else ax.iq_report + k * (raw - ax.iq_report)
        return ax.iq_report

    def _pos_reported(self, ax: _SimAxis) -> float:
        cpr = max(1, int(ax.cfg["encoder.config.cpr"]))
        return round(ax.pos * cpr) / cpr

    # --- functions -------------------------------------------------------------------------
    def _set_system_error(self, value) -> None:
        self.system_error = int(value)

    def _clear_errors(self) -> None:
        self.system_error = 0
        for ax in self.axes:
            ax.error = ax.motor_error = ax.encoder_error = ax.controller_error = 0

    def _save_configuration(self) -> bool:
        self.saved_configuration = True
        return True

    def _reboot(self) -> None:
        for ax in self.axes:
            self._go_idle(ax)
            ax.cfg["config.enable_watchdog"] = False
        self._clear_errors()

    # --- state machine ---------------------------------------------------------------------
    def _request_state(self, ax: _SimAxis, state: int) -> None:
        if state == AxisState.IDLE:
            self._go_idle(ax)
        elif state == AxisState.CLOSED_LOOP_CONTROL:
            if ax.any_error:
                self._go_idle(ax)
            elif not (ax.motor_calibrated and ax.encoder_ready):
                ax.error |= _AXIS_INVALID_STATE
                self._go_idle(ax)
            else:
                # Like the firmware: start from the current position to avoid a jump.
                ax.state = int(AxisState.CLOSED_LOOP_CONTROL)
                ax.input_pos = ax.pos_setpoint = ax.pos
                ax.vel_setpoint = 0.0
                ax.torque_setpoint = 0.0
                ax.vel_integrator = 0.0
        elif state in CALIBRATION_STATES:
            if ax.any_error:
                return
            if not ax.has_motor:
                ax.motor_error |= 0x1  # PHASE_RESISTANCE_OUT_OF_RANGE: nothing connected
                ax.error |= _AXIS_MOTOR_FAILED
                return
            ax.state = state
            ax.calibration_end = self._t + CALIBRATION_DURATION_S
        else:
            ax.error |= _AXIS_INVALID_STATE

    def _go_idle(self, ax: _SimAxis) -> None:
        ax.state = int(AxisState.IDLE)
        ax.iq_set = 0.0
        ax.vel_integrator = 0.0
        ax.calibration_end = None

    def _fail(self, ax: _SimAxis, axis_bit: int, component: str | None = None, bit: int = 0) -> None:
        if component:
            attr = {"motor": "motor_error", "encoder": "encoder_error", "controller": "controller_error"}[component]
            setattr(ax, attr, getattr(ax, attr) | bit)
        ax.error |= axis_bit
        self._go_idle(ax)

    # --- physics ---------------------------------------------------------------------------
    def _advance(self) -> None:
        now = self._clock()
        elapsed = now - self._t
        if elapsed <= 0:
            return
        # After a long pause (debugger, sleep) only simulate the last second.
        start = max(self._t, now - 1.0)
        span = now - start
        steps = max(1, math.ceil(span / self._step_s))
        h = span / steps
        t = start
        for _ in range(steps):
            t += h
            self._t = t
            for ax in self.axes:
                self._step_axis(ax, h)
            self._update_bus()
        self._t = now

    def _step_axis(self, ax: _SimAxis, dt: float) -> None:
        a = self.actuator
        cfg = ax.cfg

        if cfg["config.enable_watchdog"] and self._t > ax.wd_deadline:
            if not ax.error & _AXIS_WATCHDOG_EXPIRED:
                self._fail(ax, _AXIS_WATCHDOG_EXPIRED)

        if ax.state in CALIBRATION_STATES and ax.calibration_end is not None:
            if a.scenario == "locked":
                self._fail(ax, _AXIS_ENCODER_FAILED, "encoder", _ENCODER_NO_RESPONSE)
            elif self._t >= ax.calibration_end:
                ax.motor_calibrated = ax.encoder_ready = True
                self._go_idle(ax)

        torque_cmd = 0.0
        if ax.state == AxisState.CLOSED_LOOP_CONTROL and ax.has_motor:
            torque_cmd = self._controller(ax, dt)
        kt_cfg = float(cfg["motor.config.torque_constant"]) or 1e-9
        ax.iq_set = torque_cmd / kt_cfg if ax.state == AxisState.CLOSED_LOOP_CONTROL else 0.0
        # current loop: first-order response (~0.5 ms)
        ax.iq += (ax.iq_set - ax.iq) * (1.0 - math.exp(-dt / 0.0005))

        if not ax.has_motor:
            return
        self._mechanics(ax, dt)
        self._thermal(ax, dt)

        motor_speed_rad = ax.vel * TWO_PI
        ax.mechanical_power = a.kt_true * ax.iq * motor_speed_rad
        copper = 1.5 * a.phase_resistance * ax.iq * ax.iq
        ax.electrical_power = ax.mechanical_power + copper
        ax.i_bus = ax.electrical_power / max(self.vbus, 1.0)

        if ax.fet_temp > FET_TRIP_TEMPERATURE:
            self._fail(ax, _AXIS_MOTOR_FAILED, "motor", _MOTOR_FET_OVER_TEMP)

    def _controller(self, ax: _SimAxis, dt: float) -> float:
        """Firmware 0.5.x Controller::update(), simplified to the parts that matter here."""
        cfg = ax.cfg
        mode = int(cfg["controller.config.control_mode"])
        input_mode = int(cfg["controller.config.input_mode"])

        if input_mode == InputMode.VEL_RAMP:
            step = float(cfg["controller.config.vel_ramp_rate"]) * dt
            ax.vel_setpoint += max(-step, min(step, ax.input_vel - ax.vel_setpoint))
            ax.torque_setpoint = ax.input_torque
        elif input_mode == InputMode.TORQUE_RAMP:
            step = float(cfg["controller.config.torque_ramp_rate"]) * dt
            ax.torque_setpoint += max(-step, min(step, ax.input_torque - ax.torque_setpoint))
        else:
            ax.pos_setpoint = ax.input_pos
            ax.vel_setpoint = ax.input_vel
            ax.torque_setpoint = ax.input_torque

        vel_est = ax.vel
        vel_lim = float(cfg["controller.config.vel_limit"])
        vel_gain = float(cfg["controller.config.vel_gain"])

        vel_des = ax.vel_setpoint
        if mode >= ControlMode.POSITION:
            vel_des += float(cfg["controller.config.pos_gain"]) * (ax.pos_setpoint - ax.pos)
        if mode >= ControlMode.VELOCITY and cfg["controller.config.enable_vel_limit"]:
            vel_des = max(-vel_lim, min(vel_lim, vel_des))

        if cfg["controller.config.enable_overspeed_error"]:
            if abs(vel_est) > float(cfg["controller.config.vel_limit_tolerance"]) * vel_lim:
                self._fail(ax, _AXIS_CONTROLLER_FAILED, "controller", _CONTROLLER_OVERSPEED)
                return 0.0

        torque = ax.torque_setpoint
        v_err = 0.0
        if mode >= ControlMode.VELOCITY:
            v_err = vel_des - vel_est
            torque += vel_gain * v_err + ax.vel_integrator

        if mode < ControlMode.VELOCITY and cfg["controller.config.enable_torque_mode_vel_limit"]:
            t_max = (vel_lim - vel_est) * vel_gain
            t_min = (-vel_lim - vel_est) * vel_gain
            torque = max(t_min, min(t_max, torque))

        kt_cfg = float(cfg["motor.config.torque_constant"])
        t_lim = min(float(cfg["motor.config.torque_lim"]), float(cfg["motor.config.current_lim"]) * kt_cfg)
        limited = False
        if torque > t_lim:
            torque, limited = t_lim, True
        elif torque < -t_lim:
            torque, limited = -t_lim, True

        if mode < ControlMode.VELOCITY:
            ax.vel_integrator = 0.0
        else:
            if limited:
                ax.vel_integrator *= 0.99
            else:
                ax.vel_integrator += float(cfg["controller.config.vel_integrator_gain"]) * dt * v_err
            lim = float(cfg["controller.config.vel_integrator_limit"])
            ax.vel_integrator = max(-lim, min(lim, ax.vel_integrator))
        return torque

    def _mechanics(self, ax: _SimAxis, dt: float) -> None:
        a = self.actuator
        n = a.gear_ratio
        theta = ax.pos / n * TWO_PI        # output angle [rad]
        omega = ax.vel / n * TWO_PI        # output speed [rad/s]

        if a.scenario == "locked":
            ax.vel = 0.0
            return

        inertia = a.rotor_inertia * n * n + a.load_inertia
        external = 0.0
        if a.has_arm:
            inertia += a.mass_kg * a.arm_length_m ** 2 + a.arm_mass_kg * a.arm_length_m ** 2 / 3.0
        if a.scenario == "lever":
            lever_moment = a.mass_kg * a.arm_length_m + a.arm_mass_kg * a.arm_length_m / 2.0
            external = -G * lever_moment * math.cos(theta)

        drive = a.kt_true * ax.iq * n * a.true_efficiency
        applied = drive + external - a.viscous_friction * omega
        stiction = a.coulomb_friction
        if abs(omega) < 1e-4:
            if abs(applied) <= stiction:
                omega_new = 0.0
            else:
                omega_new = omega + (applied - math.copysign(stiction, applied)) / inertia * dt
        else:
            omega_new = omega + (applied - math.copysign(stiction, omega)) / inertia * dt
            if omega_new * omega < 0 and abs(applied) <= stiction:
                omega_new = 0.0  # friction stops the motion, it can't reverse it

        theta_new = theta + omega_new * dt
        if a.scenario == "wall":
            limit = math.radians(a.wall_angle_deg)
            if theta_new >= limit:
                theta_new, omega_new = limit, min(0.0, omega_new)
            elif theta_new <= -limit:
                theta_new, omega_new = -limit, max(0.0, omega_new)

        ax.pos = theta_new / TWO_PI * n
        ax.vel = omega_new / TWO_PI * n

    def _thermal(self, ax: _SimAxis, dt: float) -> None:
        # Power stage: ~10 degC rise at 10 A continuous, time constant ~60 s.
        ax.fet_temp += (0.05 * ax.iq * ax.iq - (ax.fet_temp - 25.0) / 2.0) / 30.0 * dt
        ax.motor_temp += (0.08 * ax.iq * ax.iq - (ax.motor_temp - 25.0) / 2.5) / 120.0 * dt

    def _update_bus(self) -> None:
        a = self.actuator
        load = sum(ax.electrical_power for ax in self.axes) + 1.5  # board idle power
        self.ibus = load / max(self.vbus, 1.0)
        self.vbus = a.supply_voltage - a.supply_resistance * self.ibus
        if self.vbus < float(self._props["config.dc_bus_undervoltage_trip_level"].get()):
            self.system_error |= _SYSTEM_UNDER_VOLTAGE


def _coerce(value, kind: type):
    if kind is bool:
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true")
        return bool(value)
    if kind is int:
        return int(value)
    return float(value)


class SimulatorBackend(Backend):
    """Talks to a SimulatedODrive directly (no serial port involved)."""

    kind = "simulator"

    def __init__(self, sim: SimulatedODrive | None = None):
        self.sim = sim or SimulatedODrive()

    @property
    def description(self) -> str:
        return "Simulator (no hardware)"

    def connect(self) -> None:
        pass

    def close(self) -> None:
        pass

    def read(self, path: str):
        return self.sim.read(path)

    def write(self, path: str, value) -> None:
        self.sim.write(path, value)

    def call(self, path: str) -> None:
        self.sim.call(path)

    def identity(self) -> dict[str, str]:
        hw, fw = SimulatedODrive.HW, SimulatedODrive.FW
        return {
            "hardware": f"{hw[0]}.{hw[1]}-{hw[2]}V (simulated)",
            "firmware": f"{fw[0]}.{fw[1]}.{fw[2]}",
            "serial": f"{SimulatedODrive.SERIAL:012X}",
        }
