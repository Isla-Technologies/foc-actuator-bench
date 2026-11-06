"""An ODrive V3.6 axis (firmware 0.5.x), reached through one of the backends in ``backends/``."""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable

from ..backends.base import Backend, BackendError, PropertyUnavailable
from ..device import (
    ARM_TIMEOUT_S,
    ArmError,
    Capabilities,
    DeviceInfo,
    FastReading,
    MotorController,
    MotorParameters,
    Readiness,
    SlowReading,
)
from ..odrive_api import (
    MAX_PHASE_VOLTAGE_PER_BUS_VOLT,
    AxisPaths,
    AxisState,
    ControlMode,
    ErrorFlag,
    InputMode,
    SystemPaths,
    decode_errors,
)

log = logging.getLogger(__name__)


def _num(value, default=math.nan) -> float:
    return default if value is None else float(value)


class ODriveAxis(MotorController):
    """One axis (M0 or M1) of an ODrive. The ODrive takes torque in N·m and turns it into current
    with its own ``torque_constant``, so current commands are converted with that value."""

    family = "odrive"
    name = "ODrive"
    capabilities = Capabilities(watchdog=True, motor_calibration=True, write_torque_constant=True,
                                clear_errors=True, bus_voltage=True, driver_temperature=True,
                                torque_mode_speed_limit=True)

    def __init__(self, backend: Backend, axis: int = 0):
        super().__init__()
        self.backend = backend
        self.axis = axis
        self.p = AxisPaths(axis)
        self.kt_device: float | None = None   # the ODrive's torque_constant, read by readiness()
        self._unsupported: set[str] = set()
        self._original: dict[str, object] = {}
        self._changed: set[str] = set()
        self._watchdog_on = False

    # --- connection ------------------------------------------------------------------------
    @property
    def description(self) -> str:
        return self.backend.description

    @property
    def backend_kind(self) -> str:
        return self.backend.kind

    def close(self) -> None:
        self.backend.close()

    @property
    def device_messages(self) -> list[str]:
        return list(getattr(self.backend, "device_messages", []))

    # --- reading ---------------------------------------------------------------------------
    def read_optional(self, paths: list[str]) -> list:
        """Read paths that may not exist in this firmware; missing ones are remembered."""
        wanted = [p for p in paths if p not in self._unsupported]
        values = dict(zip(wanted, self.backend.read_many(wanted))) if wanted else {}
        result = []
        for path in paths:
            value = values.get(path)
            if path in values and value is None:
                self._unsupported.add(path)
                log.info("Property not available on this firmware/connection: %s", path)
            result.append(value)
        return result

    def read_info(self) -> DeviceInfo:
        s = SystemPaths
        ident = self.backend.identity()
        fw_major, fw_minor, fw_rev, hw_major, hw_minor, hw_variant = self.read_optional([
            s.fw_version_major, s.fw_version_minor, s.fw_version_revision,
            s.hw_version_major, s.hw_version_minor, s.hw_version_variant,
        ])
        fw = None
        if None not in (fw_major, fw_minor, fw_rev):
            fw = (int(fw_major), int(fw_minor), int(fw_rev))
        hardware = ident.get("hardware") or (
            f"{int(hw_major)}.{int(hw_minor)}-{int(hw_variant)}V" if None not in (hw_major, hw_minor, hw_variant) else "unknown"
        )
        firmware = ident.get("firmware") or (".".join(map(str, fw)) if fw else "unknown")
        serial = ident.get("serial", "")
        if not serial and self.backend.kind != "serial":  # ASCII replies truncate long numbers
            (number,) = self.read_optional([s.serial_number])
            serial = f"{int(number):012X}" if number is not None else ""
        (enabled,) = self.read_optional([self.p.motor_thermistor_enabled])
        self.motor_thermistor_enabled = bool(enabled)
        return DeviceInfo(self.backend.description, self.backend.kind, hardware, firmware, fw, serial or "unknown",
                          controller=self.name, family=self.family, capabilities=self.capabilities)

    def read_fast(self) -> FastReading:
        iq, iq_set, pos, vel = self.backend.read_many(self.p.fast)
        if iq is None or pos is None or vel is None:
            raise BackendError("incomplete reading")
        return FastReading(float(iq), _num(iq_set, 0.0), float(pos), float(vel))

    def read_slow(self) -> SlowReading:
        p = self.p
        regs = p.error_registers
        paths = [SystemPaths.vbus_voltage, SystemPaths.ibus, p.fet_temperature, p.current_state,
                 SystemPaths.error, *regs.values()]
        if self.motor_thermistor_enabled:
            paths.append(p.motor_temperature)
        values = self.read_optional(paths)
        vbus, ibus, fet, state, system_error = values[:5]
        raw = {"system": int(system_error or 0)}
        for (component, _), value in zip(regs.items(), values[5:9]):
            raw[component] = int(value or 0)
        errors = [flag for component, value in raw.items() for flag in decode_errors(component, value)]
        motor_temp = values[9] if self.motor_thermistor_enabled else None
        return SlowReading(_num(vbus), _num(ibus), _num(fet), _num(motor_temp),
                           None if state is None else int(state), errors, raw)

    def current_state(self) -> int | None:
        (state,) = self.read_optional([self.p.current_state])
        return None if state is None else int(state)

    def readiness(self) -> Readiness:
        p = self.p
        state, calibrated, ready, kt, current_lim, vel_limit, vel_gain, vbus = self.read_optional([
            p.current_state, p.motor_is_calibrated, p.encoder_is_ready, p.torque_constant,
            p.current_lim, p.vel_limit, p.vel_gain, SystemPaths.vbus_voltage,
        ])
        self.kt_device = float(kt) if kt else None
        vel_gain = None if vel_gain is None else float(vel_gain)
        kt_used = self._kt()
        return Readiness(
            state=None if state is None else int(state),
            motor_calibrated=None if calibrated is None else bool(calibrated),
            encoder_ready=None if ready is None else bool(ready),
            errors=self.read_errors(),
            kt_device=None if kt is None else float(kt),
            current_lim=None if current_lim is None else float(current_lim),
            vel_limit=None if vel_limit is None else float(vel_limit),
            vel_gain=vel_gain,
            vbus=None if vbus is None else float(vbus),
            speed_limiter_gain=vel_gain / kt_used if vel_gain and kt_used else None,
        )

    def config_snapshot(self) -> dict[str, object]:
        paths = [SystemPaths.vbus_voltage, SystemPaths.ibus, *self.p.config_snapshot]
        return dict(zip(paths, self.read_optional(paths)))

    def motor_parameters(self) -> MotorParameters:
        """The phase resistance and inductance the ODrive measured in its motor calibration."""
        p = self.p
        resistance, inductance, pole_pairs = self.read_optional([p.phase_resistance, p.phase_inductance,
                                                                 p.pole_pairs])
        return MotorParameters(float(resistance) if resistance else None,
                               float(inductance) if inductance else None,
                               int(pole_pairs) if pole_pairs else None)

    @property
    def modulation_ratio(self) -> float:
        return MAX_PHASE_VOLTAGE_PER_BUS_VOLT

    def _kt(self) -> float:
        """N·m per amp the ODrive uses to turn torque commands into current."""
        if self.kt_device and self.kt_device > 0:
            return self.kt_device
        if self.motor_kt and self.motor_kt > 0:
            return self.motor_kt
        raise BackendError("the ODrive's torque_constant couldn't be read")

    # --- configuration we may change ---------------------------------------------------------
    def remember_config(self) -> None:
        paths = self.p.restorable
        self._original = {p: v for p, v in zip(paths, self.read_optional(paths)) if v is not None}
        self._changed.clear()

    def _set(self, path: str, value) -> None:
        self.backend.write(path, value)
        if path in self._original:
            self._changed.add(path)

    def _try_set(self, path: str, value) -> None:
        """Like _set, for settings that older 0.5.x firmware may not have."""
        try:
            self._set(path, value)
        except PropertyUnavailable as exc:
            log.info("Skipping %s: %s", path, exc)

    def restore_config(self) -> None:
        """Put back every setting this session changed (runtime only, nothing is saved)."""
        changed = [p for p in self.p.restorable if p in self._changed]
        if not changed:
            return
        for path in changed:
            try:
                self.backend.write(path, self._original[path])
            except BackendError as exc:
                log.warning("Could not restore %s: %s", path, exc)
        try:
            readback = dict(zip(changed, self.backend.read_many(changed)))
        except BackendError as exc:
            log.warning("Could not verify restored settings: %s", exc)
            return
        for path in changed:
            original, now = self._original[path], readback.get(path)
            if now is None or original is None:
                continue
            if isinstance(original, float) and math.isinf(original) and not math.isinf(float(now)):
                self.backend.write(path, 1e10)  # firmware didn't accept "inf"; use a huge limit instead
            elif not math.isclose(float(now), float(original), rel_tol=1e-4, abs_tol=1e-6):
                log.warning("%s is %s after restore (expected %s)", path, now, original)
        self._changed.clear()

    # --- control ---------------------------------------------------------------------------
    def arm(
        self,
        mode: ControlMode,
        *,
        vel_limit: float,
        current_lim: float,
        torque_mode_vel_limit: bool = True,
        watchdog_timeout: float | None = None,
        should_abort: Callable[[], bool] | None = None,
    ) -> None:
        """Enter closed-loop control in ``mode`` without any jump in position or torque."""
        errors = self.read_errors()
        if errors:
            raise ArmError("The ODrive has errors. Clear them first.", errors)
        p = self.p
        already_armed = self.current_state() == AxisState.CLOSED_LOOP_CONTROL
        self._prepare_inputs(mode)
        self._set(p.vel_limit, float(vel_limit))
        self._try_set(p.torque_lim, float(current_lim) * self._kt())
        self._try_set(p.enable_torque_mode_vel_limit, bool(torque_mode_vel_limit))
        self._set(p.input_mode, int(InputMode.PASSTHROUGH))
        self._set(p.control_mode, int(mode))
        if watchdog_timeout:
            self.enable_watchdog(watchdog_timeout)
        if already_armed:
            return
        self.backend.write(p.requested_state, int(AxisState.CLOSED_LOOP_CONTROL))
        deadline = time.monotonic() + ARM_TIMEOUT_S
        while True:
            self.feed_watchdog()  # nobody else feeds it while we wait here
            state = self.current_state()
            if state == AxisState.CLOSED_LOOP_CONTROL:
                return
            if should_abort and should_abort():
                self.disarm()
                raise ArmError("Stopped while arming.")
            if time.monotonic() > deadline or state == AxisState.IDLE and self.read_errors():
                errors = self.read_errors()
                self.disarm()
                hint = "" if errors else " (no error reported; is the motor calibrated?)"
                raise ArmError("The ODrive did not enter closed-loop control" + hint, errors)
            time.sleep(0.02)

    def _prepare_inputs(self, mode: ControlMode) -> None:
        """Set safe setpoints before switching mode: zero torque/speed, hold current position."""
        p = self.p
        self.backend.write(p.input_torque, 0.0)
        self.backend.write(p.input_vel, 0.0)
        if mode == ControlMode.POSITION:
            pos = self.backend.read(p.pos_estimate)
            self.backend.write(p.input_pos, float(pos))

    def change_mode(self, mode: ControlMode, *, torque_mode_vel_limit: bool = True) -> None:
        self._prepare_inputs(mode)
        self._try_set(self.p.enable_torque_mode_vel_limit, bool(torque_mode_vel_limit))
        self._set(self.p.control_mode, int(mode))

    def set_limits(self, *, vel_limit: float, current_lim: float) -> None:
        self._set(self.p.vel_limit, float(vel_limit))
        self._try_set(self.p.torque_lim, float(current_lim) * self._kt())

    def disarm(self) -> None:
        self.backend.write(self.p.requested_state, int(AxisState.IDLE))
        self.backend.write(self.p.input_torque, 0.0)
        self.backend.write(self.p.input_vel, 0.0)
        self.disable_watchdog()

    def estop(self) -> None:
        for _ in range(2):
            try:
                self.backend.write(self.p.requested_state, int(AxisState.IDLE))
                break
            except BackendError as exc:
                log.error("E-stop write failed: %s", exc)
        try:
            self.backend.write(self.p.input_torque, 0.0)
            self.disable_watchdog()
        except BackendError:
            pass

    def set_current(self, amps: float) -> None:
        self.backend.write(self.p.input_torque, float(amps) * self._kt())

    def set_velocity(self, turns_per_s: float) -> None:
        self.backend.write(self.p.input_vel, float(turns_per_s))

    def set_position(self, turns: float, vel_ff: float | None = None) -> None:
        if vel_ff is not None:
            self.backend.write(self.p.input_vel, float(vel_ff))
        self.backend.write(self.p.input_pos, float(turns))

    # --- watchdog --------------------------------------------------------------------------
    def enable_watchdog(self, timeout_s: float) -> None:
        """Set the timeout, feed, then enable. The firmware's feed() loads its countdown from the
        current timeout, so feeding before setting it (default 0) would expire it instantly."""
        try:
            self._set(self.p.watchdog_timeout, float(timeout_s))
            self.backend.call(self.p.watchdog_feed)
            self._set(self.p.enable_watchdog, True)
            self.backend.call(self.p.watchdog_feed)
            self._watchdog_on = True
        except PropertyUnavailable as exc:
            log.warning("Watchdog not available, continuing without it: %s", exc)

    def feed_watchdog(self) -> None:
        if self._watchdog_on:
            self.backend.call(self.p.watchdog_feed)

    def disable_watchdog(self) -> None:
        if self._watchdog_on:
            self._watchdog_on = False
            self._set(self.p.enable_watchdog, False)

    @property
    def watchdog_on(self) -> bool:
        return self._watchdog_on

    # --- maintenance -----------------------------------------------------------------------
    def clear_errors(self) -> list[ErrorFlag]:
        """Clear all errors; returns the errors that remain."""
        try:
            self.backend.call(SystemPaths.clear_errors)
        except PropertyUnavailable:
            pass
        remaining = self.read_errors()
        if remaining:  # older firmware: clear the registers directly
            for path in [*self.p.error_registers.values(), SystemPaths.error]:
                try:
                    self.backend.write(path, 0)
                except BackendError:
                    pass
            remaining = self.read_errors()
        return remaining

    def start_calibration(self) -> None:
        self.backend.write(self.p.requested_state, int(AxisState.FULL_CALIBRATION_SEQUENCE))

    def apply_torque_constant(self, kt: float) -> None:
        """Write torque_constant (runtime only, lost on reboot unless saved in odrivetool)."""
        self.backend.write(self.p.torque_constant, float(kt))
        self.kt_device = float(kt)
