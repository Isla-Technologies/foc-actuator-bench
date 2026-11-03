"""The acquisition worker: owns the ODrive connection on a background thread.

It polls the fast values (current, position, speed) at the sample rate and the slow ones
(voltage, temperature, state, errors) a few times per second, turns them into torque, runs the
automated procedures, feeds the ODrive watchdog while the motor is armed, and executes commands
queued from the UI. Every interaction with the device happens on this one thread.
"""

from __future__ import annotations

import logging
import math
import queue
import threading
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass, replace

from .backends.base import Backend, BackendError, ConnectionLost
from .backends.native_backend import NativeBackend, odrive_library_version
from .backends.serial_backend import SerialBackend
from .backends.sim_serial import SimulatedSerialPort
from .backends.simulator import SimActuator, SimulatedODrive, SimulatorBackend
from .device import ArmError, DeviceInfo, ODriveAxis, Readiness, SlowReading
from .odrive_api import AxisState, ControlMode
from .procedures import Procedure, ProcedureError, ProcedureStatus
from .samples import Limits, Sample, SampleBatch
from .torque import TorqueModel

log = logging.getLogger(__name__)

CONNECTION_MODES = {
    "auto": "Auto (recommended)",
    "serial": "USB serial (no driver needed)",
    "native": "Official ODrive library (USB)",
    "simulator": "Simulator (no hardware)",
}

WATCHDOG_FEED_INTERVAL_S = 0.1
MAX_CONSECUTIVE_FAILURES = 10


@dataclass
class ConnectRequest:
    mode: str = "auto"                  # auto | serial | native | simulator | simulator-serial
    port: str | None = None
    axis: int = 0
    sim_actuator: SimActuator | None = None
    backend: Backend | None = None      # pre-built backend (tests)


@dataclass
class ConnectionStatus:
    state: str                          # connecting | connected | disconnected | failed
    message: str = ""
    info: DeviceInfo | None = None
    readiness: Readiness | None = None


class WorkerListener:
    """Receives worker events. All methods are called on the worker thread."""

    def on_connection(self, status: ConnectionStatus) -> None: ...
    def on_samples(self, batch: SampleBatch) -> None: ...
    def on_slow(self, slow: SlowReading) -> None: ...
    def on_readiness(self, readiness: Readiness) -> None: ...
    def on_armed(self, armed: bool, mode: ControlMode | None) -> None: ...
    def on_procedure(self, status: ProcedureStatus) -> None: ...
    def on_message(self, level: str, text: str) -> None: ...
    def on_snapshot(self, snapshot: dict) -> None: ...


def make_sample(model: TorqueModel, t: float, fast, slow: SlowReading | None, pos_zero: float,
                state: int | None) -> Sample:
    """Turn raw readings into a Sample with torque and output-side values."""
    vbus = slow.vbus if slow else math.nan
    ibus = slow.ibus if slow else math.nan
    return Sample(
        t=t, iq=fast.iq, iq_set=fast.iq_set, pos=fast.pos, vel=fast.vel,
        torque_motor=model.motor_torque(fast.iq),
        torque_out=model.output_torque(fast.iq),
        torque_cmd_out=model.output_torque(fast.iq_set),
        angle_out_deg=model.output_angle_deg(fast.pos - pos_zero),
        speed_out_rpm=model.output_speed_rpm(fast.vel),
        vbus=vbus, ibus=ibus, power_elec=vbus * ibus,
        power_mech=model.mechanical_power(fast.iq, fast.vel),
        fet_temp=slow.fet_temp if slow else math.nan,
        motor_temp=slow.motor_temp if slow else math.nan,
        state=state if state is not None else -1,
    )


def open_backend(request: ConnectRequest) -> tuple[Backend, SimulatedODrive | None]:
    """Create and connect the backend for a connection request."""
    if request.backend is not None:
        request.backend.connect()
        return request.backend, getattr(request.backend, "sim", None)
    mode = request.mode
    if mode == "simulator":
        sim = SimulatedODrive(request.sim_actuator or SimActuator())
        backend = SimulatorBackend(sim)
        backend.connect()
        return backend, sim
    if mode == "simulator-serial":
        sim = SimulatedODrive(request.sim_actuator or SimActuator())
        backend = SerialBackend("SIM-SERIAL", opener=lambda _port: SimulatedSerialPort(sim))
        backend.connect()
        return backend, sim
    if mode == "serial":
        backend = SerialBackend(request.port or None)
        backend.connect()
        return backend, None
    if mode == "native":
        backend = NativeBackend()
        backend.connect()
        return backend, None
    if mode == "auto":
        return _open_auto(request), None
    raise BackendError(f"Unknown connection mode {mode!r}")


def _open_auto(request: ConnectRequest) -> Backend:
    """USB serial first (fast, no driver); the official library as fallback."""
    problems = []
    has_library = odrive_library_version() is not None
    try:
        serial_backend = SerialBackend(request.port or None)
        serial_backend.connect()
    except BackendError as exc:
        problems.append(f"USB serial: {exc}")
    else:
        fw = serial_backend.read_many(["fw_version_major", "fw_version_minor", "fw_version_revision"])
        buggy = fw[:2] == [0, 5] and fw[2] in (2, 3)
        if not (buggy and has_library):
            return serial_backend
        try:  # 0.5.2/0.5.3 reject some ASCII writes: prefer the official library there
            native = NativeBackend(timeout=5.0)
            native.connect()
            serial_backend.close()
            return native
        except BackendError:
            return serial_backend
    if has_library:
        try:
            native = NativeBackend(timeout=5.0)
            native.connect()
            return native
        except BackendError as exc:
            problems.append(f"official library: {exc}")
    else:
        problems.append("official library: not installed")
    raise BackendError("Couldn't find an ODrive. " + " | ".join(problems))


class _ProcedureContext:
    """What a procedure may do (see procedures.ProcedureContext)."""

    def __init__(self, worker: AcquisitionWorker):
        self._w = worker

    @property
    def model(self) -> TorqueModel:
        return self._w.model

    @property
    def limits(self) -> Limits:
        return self._w.limits

    @property
    def vel_gain(self) -> float | None:
        r = self._w.readiness
        return r.vel_gain if r else None

    @property
    def latest_slow(self) -> SlowReading | None:
        return self._w.latest_slow

    @property
    def armed(self) -> bool:
        return self._w.armed

    @property
    def armed_mode(self) -> ControlMode | None:
        return self._w.armed_mode

    def arm(self, mode: ControlMode, *, vel_limit: float | None = None, max_current: float | None = None,
            torque_mode_vel_limit: bool = True) -> None:
        self._w._arm(mode, vel_limit, max_current, torque_mode_vel_limit)

    def set_torque_out(self, torque_out: float) -> None:
        if self._w.armed and self._w._dev:
            self._w._dev.set_torque(self._w.model.odrive_input_torque(torque_out))

    def set_position_turns(self, turns: float, vel_ff: float = 0.0) -> None:
        if self._w.armed and self._w._dev:
            self._w._dev.set_position(turns, vel_ff)

    def describe_errors(self) -> str:
        slow = self._w.latest_slow
        if not slow or not slow.errors:
            return ""
        return " Errors: " + ", ".join(f"{e.name} ({e.explanation})" for e in slow.errors)


class AcquisitionWorker:
    def __init__(
        self,
        listener: WorkerListener,
        *,
        clock: Callable[[], float] = time.monotonic,
        sample_rate_hz: float = 100.0,
        slow_interval_s: float = 0.2,
        emit_interval_s: float = 0.033,
        watchdog_timeout_s: float | None = 1.0,
    ):
        self.listener = listener
        self._clock = clock
        self._period = 1.0 / sample_rate_hz
        self._slow_interval = slow_interval_s
        self._emit_interval = emit_interval_s
        self._watchdog_timeout = watchdog_timeout_s
        self._commands: queue.Queue[Callable[[], None]] = queue.Queue()
        self._estop = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._ctx = _ProcedureContext(self)

        # Everything below is only touched on the worker thread.
        self._dev: ODriveAxis | None = None
        self._backend: Backend | None = None
        self.sim: SimulatedODrive | None = None
        self.info: DeviceInfo | None = None
        self.readiness: Readiness | None = None
        self.model = TorqueModel(kt_motor=0.0827)
        self.limits = Limits(max_current=10.0, max_motor_speed=2.0)
        self.armed = False
        self.armed_mode: ControlMode | None = None
        self.latest_slow: SlowReading | None = None
        self._state: int | None = None
        self._latest: Sample | None = None
        self._pending: list[Sample] = []
        self._proc: Procedure | None = None
        self._t0 = 0.0
        self._last_slow = -math.inf
        self._last_emit = -math.inf
        self._last_feed = -math.inf
        self._pos_zero = 0.0
        self._position_target = 0.0
        self._failures = 0
        self._rate_count = 0
        self._rate_t0: float | None = None
        self.sample_rate = 0.0

    # --- lifecycle -------------------------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="odrive-worker", daemon=True)
        self._thread.start()

    def shutdown(self, timeout: float = 3.0) -> None:
        """Disconnect cleanly (motor idle, settings restored) and stop the thread."""
        if self._thread is None:
            self._do_disconnect("Closed")
            return
        self._commands.put(self._shutdown_now)
        self._thread.join(timeout)

    def _shutdown_now(self) -> None:
        self._do_disconnect("Closed")
        self._stop.set()

    @property
    def connected(self) -> bool:
        return self._dev is not None

    # --- thread-safe API (queued) ----------------------------------------------------------
    def _queue(self, func: Callable, *args, **kwargs) -> None:
        self._commands.put(lambda: func(*args, **kwargs))

    def connect(self, request: ConnectRequest) -> None:
        self._queue(self._do_connect, request)

    def disconnect(self) -> None:
        self._queue(self._do_disconnect, "Disconnected")

    def estop(self) -> None:
        """Emergency stop: handled before anything else on the next cycle."""
        self._estop.set()

    def set_model(self, model: TorqueModel) -> None:
        self._queue(self._do_set_model, model)

    def set_limits(self, limits: Limits) -> None:
        self._queue(self._do_set_limits, limits)

    def set_sample_rate(self, hz: float) -> None:
        self._queue(setattr, self, "_period", 1.0 / max(1.0, hz))

    def set_watchdog(self, timeout_s: float | None) -> None:
        self._queue(setattr, self, "_watchdog_timeout", timeout_s)

    def arm(self, mode: ControlMode) -> None:
        self._queue(self._do_arm, mode)

    def disarm(self) -> None:
        self._queue(self._do_disarm)

    def change_mode(self, mode: ControlMode) -> None:
        self._queue(self._do_change_mode, mode)

    def set_output_torque(self, torque_out: float) -> None:
        self._queue(self._do_set_torque, torque_out)

    def set_output_speed(self, rpm: float) -> None:
        self._queue(self._do_set_speed, rpm)

    def set_output_angle(self, angle_deg: float) -> None:
        self._queue(self._do_set_angle, angle_deg)

    def jog(self, delta_deg: float) -> None:
        self._queue(self._do_jog, delta_deg)

    def zero_angle(self) -> None:
        self._queue(self._do_zero_angle)

    def clear_errors(self) -> None:
        self._queue(self._do_clear_errors)

    def start_motor_calibration(self) -> None:
        self._queue(self._do_calibrate)

    def apply_torque_constant(self, kt: float) -> None:
        self._queue(self._do_apply_kt, kt)

    def refresh_readiness(self) -> None:
        self._queue(self._do_refresh_readiness)

    def start_procedure(self, procedure: Procedure) -> None:
        self._queue(self._do_start_procedure, procedure)

    def stop_procedure(self) -> None:
        self._queue(self._do_stop_procedure)

    def configure_simulator(self, **changes) -> None:
        self._queue(self._do_configure_sim, changes)

    def request_snapshot(self) -> None:
        self._queue(self._do_snapshot)

    # --- main loop -------------------------------------------------------------------------
    def _run(self) -> None:
        next_tick = time.monotonic()
        while not self._stop.is_set():
            try:
                self.step()
            except Exception:  # never let the worker die silently
                log.exception("Unexpected error in the measurement loop")
                self._emit("on_message", "error", "Internal error in the measurement loop (see log). Motor switched off.")
                self._safe_estop()
            if self._dev is None:
                self._wait_for_command(0.1)
                next_tick = time.monotonic()
                continue
            next_tick += self._period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def _wait_for_command(self, timeout: float) -> None:
        try:
            command = self._commands.get(timeout=timeout)
        except queue.Empty:
            return
        self._execute(command)

    def _run_commands(self) -> None:
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            self._execute(command)
            if self._estop.is_set():
                return

    def _execute(self, command: Callable[[], None]) -> None:
        try:
            command()
        except ConnectionLost as exc:
            self._lost(str(exc))
        except BackendError as exc:
            log.warning("Command failed: %s", exc)
            self._emit("on_message", "error", f"Command failed: {exc}")

    def step(self) -> None:
        """One cycle: emergency stop, queued commands, one sample, procedure update."""
        if self._estop.is_set():
            self._estop.clear()
            self._do_estop()
        self._run_commands()
        if self._dev is None:
            return
        now = self._clock()
        try:
            fast = self._dev.read_fast()
            if now - self._last_slow >= self._slow_interval:
                self._read_slow(now)
            if self._dev is not None and self._dev.watchdog_on and now - self._last_feed >= WATCHDOG_FEED_INTERVAL_S:
                self._dev.feed_watchdog()
                self._last_feed = now
        except ConnectionLost as exc:
            self._lost(str(exc))
            return
        except BackendError as exc:
            self._failures += 1
            if self._failures >= MAX_CONSECUTIVE_FAILURES:
                self._lost(f"no valid replies ({exc})")
            elif self._failures == 3:
                self._emit("on_message", "warning", f"Communication hiccup: {exc}")
            return
        if self._dev is None:
            return
        self._failures = 0
        sample = self._derive(now - self._t0, fast)
        self._latest = sample
        self._pending.append(sample)
        self._count_rate(now)
        if self._proc is not None:
            self._update_procedure(sample)
        if now - self._last_emit >= self._emit_interval:
            self._flush(now)

    # --- samples ---------------------------------------------------------------------------
    def _read_slow(self, now: float) -> None:
        slow = self._dev.read_slow()
        slow.sample_rate_hz = self.sample_rate
        self.latest_slow = slow
        self._last_slow = now
        if slow.state is not None:
            self._state = slow.state
        if self.armed and slow.state is not None and slow.state != AxisState.CLOSED_LOOP_CONTROL:
            self.armed, self.armed_mode = False, None
            try:
                self._dev.disable_watchdog()
            except BackendError:
                pass
            reasons = ", ".join(e.name for e in slow.errors) or "no error reported"
            self._emit("on_armed", False, None)
            self._emit("on_message", "error", f"The ODrive switched the motor off ({reasons}).")
        self._emit("on_slow", slow)

    def _derive(self, t: float, fast) -> Sample:
        return make_sample(self.model, t, fast, self.latest_slow, self._pos_zero, self._state)

    def _count_rate(self, now: float) -> None:
        if self._rate_t0 is None:
            self._rate_t0, self._rate_count = now, 0
            return
        self._rate_count += 1
        if now - self._rate_t0 >= 1.0:
            self.sample_rate = self._rate_count / (now - self._rate_t0)
            self._rate_t0, self._rate_count = now, 0

    def _flush(self, now: float) -> None:
        self._last_emit = now
        if self._pending:
            batch = SampleBatch.from_samples(self._pending)
            self._pending = []
            self._emit("on_samples", batch)
        if self._proc is not None:
            self._emit("on_procedure", replace(self._proc.status))

    # --- connection ------------------------------------------------------------------------
    def _do_connect(self, request: ConnectRequest) -> None:
        if self._dev is not None:
            self._do_disconnect("Reconnecting")
        self._emit("on_connection", ConnectionStatus("connecting", "Connecting…"))
        backend = None
        try:
            backend, sim = open_backend(request)
            dev = ODriveAxis(backend, request.axis)
            info = dev.read_info()
            dev.remember_config()
            readiness = dev.readiness()
            fast = dev.read_fast()
        except BackendError as exc:
            if backend is not None:
                backend.close()
            self._emit("on_connection", ConnectionStatus("failed", str(exc)))
            return
        self._dev, self._backend, self.sim = dev, backend, sim
        self.info, self.readiness = info, readiness
        self.model.kt_odrive = readiness.kt_odrive
        self._state = readiness.state
        self.armed, self.armed_mode = False, None
        self.latest_slow = None
        self._latest = None
        self._pending = []
        self._t0 = self._clock()
        self._last_slow = self._last_emit = self._last_feed = -math.inf
        self._failures = 0
        self._rate_t0 = None
        self._pos_zero = self._position_target = fast.pos
        self._emit("on_connection", ConnectionStatus("connected", f"Connected via {backend.description}", info, readiness))
        if not info.firmware_supported:
            self._emit("on_message", "warning",
                       f"Firmware {info.firmware} is not 0.5.x. This app targets ODrive v3.6 firmware 0.5.x; some values may be missing.")
        if backend.kind == "serial" and info.firmware_has_ascii_bugs:
            self._emit("on_message", "warning",
                       f"Firmware {info.firmware} has ASCII-protocol bugs that can block arming over USB serial. "
                       "Use the 'Official ODrive library' connection or update the firmware to 0.5.6.")

    def _do_disconnect(self, reason: str) -> None:
        if self._dev is None:
            return
        if self._proc is not None:
            self._proc.finish(False, "Disconnected.", None)
            self._emit("on_procedure", replace(self._proc.status))
            self._proc = None
        try:
            if self.armed or self._state == AxisState.CLOSED_LOOP_CONTROL:
                self._dev.disarm()
            self._dev.restore_config()
        except BackendError as exc:
            log.warning("Problem while disconnecting: %s", exc)
        self._close()
        self._emit("on_armed", False, None)
        self._emit("on_connection", ConnectionStatus("disconnected", reason))

    def _lost(self, message: str) -> None:
        log.error("Connection lost: %s", message)
        if self._proc is not None:
            self._proc.finish(False, "Connection lost.", None)
            self._emit("on_procedure", replace(self._proc.status))
            self._proc = None
        was_armed = self.armed
        self._close()
        self._emit("on_armed", False, None)
        text = f"Connection lost: {message}."
        if was_armed:
            text += " The ODrive's watchdog switches the motor off within about a second."
        self._emit("on_connection", ConnectionStatus("disconnected", text))

    def _close(self) -> None:
        if self._backend is not None:
            try:
                self._backend.close()
            except Exception:
                pass
        self._dev = None
        self._backend = None
        self.sim = None
        self.armed, self.armed_mode = False, None
        self._pending = []

    # --- arming and control ----------------------------------------------------------------
    def _arm(self, mode: ControlMode, vel_limit: float | None, max_current: float | None,
             torque_mode_vel_limit: bool) -> None:
        dev = self._require_device()
        vel_limit = self.limits.max_motor_speed if vel_limit is None else vel_limit
        max_current = self.limits.max_current if max_current is None else min(max_current, self.limits.max_current)
        torque_lim = self.model.odrive_torque_limit(max_current)
        if self.armed:
            dev.set_limits(vel_limit=vel_limit, torque_lim=torque_lim)
            dev.change_mode(mode, torque_mode_vel_limit=torque_mode_vel_limit)
        else:
            dev.arm(mode, vel_limit=vel_limit, torque_lim=torque_lim, torque_mode_vel_limit=torque_mode_vel_limit,
                    watchdog_timeout=self._watchdog_timeout, should_abort=self._estop.is_set)
        self.armed, self.armed_mode = True, mode
        self._state = int(AxisState.CLOSED_LOOP_CONTROL)
        self._last_feed = self._clock()
        if mode == ControlMode.POSITION and self._latest is not None:
            self._position_target = self._latest.pos
        self._emit("on_armed", True, mode)

    def _do_arm(self, mode: ControlMode) -> None:
        if self._proc is not None:
            self._emit("on_message", "warning", "A test is running; stop it first.")
            return
        try:
            self._arm(mode, None, None, True)
        except ArmError as exc:
            details = "; ".join(f"{e.name}: {e.explanation}" for e in exc.errors)
            self._emit("on_message", "error", f"{exc}{' ' + details if details else ''}")
            self._emit("on_armed", False, None)
            return
        if mode == ControlMode.TORQUE:
            self._warn_torque_mode_cap()

    def _warn_torque_mode_cap(self) -> None:
        """In torque mode the ODrive limits torque at standstill to vel_limit * vel_gain."""
        r = self.readiness
        if not r or not r.vel_gain:
            return
        cap_out = self.model.output_torque(self.limits.max_motor_speed * r.vel_gain / (self.model.kt_odrive or self.model.kt_motor))
        full = self.model.max_output_torque(self.limits.max_current)
        if cap_out < 0.95 * full:
            self._emit("on_message", "info",
                       f"Torque mode: at standstill the ODrive caps torque at about {cap_out:.3g} N·m "
                       f"(speed limit × vel_gain). Raise the speed limit in Setup for more, or use the Stall test, "
                       f"which handles this automatically.")

    def _do_disarm(self) -> None:
        if self._proc is not None:
            self._proc.stop(self._ctx)
            self._end_procedure(force_disarm=True)
            return
        dev = self._require_device()
        dev.disarm()
        self.armed, self.armed_mode = False, None
        self._state = int(AxisState.IDLE)
        self._emit("on_armed", False, None)

    def _do_estop(self) -> None:
        self._safe_estop()
        if self._proc is not None:
            self._proc.finish(False, "Emergency stop.", None)
            self._emit("on_procedure", replace(self._proc.status))
            self._proc = None
        self.armed, self.armed_mode = False, None
        self._state = int(AxisState.IDLE) if self._dev else self._state
        self._emit("on_armed", False, None)
        self._emit("on_message", "warning", "STOP: motor switched off.")

    def _safe_estop(self) -> None:
        if self._dev is not None:
            try:
                self._dev.estop()
            except Exception:
                log.exception("E-stop failed")

    def _do_change_mode(self, mode: ControlMode) -> None:
        if not self.armed or self._proc is not None:
            return
        self._require_device().change_mode(mode)
        self.armed_mode = mode
        if mode == ControlMode.POSITION and self._latest is not None:
            self._position_target = self._latest.pos
        self._emit("on_armed", True, mode)
        if mode == ControlMode.TORQUE:
            self._warn_torque_mode_cap()

    def _manual_ok(self, mode: ControlMode) -> bool:
        return self.armed and self.armed_mode == mode and self._proc is None and self._dev is not None

    def _do_set_torque(self, torque_out: float) -> None:
        if not self._manual_ok(ControlMode.TORQUE):
            return
        limit = self.model.max_output_torque(self.limits.max_current)
        torque_out = max(-limit, min(limit, torque_out))
        self._dev.set_torque(self.model.odrive_input_torque(torque_out))

    def _do_set_speed(self, rpm: float) -> None:
        if not self._manual_ok(ControlMode.VELOCITY):
            return
        turns_s = self.model.motor_turns_per_s_from_output_rpm(rpm)
        limit = self.limits.max_motor_speed
        self._dev.set_velocity(max(-limit, min(limit, turns_s)))

    def _do_set_angle(self, angle_deg: float) -> None:
        if not self._manual_ok(ControlMode.POSITION):
            return
        self._position_target = self._pos_zero + self.model.motor_turns_from_output_deg(angle_deg)
        self._dev.set_position(self._position_target, 0.0)

    def _do_jog(self, delta_deg: float) -> None:
        if not self._manual_ok(ControlMode.POSITION):
            return
        self._position_target += self.model.motor_turns_from_output_deg(delta_deg)
        self._dev.set_position(self._position_target, 0.0)

    def _do_zero_angle(self) -> None:
        if self._latest is not None:
            self._pos_zero = self._latest.pos

    def _do_set_model(self, model: TorqueModel) -> None:
        kt_odrive = self.readiness.kt_odrive if self.readiness else model.kt_odrive
        self.model = replace(model, kt_odrive=kt_odrive)

    def _do_set_limits(self, limits: Limits) -> None:
        self.limits = limits
        if self.armed and self._proc is None and self._dev is not None:
            self._dev.set_limits(vel_limit=limits.max_motor_speed,
                                 torque_lim=self.model.odrive_torque_limit(limits.max_current))

    # --- maintenance -----------------------------------------------------------------------
    def _do_clear_errors(self) -> None:
        remaining = self._require_device().clear_errors()
        if remaining:
            names = ", ".join(e.name for e in remaining)
            self._emit("on_message", "warning", f"Some errors came back right away: {names}")
        else:
            self._emit("on_message", "info", "Errors cleared.")
        self._do_refresh_readiness()

    def _do_calibrate(self) -> None:
        dev = self._require_device()
        if self.armed:
            self._do_disarm()
        dev.start_calibration()
        self._emit("on_message", "info",
                   "Motor calibration started: the motor beeps and turns slowly for a few seconds. "
                   "Keep the output free to move.")

    def _do_apply_kt(self, kt: float) -> None:
        self._require_device().apply_torque_constant(kt)
        self._emit("on_message", "info", f"torque_constant set to {kt:.5f} N·m/A on the ODrive (until it reboots).")
        self._do_refresh_readiness()

    def _do_refresh_readiness(self) -> None:
        dev = self._require_device()
        self.readiness = dev.readiness()
        self.model.kt_odrive = self.readiness.kt_odrive
        self._emit("on_readiness", self.readiness)

    def _do_configure_sim(self, changes: dict) -> None:
        if self.sim is not None:
            self.sim.update_actuator(**changes)

    def _do_snapshot(self) -> None:
        snapshot: dict = {"connected": self._dev is not None, "sample_rate_hz": round(self.sample_rate, 1),
                          "model": asdict(self.model), "limits": asdict(self.limits), "armed": self.armed}
        if self._dev is not None:
            snapshot["info"] = asdict(self.info) if self.info else None
            if self.readiness:
                readiness = asdict(self.readiness)
                readiness["errors"] = [str(e) for e in self.readiness.errors]
                snapshot["readiness"] = readiness
            try:
                snapshot["config"] = self._dev.config_snapshot()
            except BackendError as exc:
                snapshot["config"] = f"unavailable ({exc})"
            snapshot["device_messages"] = list(getattr(self._backend, "device_messages", []))
        self._emit("on_snapshot", snapshot)

    # --- procedures ------------------------------------------------------------------------
    def _do_start_procedure(self, procedure: Procedure) -> None:
        if self._dev is None:
            procedure.finish(False, "Not connected.", None)
            self._emit("on_procedure", replace(procedure.status))
            return
        if self._proc is not None:
            self._emit("on_message", "warning", "A test is already running.")
            return
        sample = self._latest or self._derive(self._clock() - self._t0, self._dev.read_fast())
        try:
            procedure.begin(self._ctx, sample)
        except (ProcedureError, ArmError) as exc:
            details = "".join(f" {e.name}: {e.explanation}" for e in getattr(exc, "errors", []))
            procedure.finish(False, f"{exc}{details}", None)
            self._emit("on_procedure", replace(procedure.status))
            if self.armed and not procedure.keep_armed:
                self._do_disarm()
            return
        self._proc = procedure
        self._emit("on_procedure", replace(procedure.status))

    def _update_procedure(self, sample: Sample) -> None:
        proc = self._proc
        try:
            proc.update(self._ctx, sample)
        except ConnectionLost as exc:
            self._lost(str(exc))
            return
        except BackendError as exc:
            proc.finish(False, f"Communication error: {exc}", None)
        if proc.status.done:
            self._end_procedure()

    def _do_stop_procedure(self) -> None:
        if self._proc is None:
            return
        self._proc.stop(self._ctx)
        self._end_procedure()

    def _end_procedure(self, force_disarm: bool = False) -> None:
        proc, self._proc = self._proc, None
        if proc is None:
            return
        if (force_disarm or not proc.keep_armed) and self.armed and self._dev is not None:
            self._dev.disarm()
            self.armed, self.armed_mode = False, None
            self._state = int(AxisState.IDLE)
            self._emit("on_armed", False, None)
        self._emit("on_procedure", replace(proc.status))

    # --- helpers ---------------------------------------------------------------------------
    def _require_device(self) -> ODriveAxis:
        if self._dev is None:
            raise BackendError("not connected")
        return self._dev

    def _emit(self, method: str, *args) -> None:
        try:
            getattr(self.listener, method)(*args)
        except Exception:
            log.exception("Listener %s failed", method)
