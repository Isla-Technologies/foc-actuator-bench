"""A motor on a SimpleFOC board (an ESP32 FOC driver, the SimpleFOC Shield, B-G431B-ESC1…).

Works with any SimpleFOC sketch that registers its motor with Commander, using only the standard
motor commands (letter ``M`` by default): ``E`` enable, ``C`` motion control, ``T`` torque control,
``L`` limits, ``MG`` monitoring values, ``R``/``I`` phase resistance and inductance, ``WT``
modulation. Units on the board: amps, rad and rad/s at the motor shaft.

The bench firmware in ``firmware/simplefoc_bench`` adds a command (label ``bench``, letter ``B``)
with what plain Commander lacks:

* ``BI`` info: ``key=value;…`` (``fw``, ``board``, ``pp`` pole pairs, ``imax`` the firmware's hard
  current limit, ``isense`` the current the sensors can measure, ``vbus=measured|set``,
  ``temp``/``mtemp`` 1 when a driver/motor temperature sensor is fitted, ``id``)
* ``BF`` fast values: ``iq;iq_setpoint;velocity;angle``
* ``BS`` slow values: ``vbus;driver_temp;motor_temp;motor_status;faults;enabled``
* ``BW<s>`` watchdog timeout (0 = off), ``BK`` feed it, ``BC`` clear faults, ``BA`` re-run the
  sensor alignment
* ``BO<rad/s>`` overspeed trip: the firmware switches the motor off above this speed (0 = off).
  SimpleFOC doesn't limit the speed in torque mode, and the app's own check only runs at its
  sample rate, too slowly to catch an unloaded motor

Torque can only be measured when the sketch controls current: ``foc_current`` (best) or
``dc_current``. In voltage mode there is no current reading and the targets are volts, so the app
won't switch the motor on.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Callable

from ..backends.base import BackendError
from ..control import ControlMode, ErrorFlag, MotorState, decode_flags
from ..device import (
    ArmError,
    overspeed_trip,
    Capabilities,
    DeviceInfo,
    FastReading,
    MotorController,
    MotorParameters,
    Readiness,
    SlowReading,
)
from .commander import CommanderError, CommanderLink, format_number, parse_number, parse_numbers

log = logging.getLogger(__name__)

TWO_PI = 2.0 * math.pi
DECIMALS = 5                   # digits after the point in the board's replies

# Motion control types (MotionControlType) and the names Commander prints for them.
MOTION_CODES = {ControlMode.TORQUE: 0, ControlMode.VELOCITY: 1, ControlMode.POSITION: 2}
MOTION_NAMES = {"torque": 0, "vel": 1, "angle": 2, "vel open": 3, "angle open": 4, "angle nocascade": 5,
                "custom": 6}

# Torque control types (TorqueControlType) as Commander prints them.
FOC_CURRENT, DC_CURRENT, VOLTAGE, ESTIMATED_CURRENT = "foc curr", "dc curr", "volt", "est. curr"

# Largest phase voltage (peak) per volt of supply each modulation gives without clipping.
MODULATION_RATIO = {"SinePWM": 0.5, "SVPWM": 1.0 / math.sqrt(3), "Trap 120": 0.5, "Trap 150": 0.5}

# FOCMotorStatus
MOTOR_STATUS = {
    0: "not started",
    1: "starting",
    2: "not calibrated",
    3: "calibrating",
    4: "ready",
    8: "in an error state",
    14: "calibration failed",
    15: "start-up failed",
}
STATUS_READY, STATUS_CALIBRATING = 4, 3
STATUS_FAILED = (8, 14, 15)

# Faults the bench firmware reports (BS, BC).
BENCH_FAULTS = {
    0x1: ("WATCHDOG_EXPIRED", "The board stopped hearing from this app (cable unplugged or app frozen) and "
                              "switched the motor off for safety."),
    0x2: ("OVER_TEMPERATURE", "The driver or motor got too hot. Let it cool down."),
    0x4: ("UNDER_VOLTAGE", "The supply voltage dropped too low. Check the power supply and its current rating."),
    0x8: ("OVER_VOLTAGE", "The supply voltage went too high, usually from energy fed back while braking."),
    0x10: ("OVER_CURRENT", "The measured current went well past the firmware's limit. Check the current sense "
                           "settings (shunt and gain) and the motor wiring."),
    0x20: ("START_UP_FAILED", "The motor's start-up (sensor alignment) failed. Check the sensor and the motor wires, "
                              "free the output, and reset the board or run the motor calibration."),
    0x40: ("OVERSPEED", "The motor turned faster than the speed limit allows (e.g. the arm slipped off its stop, or "
                        "the output was free in torque mode). Lower the torque or raise the speed limit."),
}


def parse_info(text: str) -> dict[str, str]:
    info = {}
    for part in text.split(";"):
        key, _, value = part.partition("=")
        if key.strip():
            info[key.strip()] = value.strip()
    return info


def _float(info: dict[str, str], key: str) -> float | None:
    value = parse_number(info.get(key, ""))
    return value if value is not None and math.isfinite(value) else None


class SimpleFOCMotor(MotorController):
    family = "simplefoc"
    name = "SimpleFOC board"

    def __init__(self, link: CommanderLink, motor_letter: str = "M"):
        super().__init__()
        if len(motor_letter) != 1 or not motor_letter.isalpha():
            raise ValueError("the motor's Commander letter must be one letter")
        self.link = link
        self.m = motor_letter
        self.bench: str | None = None          # the bench command's letter, if the sketch has one
        self.bench_info: dict[str, str] = {}
        self.capabilities = Capabilities()
        self.torque_control = ""
        self.modulation = ""
        self.voltage_limit: float | None = None
        self.max_current: float | None = None  # the bench firmware's hard current limit
        self._motion = 0
        self._enabled = False
        self._fast_cmds = [f"{self.m}MG7"]
        self._original: dict[str, str] = {}
        self._original_decimals: str | None = None
        self._watchdog_on = False
        self._warned_watchdog = False

    # --- connection ------------------------------------------------------------------------
    @property
    def description(self) -> str:
        return self.link.description

    @property
    def backend_kind(self) -> str:
        return self.link.kind

    @property
    def sim(self):
        return self.link.sim

    def close(self) -> None:
        self.link.close()

    @property
    def device_messages(self) -> list[str]:
        return list(self.link.device_messages)

    def _ask(self, command: str, timeout: float | None = None) -> str | None:
        return self.link.ask(command, timeout)

    def _query(self, commands: list[str]) -> list[str | None]:
        return self.link.query(commands)

    def _motor(self, *commands: str) -> list[str | None]:
        return self._query([self.m + c for c in commands])

    # --- reading ---------------------------------------------------------------------------
    def read_info(self) -> DeviceInfo:
        letters = self.link.scan()
        if self.m not in letters:
            found = ", ".join(f"{k} ({v})" if v else k for k, v in letters.items()) or "none"
            raise CommanderError(
                f"the sketch has no Commander command '{self.m}' for the motor (it has: {found}). Set the motor's "
                "letter in Setup, or add command.add('M', doMotor, \"motor\") to the sketch")
        self.bench = next((k for k, v in letters.items() if v.lower().startswith("bench")), None)
        self._original_decimals = self.link.ask("#")
        self.link.ask(f"#{DECIMALS}")
        if self.bench:
            self.bench_info = parse_info(self._ask(self.bench + "I") or "")
            self.max_current = _float(self.bench_info, "imax")
            self._fast_cmds = [self.bench + "F"]
        else:
            probe = self._ask(self._fast_cmds[0])
            if probe is None or parse_numbers(probe) is None or len(parse_numbers(probe)) < 7:
                self._fast_cmds = [f"{self.m}MG3", f"{self.m}MG5", f"{self.m}MG6", f"{self.m}MG0"]
        info = self.bench_info
        self.motor_thermistor_enabled = info.get("mtemp") == "1"
        self.capabilities = Capabilities(
            watchdog=bool(self.bench), motor_calibration=bool(self.bench), clear_errors=bool(self.bench),
            bus_voltage=bool(self.bench), driver_temperature=info.get("temp") == "1")
        self._read_settings()
        if self.bench:
            hardware = info.get("board", "board")
            firmware = f"bench firmware {info.get('fw', '?')}"
        else:
            hardware, firmware = "board", "SimpleFOC sketch (Commander)"
        return DeviceInfo(self.link.description, self.link.kind, hardware, firmware, None, info.get("id") or "unknown",
                          controller="SimpleFOC", family=self.family, capabilities=self.capabilities)

    def _read_settings(self) -> None:
        torque, motion, enabled, vlim, modulation = self._motor("T", "C", "E", "LU", "WT")
        self.torque_control = (torque or "").strip()
        self._motion = MOTION_NAMES.get((motion or "").strip(), self._motion)
        self._enabled = parse_number(enabled or "") == 1
        self.voltage_limit = parse_number(vlim or "")
        self.modulation = (modulation or "").strip()

    def read_fast(self) -> FastReading:
        replies = self._query(self._fast_cmds)
        if self.bench:
            values = parse_numbers(replies[0] or "")
            if values is None or len(values) < 4:
                raise BackendError(f"unexpected fast reading {replies[0]!r}")
            iq, iq_set, vel, angle = values[:4]
        elif len(replies) == 1:
            values = parse_numbers(replies[0] or "")
            if values is None or len(values) < 7:
                raise BackendError(f"unexpected reading {replies[0]!r}")
            target, _vq, _vd, iq, _id, vel, angle = values[:7]
            iq_set = target if self._motion == 0 and self._enabled else 0.0  # unknown in speed/angle mode
        else:
            values = [parse_number(r or "") for r in replies]
            if any(v is None for v in values[:3]):
                raise BackendError(f"unexpected reading {replies!r}")
            iq, vel, angle, target = values
            iq_set = target if self._motion == 0 and self._enabled and target is not None else 0.0
        return FastReading(float(iq), float(iq_set), angle / TWO_PI, vel / TWO_PI)

    def read_slow(self) -> SlowReading:
        if not self.bench:
            (enabled,) = self._motor("E")
            self._enabled = parse_number(enabled or "") == 1
            return SlowReading(state=int(MotorState.CLOSED_LOOP if self._enabled else MotorState.IDLE))
        values = parse_numbers(self._ask(self.bench + "S") or "")
        if values is None or len(values) < 6:
            raise BackendError("unexpected status reading from the bench firmware")
        vbus, driver_temp, motor_temp, status, faults, enabled = values[:6]
        self._enabled = enabled == 1
        state = MotorState.CLOSED_LOOP if self._enabled else MotorState.IDLE
        if int(status) == STATUS_CALIBRATING:
            state = MotorState.CALIBRATING
        errors = decode_flags("board", int(faults), BENCH_FAULTS)
        return SlowReading(vbus=vbus, fet_temp=driver_temp,
                           motor_temp=motor_temp if self.motor_thermistor_enabled else math.nan,
                           state=int(state), errors=errors, raw_errors={"board": int(faults)})

    def _status(self) -> int | None:
        if not self.bench:
            return None
        values = parse_numbers(self._ask(self.bench + "S") or "")
        return int(values[3]) if values and len(values) >= 4 else None

    def readiness(self) -> Readiness:
        self._read_settings()
        slow = self.read_slow()
        velocity_limit, resistance = self._motor("LV", "R")
        checks: list[tuple[str, str]] = []
        blocked = ""
        torque = self.torque_control
        if torque == FOC_CURRENT:
            checks.append(("ok", "Torque control: FOC current. The board measures Iq, so torque is measured."))
        elif torque == DC_CURRENT:
            checks.append(("warn", "Torque control: DC current. The board measures one current magnitude, so torque "
                                   "readings are rougher than with FOC current."))
        else:
            what = {VOLTAGE: "voltage", ESTIMATED_CURRENT: "estimated current"}.get(torque, repr(torque))
            blocked = (f"The sketch controls the motor by {what}, so the board doesn't measure the current: the app "
                       "can't measure torque or limit the current. Use the bench firmware, or set "
                       "motor.torque_controller = TorqueControlType::foc_current with a current sensor linked.")
            checks.append(("bad", blocked))
        if self.bench:
            info = self.bench_info
            extras = ["watchdog"]
            extras.append("supply voltage" + ("" if info.get("vbus") == "measured" else " (as configured, not measured)"))
            if self.capabilities.driver_temperature:
                extras.append("driver temperature")
            if self.motor_thermistor_enabled:
                extras.append("motor temperature")
            checks.append(("ok", f"Bench firmware {info.get('fw', '?')}: {', '.join(extras)}."))
            checks.append(("info", "SimpleFOC doesn't limit the speed in torque mode, so the bench firmware switches "
                                   "the motor off above 1.2 × your speed limit."))
            sense = _float(info, "isense")
            if sense:
                checks.append(("info", f"The board's current sensors read up to ±{sense:.3g} A; the firmware allows "
                                       f"at most {self.max_current or sense:.3g} A."))
        else:
            checks.append(("warn", "Plain SimpleFOC sketch: no watchdog, so if the app or the USB cable stops while the "
                                   "motor is on, the motor keeps its last command. Keep STOP in reach, or flash the bench "
                                   "firmware (firmware/simplefoc_bench). Supply voltage and temperature aren't reported."))
            checks.append(("warn", "SimpleFOC doesn't limit the speed in torque mode. Without the bench firmware only the "
                                   "app watches it, at its sample rate: an unloaded motor can overshoot a lot before it "
                                   "is switched off. Keep the output loaded or blocked in torque mode."))
        status = self._status()
        calibrated = None
        if status is not None:
            calibrated = status == STATUS_READY or status == STATUS_CALIBRATING
            if status != STATUS_READY:
                text = MOTOR_STATUS.get(status, f"status {status}")
                checks.append(("bad", f"Motor start-up: {text}. Free the output and run the motor calibration, or reset "
                                      "the board."))
                if status in STATUS_FAILED:
                    blocked = blocked or f"The motor's start-up {text}."
        if self.voltage_limit:
            limit_text = f"Voltage limit {self.voltage_limit:.3g} V"
            if self.modulation:
                limit_text += f", {self.modulation} modulation"
            supply_share = MODULATION_RATIO.get(self.modulation, 0.5) * slow.vbus
            if math.isfinite(supply_share) and self.voltage_limit < supply_share:
                limit_text += ": the voltage limit, not the supply, sets the top speed"
            checks.append(("info", limit_text + "."))
        vel_limit = parse_number(velocity_limit or "")
        return Readiness(
            state=slow.state,
            motor_calibrated=calibrated,
            encoder_ready=calibrated,
            errors=slow.errors,
            kt_device=None,
            current_lim=self.max_current,
            vel_limit=vel_limit / TWO_PI if vel_limit is not None and math.isfinite(vel_limit) else None,
            vel_gain=None,
            vbus=slow.vbus if math.isfinite(slow.vbus) else None,
            checks=checks,
            blocked=blocked,
        )

    def config_snapshot(self) -> dict[str, object]:
        commands = ["T", "C", "E", "LC", "LV", "LU", "R", "I", "K", "WT", "VP", "VI", "VD", "VF", "AP", "QP", "QI",
                    "SM", "SE"]
        snapshot: dict[str, object] = {}
        for command in commands:
            try:
                (reply,) = self._motor(command)
            except BackendError as exc:
                reply = f"(no reply: {exc})"
            snapshot[f"{self.m}{command}"] = reply
        if self.bench:
            snapshot["bench_info"] = dict(self.bench_info)
        return snapshot

    def motor_parameters(self) -> MotorParameters:
        resistance, inductance = self._motor("R", "I")
        r, inductance = parse_number(resistance or ""), parse_number(inductance or "")
        pole_pairs = _float(self.bench_info, "pp")
        return MotorParameters(r if r and math.isfinite(r) and r > 0 else None,
                               inductance if inductance and math.isfinite(inductance) and inductance > 0 else None,
                               int(pole_pairs) if pole_pairs else None)

    @property
    def modulation_ratio(self) -> float:
        """The modulation's share of the supply: half with sine PWM, 1/√3 with space-vector PWM."""
        return MODULATION_RATIO.get(self.modulation, 0.5)

    @property
    def voltage_cap(self) -> float | None:
        """SimpleFOC never applies more than ``voltage_limit`` (phase peak)."""
        v = self.voltage_limit
        return v if v and math.isfinite(v) and v > 0 else None

    # --- configuration we may change -------------------------------------------------------
    def remember_config(self) -> None:
        motion, current, velocity = self._motor("C", "LC", "LV")
        self._original = {}
        if motion is not None and motion.strip() in MOTION_NAMES:
            self._original["C"] = str(MOTION_NAMES[motion.strip()])
        for key, value in (("LC", current), ("LV", velocity)):
            number = parse_number(value or "")
            if number is not None and math.isfinite(number):
                self._original[key] = format_number(number)

    def restore_config(self) -> None:
        commands = [f"{self.m}{key}{value}" for key, value in self._original.items()]
        if self.bench:
            commands.append(f"{self.bench}O0")
        if self._original_decimals and self._original_decimals.strip().isdigit():
            commands.append(f"#{self._original_decimals.strip()}")
        try:
            if commands:
                self._query(commands)
            self.link.send("@2")  # back to Commander's default, readable output (its reply isn't needed)
        except BackendError as exc:
            log.warning("Could not restore the board's settings: %s", exc)

    # --- control ---------------------------------------------------------------------------
    def _current(self, amps: float) -> float:
        if self.max_current:
            return max(-self.max_current, min(self.max_current, amps))
        return amps

    def _angle_now(self) -> float:
        return self.read_fast().pos * TWO_PI

    def _set_mode_commands(self, mode: ControlMode) -> list[str]:
        if mode not in MOTION_CODES:
            raise ArmError(f"The {self.name} can't run in {mode.name.lower()} mode from this app.")
        code = MOTION_CODES[mode]
        target = self._angle_now() if mode == ControlMode.POSITION else 0.0
        return [f"{self.m}C{code}", f"{self.m}{format_number(target)}"]

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
        self._read_settings()
        if self.torque_control not in (FOC_CURRENT, DC_CURRENT):
            raise ArmError("The sketch doesn't control current, so the app won't switch the motor on: its commands "
                           "would be volts, not amps. Use foc_current (see the Connect checks).")
        errors = self.read_errors()
        if errors:
            raise ArmError("The board reports errors. Clear them first.", errors)
        status = self._status()
        if status is not None and status != STATUS_READY:
            raise ArmError(f"The motor isn't ready: its start-up {MOTOR_STATUS.get(status, f'status {status}')}.")
        commands = [f"{self.m}LC{format_number(self._current(abs(current_lim)))}",
                    f"{self.m}LV{format_number(abs(vel_limit) * TWO_PI)}",
                    *self._overspeed_commands(vel_limit),
                    *self._set_mode_commands(mode)]
        self._query(commands)
        self._motion = MOTION_CODES[mode]
        if watchdog_timeout:
            self.enable_watchdog(watchdog_timeout)
        if should_abort and should_abort():
            self.disarm()
            raise ArmError("Stopped while arming.")
        (enabled,) = self._motor("E1")
        if parse_number(enabled or "") != 1:
            self.disarm()
            raise ArmError("The board didn't switch the motor on.")
        self._enabled = True

    def change_mode(self, mode: ControlMode, *, torque_mode_vel_limit: bool = True) -> None:
        self._query(self._set_mode_commands(mode))
        self._motion = MOTION_CODES[mode]

    def set_limits(self, *, vel_limit: float, current_lim: float) -> None:
        self._query([f"{self.m}LC{format_number(self._current(abs(current_lim)))}",
                     f"{self.m}LV{format_number(abs(vel_limit) * TWO_PI)}", *self._overspeed_commands(vel_limit)])

    def _overspeed_commands(self, vel_limit: float) -> list[str]:
        if not self.bench:
            return []
        return [f"{self.bench}O{format_number(overspeed_trip(vel_limit) * TWO_PI)}"]

    def disarm(self) -> None:
        """Motor off, then torque mode with a zero target: the safest state to leave it in."""
        self._motor("E0", "C0", "0")
        self._enabled = False
        self._motion = 0
        self.disable_watchdog()

    def estop(self) -> None:
        for _ in range(2):
            try:
                self._motor("E0")
                break
            except BackendError as exc:
                log.error("E-stop command failed: %s", exc)
        self._enabled = False
        try:
            self._motor("0")
            self.disable_watchdog()
        except BackendError:
            pass

    def set_current(self, amps: float) -> None:
        self._motor(format_number(self._current(amps)))

    def set_velocity(self, turns_per_s: float) -> None:
        self._motor(format_number(turns_per_s * TWO_PI))

    def set_position(self, turns: float, vel_ff: float | None = None) -> None:
        self._motor(format_number(turns * TWO_PI))

    # --- watchdog --------------------------------------------------------------------------
    def enable_watchdog(self, timeout_s: float) -> None:
        if not self.bench:
            if not self._warned_watchdog:
                log.warning("This SimpleFOC sketch has no watchdog; continuing without it")
                self._warned_watchdog = True
            return
        self._query([f"{self.bench}W{format_number(timeout_s)}", f"{self.bench}K"])
        self._watchdog_on = True

    def feed_watchdog(self) -> None:
        if self._watchdog_on:
            self._ask(self.bench + "K")

    def disable_watchdog(self) -> None:
        if self._watchdog_on:
            self._watchdog_on = False
            self._ask(self.bench + "W0")

    @property
    def watchdog_on(self) -> bool:
        return self._watchdog_on

    # --- maintenance -----------------------------------------------------------------------
    def clear_errors(self) -> list[ErrorFlag]:
        if not self.bench:
            return []
        remaining = parse_number(self._ask(self.bench + "C") or "") or 0
        return decode_flags("board", int(remaining), BENCH_FAULTS)

    def start_calibration(self) -> None:
        """Re-run the sensor alignment. The board is busy (and silent) until it finishes."""
        if not self.bench:
            raise BackendError("plain SimpleFOC sketches align their sensor when the board starts; reset the board "
                               "with the output free to run it again")
        self._watchdog_on = False
        reply = self._ask(self.bench + "A", timeout=20.0)
        if parse_number(reply or "") != 1:
            raise BackendError("the sensor alignment failed; check the sensor and motor wiring and free the output")
