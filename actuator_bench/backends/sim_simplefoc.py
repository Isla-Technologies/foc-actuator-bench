"""A fake serial port that answers like a SimpleFOC board: Commander, plus the bench firmware's
``B`` command unless ``bench=False`` (then it is a plain SimpleFOC sketch).

The motor, gearbox and load are the ones of a SimulatedODrive, driven the way SimpleFOC drives a
motor: no speed limit in torque mode, no overspeed error, the velocity limit only in angle mode,
and at most ``voltage_limit`` (and what the modulation makes of the supply) on the phases.
Parsing and replies follow Arduino-FOC 2.4.0's Commander (src/communication/Commander.cpp).
"""

from __future__ import annotations

import math

from ..odrive_api import AxisState, ControlMode, InputMode
from .simulator import SimulatedODrive

TWO_PI = 2.0 * math.pi
_A = "axis0."
MOTION_NAMES = {0: "torque", 1: "vel", 2: "angle", 3: "vel open", 4: "angle open", 5: "angle nocascade", 6: "custom"}
TORQUE_NAMES = {0: "volt", 1: "dc curr", 2: "foc curr", 3: "est. curr"}
MODULATION_NAMES = {0: "SinePWM", 1: "SVPWM", 2: "Trap 120", 3: "Trap 150"}
MODULATION_RATIO = {0: 0.5, 1: 1.0 / math.sqrt(3), 2: 0.5, 3: 0.5}
_ODRIVE_MODE = {0: ControlMode.TORQUE, 1: ControlMode.VELOCITY, 2: ControlMode.POSITION}


class SimulatedSimpleFOCBoard:
    """Implements the subset of the pyserial ``Serial`` API that CommanderLink uses."""

    def __init__(
        self,
        sim: SimulatedODrive,
        *,
        bench: bool = True,
        motor_letter: str = "M",
        torque_control: int = 2,           # foc_current
        voltage_limit: float = 11.0,       # V: with SVPWM on 24 V, about what an ODrive applies
        modulation: int = 1,               # SVPWM
        max_current: float = 20.0,         # the bench firmware's hard limit [A]
        has_get_all: bool = True,          # MMG7 (older SimpleFOC versions answer err)
        port: str = "SIM-SIMPLEFOC",
    ):
        self.sim = sim
        self.port = port
        self.is_open = True
        self.bench = bench
        self.letter = motor_letter
        self.has_get_all = has_get_all
        self.verbose = 2                   # Commander starts user-friendly
        self.decimals = 3
        self.received_lines: list[str] = []
        self._from_host = bytearray()
        self._to_host = bytearray()

        a = sim.actuator
        self.motion = 0
        self.torque_control = torque_control
        self.modulation = modulation
        self.target = 0.0
        self.current_limit = min(2.0, max_current)  # SimpleFOC's default (DEF_CURRENT_LIM)
        self.velocity_limit = 20.0                   # rad/s (DEF_VEL_LIM)
        self.voltage_limit = voltage_limit
        self.max_current = max_current
        self.phase_resistance = a.phase_resistance
        self.phase_inductance = a.phase_inductance
        self.kv = a.kv
        self.pid = {"V": {"P": 0.5, "I": 10.0, "D": 0.0, "R": 1000.0, "L": 2.0, "F": 0.01},
                    "A": {"P": 20.0, "I": 0.0, "D": 0.0, "R": 0.0, "L": 20.0, "F": 0.0},
                    "Q": {"P": 3.0, "I": 300.0, "D": 0.0, "R": 0.0, "L": 12.0, "F": 0.005},
                    "D": {"P": 3.0, "I": 300.0, "D": 0.0, "R": 0.0, "L": 12.0, "F": 0.005}}
        self.watchdog_s = 0.0
        self.overspeed_rad_s = 0.0

        sim.max_phase_voltage = lambda vbus: min(self.voltage_limit, MODULATION_RATIO[self.modulation] * vbus)
        sim.write(_A + "controller.config.enable_torque_mode_vel_limit", False)
        sim.write(_A + "controller.config.enable_overspeed_error", False)
        sim.write(_A + "controller.config.input_mode", int(InputMode.PASSTHROUGH))
        sim.write(_A + "motor.config.torque_constant", a.kt_nominal)
        sim.write(_A + "motor.config.torque_lim", math.inf)
        self._apply()

    # --- pyserial-like API -----------------------------------------------------------------
    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise OSError("port is closed")
        self._from_host.extend(data)
        while True:
            ends = [i for i in (self._from_host.find(b"\n"), self._from_host.find(b"\r")) if i >= 0]
            if not ends:
                break
            end = min(ends)
            line = bytes(self._from_host[:end]).decode("ascii", errors="replace")
            del self._from_host[:end + 1]
            if line:
                self.received_lines.append(line)
                self._run(line)
        return len(data)

    def readline(self) -> bytes:
        if not self.is_open:
            raise OSError("port is closed")
        idx = self._to_host.find(b"\n")
        if idx < 0:
            data = bytes(self._to_host)
            self._to_host.clear()
            return data
        line = bytes(self._to_host[:idx + 1])
        del self._to_host[:idx + 1]
        return line

    def reset_input_buffer(self) -> None:
        self._to_host.clear()

    def close(self) -> None:
        self.is_open = False

    def inject_line(self, text: str) -> None:
        """Output the sketch prints by itself (boot messages, debug prints)."""
        self._to_host.extend(f"{text}\r\n".encode("ascii"))

    # --- Commander -------------------------------------------------------------------------
    def _print(self, text: str) -> None:
        if self.verbose:
            self._to_host.extend(f"{text}\r\n".encode("ascii"))

    def _reply(self, prefix: str, payload) -> None:
        if isinstance(payload, float):
            payload = self._num(payload)
        self._print((prefix if self.verbose == 3 else "") + str(payload))

    def _num(self, value: float) -> str:
        if math.isnan(value):
            return "nan"
        if math.isinf(value):
            return "inf" if value > 0 else "-inf"
        return f"{value:.{self.decimals}f}"

    def _run(self, line: str) -> None:
        head, rest = line[0], line[1:]
        if head == "@":
            if rest:
                self.verbose = int(_atof(rest))
            self._print({0: "off!", 3: "@machine"}.get(self.verbose, "on!"))
        elif head == "#":
            if rest:
                self.decimals = int(_atof(rest))
            self._reply("#", str(self.decimals))
        elif head == "?":
            for letter, label in self._registered().items():
                self._print(("?" if self.verbose == 3 else "") + f"{letter}:{label}")
        elif head == self.letter:
            self._motor(rest)
        elif head == "B" and self.bench:
            self._bench(rest)
        # Like Commander: a letter nobody registered gets no reply at all.

    def _registered(self) -> dict[str, str]:
        letters = {self.letter: "motor"}
        if self.bench:
            letters["B"] = "bench"
        return letters

    def _motor(self, cmd: str) -> None:
        letter = self.letter
        if not cmd or cmd[0].isdigit() or cmd[0] in "+-":
            if cmd:
                self.target = _atof(cmd.split(" ")[0])
                self._apply()
                self._reply(letter, float(self.target))
            elif self.verbose == 3:
                self._reply(letter, float(self.target))
            return
        c, sub = cmd[0], cmd[1:2]
        upper_sub = bool(sub) and "A" <= sub <= "Z"
        value_text = cmd[2:] if upper_sub or sub == "#" else cmd[1:]
        get = value_text == ""
        value = _atof(value_text)
        prefix = letter + c + (sub if upper_sub else "")
        if c == "L":
            attr = {"C": "current_limit", "V": "velocity_limit", "U": "voltage_limit"}.get(sub)
            if attr is None:
                self._reply(prefix, "err")
                return
            if not get:
                setattr(self, attr, value)
                self._apply()
            self._reply(prefix, float(getattr(self, attr)))
        elif c == "C":
            value = _atof(cmd[2:] if upper_sub else cmd[1:])
            if not (cmd[1:] == "" or upper_sub) and 0 <= value < 7:
                self._set_motion(int(value))
            self._reply(prefix, MOTION_NAMES[self.motion])
        elif c == "T":
            if not get and 0 <= value < 4:
                self.torque_control = int(value)
            self._reply(prefix, TORQUE_NAMES[self.torque_control])
        elif c == "E":
            if not get:
                self._enable(bool(value))
            self._reply(prefix, str(int(self._enabled())))
        elif c in "RIK":
            attr = {"R": "phase_resistance", "I": "phase_inductance", "K": "kv"}[c]
            if not get:
                setattr(self, attr, value)
            self._reply(prefix, float(getattr(self, attr) or 0.0))
        elif c == "W" and sub == "T":
            if not get:
                self.modulation = int(value)
            self._reply(prefix, MODULATION_NAMES[self.modulation])
        elif c == "M" and sub == "G":
            self._monitor(prefix, int(value))
        elif c in self.pid and sub in self.pid[c]:
            if not get:
                self.pid[c][sub] = value
            self._reply(prefix, float(self.pid[c][sub]))
        elif c == "S" and sub in ("M", "E"):
            self._reply(prefix, 0.0)
        else:
            self._reply(prefix, "err")

    def _monitor(self, prefix: str, which: int) -> None:
        iq = self._read("motor.current_control.Iq_measured")
        vel = self._read("encoder.vel_estimate") * TWO_PI
        angle = self._read("encoder.pos_estimate") * TWO_PI
        vq = self.phase_resistance * iq + self.sim.actuator.flux * vel
        values = [self.target, vq, 0.0, iq, self._read("motor.current_control.Id_measured"), vel, angle]
        if which == 7 and self.has_get_all:
            self._reply(prefix, ";".join(self._num(v) for v in values))
        elif 0 <= which < 7:
            self._reply(prefix, float(values[which]))
        else:
            self._reply(prefix, "err")

    # --- the bench firmware's command ------------------------------------------------------
    def _bench(self, cmd: str) -> None:
        sub, value_text = cmd[:1], cmd[1:]
        prefix = "B" + sub
        if sub == "I":
            info = {"fw": "1.0", "board": "simulated ESP32 FOC", "pp": str(self.sim.actuator.pole_pairs),
                    "imax": self._num(self.max_current), "isense": self._num(1.5 * self.max_current),
                    "vbus": "measured", "temp": "1", "mtemp": "1" if self.sim.actuator.motor_thermistor else "0",
                    "id": "SIM-0001"}
            self._reply(prefix, ";".join(f"{k}={v}" for k, v in info.items()))
        elif sub == "F":
            values = [self._read("motor.current_control.Iq_measured"), self._read("motor.current_control.Iq_setpoint"),
                      self._read("encoder.vel_estimate") * TWO_PI, self._read("encoder.pos_estimate") * TWO_PI]
            self._reply(prefix, ";".join(self._num(v) for v in values))
        elif sub == "S":
            ax = self.sim.axes[0]
            values = [self.sim.read("vbus_voltage"), self._read("motor.fet_thermistor.temperature"),
                      ax.motor_temp if self.sim.actuator.motor_thermistor else math.nan,
                      4 if ax.motor_calibrated and not ax.encoder_error else 15, self._faults(), int(self._enabled())]
            self._reply(prefix, ";".join(self._num(v) if isinstance(v, float) else str(v) for v in values))
        elif sub == "W":
            if value_text:
                self.watchdog_s = max(0.0, _atof(value_text))
                if self.watchdog_s > 0:
                    self.sim.write(_A + "config.watchdog_timeout", self.watchdog_s)
                    self.sim.call(_A + "watchdog_feed")
                self.sim.write(_A + "config.enable_watchdog", self.watchdog_s > 0)
            self._reply(prefix, float(self.watchdog_s))
        elif sub == "O":
            if value_text:
                self.overspeed_rad_s = max(0.0, _atof(value_text))
                self._apply()
            self._reply(prefix, float(self.overspeed_rad_s))
        elif sub == "K":
            self.sim.call(_A + "watchdog_feed")
            self._reply(prefix, "1")
        elif sub == "C":
            self.sim.call("clear_errors")
            self._reply(prefix, str(self._faults()))
        elif sub == "A":
            self._enable(False)
            ok = self.sim.actuator.scenario != "locked"
            if not ok:
                self.sim.axes[0].encoder_error |= 0x4
            self._reply(prefix, "1" if ok else "0")
        else:
            self._reply(prefix, "err")

    def _faults(self) -> int:
        ax = self.sim.axes[0]
        faults = 0
        if ax.error & 0x800:
            faults |= 0x1   # watchdog
        if ax.motor_error & 0x40000:
            faults |= 0x2   # driver over temperature
        if self.sim.system_error & 0x2:
            faults |= 0x4   # under voltage
        if ax.encoder_error or ax.error & 0x1:
            faults |= 0x20  # start-up (alignment) failed
        if ax.controller_error & 0x1:
            faults |= 0x40  # overspeed
        return faults

    # --- driving the simulated motor -------------------------------------------------------
    def _read(self, path: str) -> float:
        return float(self.sim.read(_A + path))

    def _enabled(self) -> bool:
        return int(self.sim.read(_A + "current_state")) == AxisState.CLOSED_LOOP_CONTROL

    def _enable(self, on: bool) -> None:
        self.sim.write(_A + "requested_state", int(AxisState.CLOSED_LOOP_CONTROL if on else AxisState.IDLE))
        if on:
            self._apply()  # SimpleFOC goes straight for its target (the simulated ODrive starts from where it is)

    def _set_motion(self, motion: int) -> None:
        """updateMotionControlType(): a new mode starts from a zero target, angle from where it is."""
        if motion == self.motion:
            return
        if motion == 2:
            self.target = self._read("encoder.pos_estimate") * TWO_PI
        else:
            self.target = 0.0
        self.motion = motion
        self._apply()

    def _apply(self) -> None:
        sim = self.sim
        mode = _ODRIVE_MODE.get(self.motion, ControlMode.TORQUE)
        sim.write(_A + "controller.config.control_mode", int(mode))
        sim.write(_A + "controller.config.enable_vel_limit", self.motion == 2)
        sim.write(_A + "controller.config.vel_limit", max(self.velocity_limit, 1e-3) / TWO_PI)
        # The firmware's overspeed trip, checked in the control loop: the simulated ODrive's own check,
        # with its tolerance set so it trips at the same speed.
        trip = self.overspeed_rad_s
        sim.write(_A + "controller.config.enable_overspeed_error", trip > 0)
        if trip > 0:
            sim.write(_A + "controller.config.vel_limit_tolerance", trip / max(self.velocity_limit, 1e-3))
        sim.write(_A + "motor.config.current_lim", min(self.current_limit, self.max_current))
        kt = float(sim.read(_A + "motor.config.torque_constant"))
        if self.motion == 0:
            sim.write(_A + "controller.input_torque", self.target * kt)
        elif self.motion == 1:
            sim.write(_A + "controller.input_torque", 0.0)
            sim.write(_A + "controller.input_vel", self.target / TWO_PI)
        elif self.motion == 2:
            sim.write(_A + "controller.input_torque", 0.0)
            sim.write(_A + "controller.input_vel", 0.0)
            sim.write(_A + "controller.input_pos", self.target / TWO_PI)


def _atof(text: str) -> float:
    """C's atof: the longest number at the start, 0 if there is none."""
    text = text.strip()
    best = 0.0
    for end in range(len(text), 0, -1):
        try:
            return float(text[:end])
        except ValueError:
            continue
    return best
