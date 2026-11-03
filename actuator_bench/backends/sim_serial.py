"""A fake serial port that answers like an ODrive v3.6 (firmware 0.5.x) ASCII protocol.

It wraps a SimulatedODrive so the real SerialBackend code can be tested end-to-end without
hardware. Parsing and replies follow the firmware source (communication/ascii_protocol.cpp):
replies end with CRLF, carry ``*<checksum>`` only if the command had one, ``w`` and the
``s?``/``u`` commands reply nothing on success, and ``r`` values are cut to 9 characters.
"""

from __future__ import annotations

import math
import re

from ..odrive_api import ControlMode, InputMode
from .base import PropertyUnavailable
from .simulator import SimulatedODrive

_TERMINATORS = re.compile(rb"[\r\n!]")


def xor_checksum(text: str) -> int:
    value = 0
    for byte in text.encode("ascii", errors="replace"):
        value ^= byte
    return value


class SimulatedSerialPort:
    """Implements the subset of the pyserial ``Serial`` API used by SerialBackend."""

    def __init__(self, sim: SimulatedODrive, port: str = "SIM-SERIAL", timeout: float = 0.05):
        self.sim = sim
        self.port = port
        self.timeout = timeout
        self.is_open = True
        self._from_host = bytearray()
        self._to_host = bytearray()
        self.received_lines: list[str] = []

    # --- pyserial-like API -----------------------------------------------------------------
    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise OSError("port is closed")
        self._from_host.extend(data)
        while True:
            match = _TERMINATORS.search(self._from_host)
            if not match:
                break
            line = bytes(self._from_host[: match.start()]).decode("ascii", errors="replace")
            del self._from_host[: match.end()]
            if line.strip():
                self.received_lines.append(line)
                self._handle_line(line)
        return len(data)

    def readline(self) -> bytes:
        if not self.is_open:
            raise OSError("port is closed")
        idx = self._to_host.find(b"\n")
        if idx < 0:  # behaves like a read timeout
            data = bytes(self._to_host)
            self._to_host.clear()
            return data
        line = bytes(self._to_host[: idx + 1])
        del self._to_host[: idx + 1]
        return line

    def read(self, size: int = 1) -> bytes:
        data = bytes(self._to_host[:size])
        del self._to_host[:size]
        return data

    @property
    def in_waiting(self) -> int:
        return len(self._to_host)

    def reset_input_buffer(self) -> None:
        self._to_host.clear()

    def reset_output_buffer(self) -> None:
        self._from_host.clear()

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False

    # --- test helpers ----------------------------------------------------------------------
    def inject_line(self, text: str) -> None:
        """Queue an unsolicited line (like firmware debug output)."""
        self._to_host.extend(text.encode("ascii") + b"\r\n")

    # --- protocol --------------------------------------------------------------------------
    def _respond(self, use_checksum: bool, text: str) -> None:
        if use_checksum:
            text = f"{text}*{xor_checksum(text)}"
        self._to_host.extend(text.encode("ascii") + b"\r\n")

    def _handle_line(self, line: str) -> None:
        line = line.split(";", 1)[0]
        use_checksum = False
        if "*" in line:
            body, _, received = line.partition("*")
            try:
                received_value = int(received.strip())
            except ValueError:
                return
            if xor_checksum(body) != received_value:
                return  # the firmware silently drops lines with a bad checksum
            line, use_checksum = body, True
        line = line.strip()
        if not line:
            return
        handler = {
            "p": self._cmd_position, "q": self._cmd_position_limits, "v": self._cmd_velocity,
            "c": self._cmd_torque, "t": self._cmd_trajectory, "f": self._cmd_feedback,
            "u": self._cmd_watchdog, "r": self._cmd_read, "w": self._cmd_write,
            "s": self._cmd_system, "i": self._cmd_info, "h": self._cmd_help,
        }.get(line[0])
        if handler is None:
            self._respond(use_checksum, "unknown command")
        else:
            handler(line, use_checksum)

    def _motor_args(self, line: str, use_checksum: bool, minimum: int) -> tuple[int, list[float]] | None:
        parts = line.split()[1:]
        try:
            motor = int(parts[0])
            values = [float(x) for x in parts[1:]]
        except (IndexError, ValueError):
            self._respond(use_checksum, "invalid command format")
            return None
        if len(values) + 1 < minimum:
            self._respond(use_checksum, "invalid command format")
            return None
        if motor not in (0, 1):
            self._respond(use_checksum, f"invalid motor {motor}")
            return None
        return motor, values

    def _axis(self, motor: int) -> str:
        return f"axis{motor}."

    def _cmd_position(self, line: str, use_checksum: bool) -> None:
        args = self._motor_args(line, use_checksum, 2)
        if args:
            motor, values = args
            a = self._axis(motor)
            self.sim.write(a + "controller.config.control_mode", int(ControlMode.POSITION))
            self.sim.write(a + "controller.input_pos", values[0])
            if len(values) > 1:
                self.sim.write(a + "controller.input_vel", values[1])
            if len(values) > 2:
                self.sim.write(a + "controller.input_torque", values[2])
            self.sim.call(a + "watchdog_feed")

    def _cmd_position_limits(self, line: str, use_checksum: bool) -> None:
        args = self._motor_args(line, use_checksum, 2)
        if args:
            motor, values = args
            a = self._axis(motor)
            self.sim.write(a + "controller.config.control_mode", int(ControlMode.POSITION))
            self.sim.write(a + "controller.input_pos", values[0])
            if len(values) > 1:
                self.sim.write(a + "controller.config.vel_limit", values[1])
            if len(values) > 2:
                self.sim.write(a + "motor.config.torque_lim", values[2])
            self.sim.call(a + "watchdog_feed")

    def _cmd_velocity(self, line: str, use_checksum: bool) -> None:
        args = self._motor_args(line, use_checksum, 2)
        if args:
            motor, values = args
            a = self._axis(motor)
            self.sim.write(a + "controller.config.control_mode", int(ControlMode.VELOCITY))
            self.sim.write(a + "controller.input_vel", values[0])
            if len(values) > 1:
                self.sim.write(a + "controller.input_torque", values[1])
            self.sim.call(a + "watchdog_feed")

    def _cmd_torque(self, line: str, use_checksum: bool) -> None:
        args = self._motor_args(line, use_checksum, 2)
        if args:
            motor, values = args
            a = self._axis(motor)
            self.sim.write(a + "controller.config.control_mode", int(ControlMode.TORQUE))
            self.sim.write(a + "controller.input_torque", values[0])
            self.sim.call(a + "watchdog_feed")

    def _cmd_trajectory(self, line: str, use_checksum: bool) -> None:
        args = self._motor_args(line, use_checksum, 2)
        if args:
            motor, values = args
            a = self._axis(motor)
            self.sim.write(a + "controller.config.control_mode", int(ControlMode.POSITION))
            self.sim.write(a + "controller.config.input_mode", int(InputMode.TRAP_TRAJ))
            self.sim.write(a + "controller.input_pos", values[0])
            self.sim.call(a + "watchdog_feed")

    def _cmd_feedback(self, line: str, use_checksum: bool) -> None:
        args = self._motor_args(line, use_checksum, 1)
        if args:
            motor, _ = args
            a = self._axis(motor)
            pos = self.sim.read(a + "encoder.pos_estimate")
            vel = self.sim.read(a + "encoder.vel_estimate")
            self._respond(use_checksum, f"{pos:f} {vel:f}")

    def _cmd_watchdog(self, line: str, use_checksum: bool) -> None:
        args = self._motor_args(line, use_checksum, 1)
        if args:
            self.sim.call(self._axis(args[0]) + "watchdog_feed")

    def _cmd_read(self, line: str, use_checksum: bool) -> None:
        parts = line.split()
        if len(parts) < 2:
            self._respond(use_checksum, "invalid command format")
            return
        path = parts[1]
        kind = self.sim.property_kind(path)
        if kind is None:
            self._respond(use_checksum, "invalid property")
            return
        try:
            value = self.sim.read(path)
        except PropertyUnavailable:
            self._respond(use_checksum, "invalid property")
            return
        self._respond(use_checksum, _format_like_firmware(value, kind))

    def _cmd_write(self, line: str, use_checksum: bool) -> None:
        parts = line.split()
        if len(parts) < 2:
            self._respond(use_checksum, "invalid command format")
            return
        path = parts[1]
        if not self.sim.is_writable(path):
            self._respond(use_checksum, "invalid property")
            return
        if len(parts) < 3:
            self._respond(use_checksum, "not implemented")
            return
        kind = self.sim.property_kind(path)
        try:
            value = _parse_like_firmware(parts[2], kind)
        except ValueError:
            self._respond(use_checksum, "not implemented")
            return
        self.sim.write(path, value)

    def _cmd_system(self, line: str, use_checksum: bool) -> None:
        sub = line[1:2]
        if sub == "s":
            self.sim.call("save_configuration")
        elif sub == "e":
            self.sim.call("erase_configuration")
        elif sub == "r":
            self.sim.call("reboot")
        elif sub == "c":
            self.sim.call("clear_errors")

    def _cmd_info(self, line: str, use_checksum: bool) -> None:
        hw, fw = SimulatedODrive.HW, SimulatedODrive.FW
        self._respond(use_checksum, f"Hardware version: {hw[0]}.{hw[1]}-{hw[2]}V")
        self._respond(use_checksum, f"Firmware version: {fw[0]}.{fw[1]}.{fw[2]}")
        self._respond(use_checksum, f"Serial number: {SimulatedODrive.SERIAL:012X}")

    def _cmd_help(self, line: str, use_checksum: bool) -> None:
        for text in ("Please see documentation for more details", "", "Available commands syntax reference:",
                     "Position: q axis pos vel-lim I-lim", "Save config: ss", "Erase config: se", "Reboot: sr"):
            self._respond(use_checksum, text)


def _format_like_firmware(value, kind: type) -> str:
    if kind is bool:
        text = "1" if value else "0"
    elif kind is int:
        text = str(int(value))
    elif math.isinf(value):
        text = "inf" if value > 0 else "-inf"
    elif math.isnan(value):
        text = "nan"
    else:
        text = f"{value:f}"
    return text[:9]  # the firmware formats into a 10-byte buffer


def _parse_like_firmware(text: str, kind: type):
    if kind is bool:
        lowered = text.lower()
        if lowered in ("1", "true"):
            return True
        if lowered in ("0", "false"):
            return False
        raise ValueError(text)
    if kind is int:
        return int(float(text))
    return float(text)
