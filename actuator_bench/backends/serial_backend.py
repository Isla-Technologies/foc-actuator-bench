"""Talk to the ODrive with its ASCII protocol over the USB virtual serial port.

An ODrive v3.6 shows up as a USB serial port with no driver needed (``/dev/cu.usbmodem…`` on
macOS, ``COMx`` on Windows). Firmware 0.5.x speaks the ASCII protocol on it by default
(``config.usb_cdc_protocol = ASCII_AND_STDOUT``).

Reads are pipelined and sent with checksums. The firmware only adds a checksum to replies of
commands that had one, so real answers can be told apart from debug output and from error
replies to writes (writes are sent without a checksum and reply nothing on success).
"""

from __future__ import annotations

import logging
import math
import re
import sys
import time
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from ..odrive_api import ODRIVE_USB_PIDS, ODRIVE_USB_VID
from .base import Backend, BackendError, ConnectionLost, PropertyUnavailable

log = logging.getLogger(__name__)

ERROR_REPLIES = ("invalid property", "not implemented", "invalid command format",
                 "unknown command", "invalid motor")
_REPLY_WITH_CHECKSUM = re.compile(r"^(.*)\*(\d+)$")
_LEADING_NUMBER = re.compile(r"^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?")
_FUNCTION_COMMANDS = {
    "clear_errors": "sc",
    "save_configuration": "ss",
    "erase_configuration": "se",
    "reboot": "sr",
}
_WATCHDOG_FEED = re.compile(r"axis(\d)\.watchdog_feed")


@dataclass
class PortInfo:
    device: str
    description: str
    is_odrive: bool
    serial_number: str = ""

    @property
    def label(self) -> str:
        text = f"{self.device}  ({self.description})" if self.description else self.device
        return f"{text}  [ODrive]" if self.is_odrive else text


def list_serial_ports() -> list[PortInfo]:
    """All serial ports, ODrives first."""
    from serial.tools import list_ports

    ports = []
    for p in list_ports.comports():
        text = " ".join(filter(None, [p.description, p.manufacturer, p.product])).lower()
        is_odrive = (p.vid == ODRIVE_USB_VID and p.pid in ODRIVE_USB_PIDS) or "odrive" in text
        ports.append(PortInfo(p.device, p.description or "", is_odrive, p.serial_number or ""))
    ports.sort(key=lambda port: (not port.is_odrive, port.device))
    return ports


def xor_checksum(text: str) -> int:
    value = 0
    for byte in text.encode("ascii", errors="replace"):
        value ^= byte
    return value


def frame_command(command: str, with_checksum: bool) -> str:
    """One command line. With a checksum: ``r vbus_voltage *93`` (XOR of everything before '*')."""
    if not with_checksum:
        return f"{command}\n"
    body = f"{command} "
    return f"{body}*{xor_checksum(body)}\n"


def parse_number(text: str) -> float | int | None:
    """Parse a value printed by the firmware ('24.087744', '8', '1', 'inf', ...)."""
    t = text.strip().lower()
    if not t:
        return None
    if t in ("inf", "+inf", "infinity"):
        return math.inf
    if t in ("-inf", "-infinity"):
        return -math.inf
    if t == "nan":
        return math.nan
    if t == "true":
        return 1
    if t == "false":
        return 0
    try:
        return int(t)
    except ValueError:
        pass
    try:
        return float(t)
    except ValueError:
        pass
    match = _LEADING_NUMBER.match(t)  # e.g. "0d" printed by buggy 0.5.2/0.5.3 builds
    if match:
        number = match.group(0)
        return float(number) if any(c in number for c in ".eE") else int(number)
    return None


def format_value(value) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    value = float(value)
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return f"{value:.9g}"


class SerialBackend(Backend):
    kind = "serial"

    def __init__(
        self,
        port: str | None = None,
        *,
        timeout: float = 0.3,
        opener: Callable[[str], object] | None = None,
    ):
        self.port = port
        self.timeout = timeout
        self._opener = opener
        self._ser = None
        self.use_checksums = True
        self._identity: dict[str, str] = {}
        self.device_messages: deque[str] = deque(maxlen=50)

    @property
    def description(self) -> str:
        return f"USB serial ({self.port})" if self.port else "USB serial"

    # --- connection ------------------------------------------------------------------------
    def connect(self) -> None:
        if self.port:
            candidates = [self.port]
        else:
            candidates = [p.device for p in list_serial_ports() if p.is_odrive]
            if not candidates:
                raise BackendError(
                    "No ODrive serial port found. Check that the ODrive's DC power is on "
                    "(USB alone does not power it) and that the USB cable carries data."
                )
        problems = []
        for device in candidates:
            try:
                self._open(device)
                self._probe()
                self.port = device
                log.info("Connected to ODrive on %s (checksums %s)", device,
                         "on" if self.use_checksums else "off")
                return
            except BackendError as exc:
                problems.append(f"{device}: {exc}")
                self._close_port()
        raise BackendError("; ".join(problems))

    def close(self) -> None:
        self._close_port()

    def _open(self, device: str) -> None:
        if self._opener is not None:
            self._ser = self._opener(device)
        else:
            import serial

            # On macOS/Linux ask for exclusive access so no other program interleaves commands
            # (Windows COM ports are always exclusive).
            extra = {} if sys.platform == "win32" else {"exclusive": True}
            try:
                self._ser = serial.Serial(device, 115200, timeout=min(self.timeout, 0.1), write_timeout=1.0, **extra)
            except (serial.SerialException, OSError, ValueError) as exc:
                raise BackendError(f"can't open the port ({exc})") from exc
        self._ser.reset_input_buffer()

    def _close_port(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None

    def _probe(self) -> None:
        paths = ["fw_version_major", "fw_version_minor", "fw_version_revision"]
        for use_checksums in (True, False):
            self.use_checksums = use_checksums
            try:
                values = self._read_batch(paths)
            except BackendError:
                continue
            if all(v is not None for v in values):
                self._identity = self._read_identity()
                return
        raise BackendError("it didn't answer like an ODrive (firmware 0.5.x with the ASCII protocol on USB)")

    def _read_identity(self) -> dict[str, str]:
        self._send(["i"], self.use_checksums)
        deadline = time.monotonic() + self.timeout
        identity: dict[str, str] = {}
        keys = {"hardware version": "hardware", "firmware version": "firmware", "serial number": "serial"}
        while len(identity) < 3:
            reply = self._next_reply(deadline)
            if reply is None or reply.lower().startswith(ERROR_REPLIES):
                break
            name, _, value = reply.partition(":")
            key = keys.get(name.strip().lower())
            if key:
                identity[key] = value.strip()
        return identity

    def identity(self) -> dict[str, str]:
        return dict(self._identity)

    # --- property access -------------------------------------------------------------------
    def read(self, path: str):
        value = self._read_batch([path])[0]
        if value is None:
            raise PropertyUnavailable(path, "not readable over the serial connection")
        return value

    def read_many(self, paths: Sequence[str]):
        return self._read_batch(list(paths))

    def write(self, path: str, value) -> None:
        self._send([f"w {path} {format_value(value)}"], with_checksum=False)
        if not self.use_checksums:
            self._drain_unsolicited()

    def call(self, path: str) -> None:
        command = _FUNCTION_COMMANDS.get(path)
        if command is None:
            match = _WATCHDOG_FEED.fullmatch(path)
            if not match:
                raise PropertyUnavailable(path, "function not available over the serial connection")
            command = f"u {match.group(1)}"
        self._send([command], with_checksum=False)
        if not self.use_checksums:
            self._drain_unsolicited()

    # --- low level -------------------------------------------------------------------------
    def _read_batch(self, paths: list[str]) -> list[float | int | None]:
        self._send([f"r {p}" for p in paths], self.use_checksums)
        deadline = time.monotonic() + self.timeout + 0.005 * len(paths)
        values: list[float | int | None] = []
        for path in paths:
            reply = self._next_reply(deadline)
            if reply is None:
                self._resync()
                raise BackendError(f"timed out waiting for {path}")
            values.append(self._parse_reply(path, reply))
        return values

    def _parse_reply(self, path: str, reply: str):
        if reply.lower().startswith(ERROR_REPLIES):
            log.debug("ODrive replied %r to read of %s", reply, path)
            return None
        value = parse_number(reply)
        if value is None:
            log.warning("Unexpected reply %r to read of %s", reply, path)
        return value

    def _send(self, commands: list[str], with_checksum: bool) -> None:
        if self._ser is None:
            raise ConnectionLost("serial port is not open")
        payload = "".join(frame_command(c, with_checksum) for c in commands)
        try:
            self._ser.write(payload.encode("ascii"))
        except (OSError, ValueError) as exc:
            raise ConnectionLost(f"serial write failed ({exc})") from exc

    def _read_line(self, deadline: float) -> str | None:
        if self._ser is None:
            raise ConnectionLost("serial port is not open")
        while time.monotonic() < deadline:
            try:
                raw = self._ser.readline()
            except (OSError, ValueError) as exc:
                raise ConnectionLost(f"serial read failed ({exc})") from exc
            if raw:
                return raw.decode("ascii", errors="replace").strip()
        return None

    def _next_reply(self, deadline: float) -> str | None:
        """Next reply to one of our commands; anything else is recorded and skipped."""
        while True:
            line = self._read_line(deadline)
            if line is None:
                return None
            if not line:
                continue
            if not self.use_checksums:
                return line
            match = _REPLY_WITH_CHECKSUM.match(line)
            if match and xor_checksum(match.group(1)) == int(match.group(2)):
                return match.group(1).strip()
            self._note_device_message(line)

    def _note_device_message(self, line: str) -> None:
        self.device_messages.append(line)
        if line.lower().startswith(ERROR_REPLIES):
            log.warning("ODrive rejected a command: %s", line)
        else:
            log.debug("ODrive message: %s", line)

    def _drain_unsolicited(self) -> None:
        """Without checksums, wait briefly and discard any error reply to a write."""
        time.sleep(0.003)
        try:
            waiting = self._ser.in_waiting
            if waiting:
                for line in self._ser.read(waiting).decode("ascii", errors="replace").splitlines():
                    if line.strip():
                        self._note_device_message(line.strip())
        except (OSError, ValueError) as exc:
            raise ConnectionLost(f"serial read failed ({exc})") from exc

    def _resync(self) -> None:
        try:
            time.sleep(0.02)
            self._ser.reset_input_buffer()
        except (OSError, ValueError, AttributeError):
            pass
