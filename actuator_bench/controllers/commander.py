"""SimpleFOC's Commander: a line-based text protocol on a serial port.

Every SimpleFOC sketch that registers its motor with Commander (``command.add('M', doMotor,
"motor")``) answers it, whatever the board: ESP32 FOC drivers, the SimpleFOC Shield, B-G431B-ESC1,
the MKS dual boards. The board usually reaches the computer through a USB-serial adapter (CP2102,
CH340, FTDI) or its own USB port.

The app switches Commander to machine-readable output (``@3``). Then every reply starts with the
command it answers: ``MMG3`` → ``MMG0.1234``, ``MLC5`` → ``MLC5.0000``, ``M1.5`` → ``M1.5000``,
an unknown sub-command → ``MZerr``. Replies come back in order, one line per command, so commands
are sent in batches and each reply is matched to its command by that prefix; anything else (boot
messages such as ``MOT: Init``, the sketch's own prints) is kept for the debug report and skipped.
A command letter the sketch didn't register gets no reply at all, so the app only uses letters
that ``?`` (the command list) shows.

Facts from the Arduino-FOC 2.4.0 source (src/communication/Commander.cpp, commands.h).
"""

from __future__ import annotations

import logging
import math
import sys
import time
from collections import deque
from collections.abc import Callable, Sequence

from ..backends.base import BackendError, ConnectionLost

log = logging.getLogger(__name__)

MAX_COMMAND_LENGTH = 20        # Commander's input buffer, end-of-line included
BAUD_RATES = (115200, 230400, 460800, 921600, 250000, 500000, 1000000)
DEFAULT_BAUD = 115200
BOOT_WAIT_S = 6.0              # opening the port may restart the board; it aligns its sensor at boot

# USB vendor IDs of the USB-serial chips SimpleFOC boards use (or that sit on an adapter cable).
USB_SERIAL_VENDORS = {
    0x10C4: "CP210x",
    0x1A86: "CH340",
    0x0403: "FTDI",
    0x303A: "ESP32 USB",
    0x0483: "STM32 / ST-Link",
    0x2341: "Arduino",
    0x2E8A: "RP2040",
    0x067B: "PL2303",
}


class CommanderError(BackendError):
    """The board didn't answer like a SimpleFOC sketch with Commander."""


def reply_prefix(command: str) -> str:
    """What a machine-readable reply to ``command`` starts with.

    Commander prints the command letter, then (for anything but a target value) the next
    character, then the one after it if it is an upper-case sub-command letter.
    """
    head = command[:1]
    if head in ("@", "#", "?", ""):
        return head
    rest = command[1:]
    if not rest or rest[0].isdigit() or rest[0] in "+-":
        return head  # a target value: ``M1.5`` → ``M1.5000``
    prefix = head + rest[0]
    if len(rest) > 1 and "A" <= rest[1] <= "Z":
        prefix += rest[1]
    return prefix


def format_number(value: float) -> str:
    """A number Commander's atof reads, short enough for its 20-character buffer."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"can't send {value} to the board")
    if abs(value) < 1e-7:
        return "0"
    text = f"{value:.7g}"
    if "e" in text or len(text) > 12:
        text = f"{value:.5e}"
    return text


def parse_number(text: str) -> float | None:
    """A value as Arduino's Serial.print writes it ('0.1234', '-3', 'nan', 'inf', 'ovf')."""
    t = text.strip().lower()
    if t in ("nan", "ovf"):
        return math.nan
    if t in ("inf", "+inf"):
        return math.inf
    if t == "-inf":
        return -math.inf
    try:
        return float(t)
    except ValueError:
        return None


def parse_numbers(text: str, separator: str = ";") -> list[float] | None:
    values = [parse_number(part) for part in text.split(separator)]
    return None if any(v is None for v in values) else values


def is_error(payload: str | None) -> bool:
    return payload is None or payload.strip().lower() == "err"


class CommanderLink:
    """A serial connection to a board running a SimpleFOC sketch with Commander."""

    kind = "commander"

    def __init__(
        self,
        port: str | None = None,
        baud: int = DEFAULT_BAUD,
        *,
        timeout: float = 0.3,
        boot_wait_s: float = BOOT_WAIT_S,
        opener: Callable[[str, int], object] | None = None,
        kind: str | None = None,
    ):
        self.port = port
        self.baud = int(baud)
        self.timeout = timeout
        self.boot_wait_s = boot_wait_s
        self._opener = opener
        self._ser = None
        self._buffer = bytearray()
        if kind:
            self.kind = kind
        self.device_messages: deque[str] = deque(maxlen=50)

    @property
    def description(self) -> str:
        where = self.port or "auto"
        return "Simulator (no hardware)" if self.kind == "simulator" else f"USB serial ({where}, {self.baud} baud)"

    @property
    def sim(self):
        """The simulated ODrive behind a simulated board (None for real hardware)."""
        return getattr(self._ser, "sim", None)

    # --- connection ------------------------------------------------------------------------
    def connect(self) -> None:
        if self.port:
            candidates = [self.port]
        else:
            from ..backends.serial_backend import list_serial_ports

            candidates = [p.device for p in list_serial_ports() if p.adapter and not p.is_odrive]
            if not candidates:
                raise CommanderError(
                    "No USB-serial adapter found. Plug in the board's USB cable (or the USB-serial adapter wired "
                    "to its TX, RX and GND) and choose its port.")
        problems = []
        for device in candidates:
            try:
                self._open(device)
                self._handshake()
                self.port = device
                log.info("SimpleFOC board answers on %s at %d baud", device, self.baud)
                return
            except BackendError as exc:
                problems.append(f"{device}: {exc}")
                self.close()
        raise CommanderError("; ".join(problems))

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None

    def _open(self, device: str) -> None:
        if self._opener is not None:
            self._ser = self._opener(device, self.baud)
        else:
            import serial

            ser = serial.Serial()
            ser.port = device
            ser.baudrate = self.baud
            ser.timeout = min(self.timeout, 0.05)
            ser.write_timeout = 1.0
            # Keep DTR and RTS released: on boards with the ESP32 auto-reset circuit, asserting them
            # restarts the board (or holds it in its bootloader).
            ser.dtr = False
            ser.rts = False
            if sys.platform != "win32":
                ser.exclusive = True
            try:
                ser.open()
            except (serial.SerialException, OSError, ValueError) as exc:
                raise CommanderError(f"can't open the port ({exc})") from exc
            self._ser = ser
        self._buffer.clear()
        self._ser.reset_input_buffer()

    def _handshake(self) -> None:
        """Switch to machine-readable replies. A board that restarted when the port opened only
        answers once its sketch has started, after the sensor alignment."""
        deadline = time.monotonic() + self.boot_wait_s
        while True:
            self._write("\n@3\n")
            line_deadline = min(deadline, time.monotonic() + 0.5)
            while True:
                line = self._read_line(line_deadline)
                if line is None:
                    break
                if line.startswith("@") and "machine" in line:
                    self._resync(0.05)
                    return
                if line:
                    self._note(line)
            if time.monotonic() >= deadline:
                raise CommanderError(
                    f"no answer like a SimpleFOC sketch at {self.baud} baud. Check the baud rate, that the sketch "
                    "uses Commander (command.run() in loop()), and that TX/RX aren't swapped")

    # --- commands --------------------------------------------------------------------------
    def query(self, commands: Sequence[str], timeout: float | None = None) -> list[str | None]:
        """Send ``commands`` and return each reply's payload (after its prefix); None for ``err``."""
        for command in commands:
            if len(command) + 1 > MAX_COMMAND_LENGTH:
                raise ValueError(f"command too long for Commander: {command!r}")
        self._write("".join(f"{c}\n" for c in commands))
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout) + 0.004 * len(commands)
        replies: list[str | None] = []
        for command in commands:
            payload = self._next_reply(command, deadline)
            if payload is None:
                self._resync()
                raise BackendError(f"no reply to {command!r} from the board")
            replies.append(None if is_error(payload) else payload)
        return replies

    def ask(self, command: str, timeout: float | None = None) -> str | None:
        return self.query([command], timeout)[0]

    def send(self, command: str) -> None:
        """Send a command without waiting for its reply (only before closing the port)."""
        self._write(f"{command}\n")

    def scan(self) -> dict[str, str]:
        """The command letters the sketch registered, with their labels (``?``)."""
        self._write("?\n")
        found: dict[str, str] = {}
        deadline = time.monotonic() + self.timeout
        while True:
            line = self._read_line(deadline)
            if line is None:
                return found
            if line.startswith("?") and len(line) >= 3 and line[2] == ":":
                found[line[1]] = line[3:].strip()
                deadline = time.monotonic() + 0.1  # the list ends when nothing more comes
            elif line:
                self._note(line)

    def _next_reply(self, command: str, deadline: float) -> str | None:
        prefix = reply_prefix(command)
        target_value = len(prefix) == 1 and prefix.isalpha()
        while True:
            line = self._read_line(deadline)
            if line is None:
                return None
            if line.startswith(prefix):
                payload = line[len(prefix):]
                # A target reply is just a number after the letter; "MOT: Init" isn't one.
                if not target_value or parse_number(payload) is not None or is_error(payload):
                    return payload
            if line:
                self._note(line)

    def _note(self, line: str) -> None:
        self.device_messages.append(line)
        log.debug("Board says: %s", line)

    # --- low level -------------------------------------------------------------------------
    def _write(self, text: str) -> None:
        if self._ser is None:
            raise ConnectionLost("serial port is not open")
        try:
            self._ser.write(text.encode("ascii"))
        except (OSError, ValueError) as exc:
            raise ConnectionLost(f"serial write failed ({exc})") from exc
        except Exception as exc:  # pyserial's SerialTimeoutException and friends
            raise ConnectionLost(f"serial write failed ({type(exc).__name__}: {exc})") from exc

    def _read_line(self, deadline: float) -> str | None:
        """The next complete line. A read that times out mid-line keeps what came so far."""
        if self._ser is None:
            raise ConnectionLost("serial port is not open")
        while True:
            end = self._buffer.find(b"\n")
            if end >= 0:
                line = bytes(self._buffer[:end])
                del self._buffer[:end + 1]
                return line.decode("ascii", errors="replace").strip()
            if time.monotonic() >= deadline:
                return None
            try:
                raw = self._ser.readline()
            except (OSError, ValueError) as exc:
                raise ConnectionLost(f"serial read failed ({exc})") from exc
            except Exception as exc:
                raise ConnectionLost(f"serial read failed ({type(exc).__name__}: {exc})") from exc
            if raw:
                self._buffer.extend(raw)

    def _resync(self, wait: float = 0.02) -> None:
        self._buffer.clear()
        try:
            time.sleep(wait)
            self._ser.reset_input_buffer()
        except (OSError, ValueError, AttributeError):
            pass
