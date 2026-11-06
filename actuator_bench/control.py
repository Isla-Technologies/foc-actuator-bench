"""The words the app uses for any motor controller: control modes, motor states and faults.

The numbers follow the ODrive's (the first controller the app supported) so that recordings from
every controller read the same: ``state`` 8 is "motor on" and 1 is "motor off" in every CSV file.
Other controllers report the subset that applies to them.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class ControlMode(IntEnum):
    VOLTAGE = 0
    TORQUE = 1
    VELOCITY = 2
    POSITION = 3


class MotorState(IntEnum):
    """What the app needs to know about a motor's state. Same values as the ODrive's AxisState."""

    UNKNOWN = 0
    IDLE = 1
    CALIBRATING = 3
    CLOSED_LOOP = 8


# The ODrive reports a few more states (calibration steps, homing); they share the numbering.
_STATE_LABELS = {
    0: "Undefined",
    1: "Idle (motor off)",
    2: "Starting up",
    3: "Calibrating",
    4: "Calibrating motor",
    6: "Searching encoder index",
    7: "Calibrating encoder",
    8: "Closed loop (motor active)",
    9: "Lock-in spin",
    10: "Finding encoder direction",
    11: "Homing",
    12: "Calibrating hall polarity",
    13: "Calibrating hall phase",
}


def state_label(value: int | None) -> str:
    if value is None:
        return "Unknown"
    return _STATE_LABELS.get(int(value), f"State {int(value)}")


@dataclass(frozen=True)
class ErrorFlag:
    """One fault a controller reports, with a plain-English explanation."""

    component: str
    name: str
    value: int
    explanation: str

    def __str__(self) -> str:
        return f"{self.component}: {self.name}"


def decode_flags(component: str, value: int | float | None,
                 table: dict[int, tuple[str, str]]) -> list[ErrorFlag]:
    """Split an error register into its flags; bits missing from ``table`` are reported too."""
    if not value:
        return []
    value = int(value)
    flags: list[ErrorFlag] = []
    remaining = value
    for bit, (name, explanation) in table.items():
        if value & bit:
            flags.append(ErrorFlag(component, name, bit, explanation))
            remaining &= ~bit
    bit = 1
    while remaining:
        if remaining & bit:
            flags.append(ErrorFlag(component, f"UNKNOWN_0x{bit:X}", bit, "Unknown error bit (newer/older firmware?)."))
            remaining &= ~bit
        bit <<= 1
    return flags
