"""What the app needs from a motor controller, whichever one drives the actuator.

The acquisition worker, the tests and the Connect page only use this interface. Each supported
controller implements it in ``controllers/``: the ODrive V3.6 (``controllers/odrive.py``) and
SimpleFOC boards (``controllers/simplefoc.py``). Units are the same for all of them:

* current: amps of Iq, the torque-producing current (positive = forward torque)
* position: motor turns; speed: motor turns per second (before the gearbox)

Only the acquisition worker thread uses a controller.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass, field

from .backends.base import BackendError
from .control import ControlMode, ErrorFlag

ARM_TIMEOUT_S = 1.5

# Controllers that don't cap the speed themselves (SimpleFOC) are switched off above this speed,
# like the ODrive's overspeed error (vel_limit_tolerance 1.2): by the bench firmware in its control
# loop, and by the app at the sample rate as a backup.
OVERSPEED_TOLERANCE = 1.2
OVERSPEED_MARGIN = 0.1               # motor turns/s, so noise at tiny speed limits doesn't trip it


def overspeed_trip(vel_limit: float) -> float:
    """Motor turns/s above which the motor is switched off, for a speed limit of ``vel_limit``."""
    return OVERSPEED_TOLERANCE * abs(vel_limit) + OVERSPEED_MARGIN


class ArmError(Exception):
    """The motor did not switch on (enter closed-loop control)."""

    def __init__(self, message: str, errors: list[ErrorFlag] | None = None):
        super().__init__(message)
        self.errors = errors or []


@dataclass(frozen=True)
class Capabilities:
    """What a controller can do beyond the basics (read current, position and speed; arm; command)."""

    watchdog: bool = False                # switches the motor off if the app stops talking to it
    motor_calibration: bool = False       # the app can start the controller's motor/encoder calibration
    write_torque_constant: bool = False   # has its own torque constant the app can set
    clear_errors: bool = False
    bus_voltage: bool = False             # reports its supply voltage
    driver_temperature: bool = False
    torque_mode_speed_limit: bool = False  # caps the speed by itself in torque mode


@dataclass
class DeviceInfo:
    connection: str
    backend_kind: str                 # serial | native | simulator | commander
    hardware: str
    firmware: str
    fw_version: tuple[int, int, int] | None
    serial: str
    controller: str = "ODrive"        # the kind of controller, as people call it
    family: str = "odrive"            # odrive | simplefoc
    capabilities: Capabilities = field(default_factory=Capabilities)

    @property
    def board(self) -> str:
        """The controller and its hardware, e.g. ``ODrive 3.6-56V``."""
        if not self.hardware or self.hardware == "unknown":
            return self.controller
        return f"{self.controller} {self.hardware}"

    @property
    def firmware_supported(self) -> bool:
        if self.family != "odrive":
            return True
        return self.fw_version is not None and self.fw_version[:2] == (0, 5)

    @property
    def firmware_has_ascii_bugs(self) -> bool:
        """ODrive 0.5.2/0.5.3 rejected some ASCII writes (fixed in 0.5.4)."""
        return (self.family == "odrive" and self.fw_version is not None and self.fw_version[:2] == (0, 5)
                and self.fw_version[2] in (2, 3))


@dataclass
class FastReading:
    iq: float
    iq_set: float
    pos: float
    vel: float


@dataclass
class SlowReading:
    vbus: float = math.nan
    ibus: float = math.nan
    fet_temp: float = math.nan        # the driver's own temperature (the ODrive's FET thermistor)
    motor_temp: float = math.nan
    state: int | None = None
    errors: list[ErrorFlag] = field(default_factory=list)
    raw_errors: dict[str, int] = field(default_factory=dict)
    sample_rate_hz: float = 0.0


@dataclass
class Readiness:
    state: int | None
    motor_calibrated: bool | None
    encoder_ready: bool | None
    errors: list[ErrorFlag]
    kt_device: float | None           # the torque constant configured inside the controller, if it has one
    current_lim: float | None         # the controller's own current limit, above which no test can go
    vel_limit: float | None
    vel_gain: float | None
    vbus: float | None
    # Torque mode on controllers that cap torque near their speed limit (the ODrive): amps of
    # torque allowed per motor turn/s below the speed limit. None: no such cap.
    speed_limiter_gain: float | None = None
    # Checks only this kind of controller knows about, as (kind, text) for the Connect page, with
    # kind one of "ok", "warn", "bad" or "info".
    checks: list[tuple[str, str]] = field(default_factory=list)
    # Why the motor can't be switched on, if the controller knows for sure (empty otherwise).
    blocked: str = ""

    @property
    def can_arm(self) -> bool:
        if self.blocked or self.errors:
            return False
        return self.motor_calibrated is not False and self.encoder_ready is not False


@dataclass
class MotorParameters:
    """Motor constants a controller knows (measured or configured); None where it doesn't."""

    phase_resistance: float | None = None   # ohm
    phase_inductance: float | None = None   # H
    pole_pairs: int | None = None


def phase_voltage_ratio(modulation_ratio: float, voltage_cap: float | None, vbus: float) -> float:
    """Phase volts per supply volt at ``vbus``: the modulation's share, unless a voltage cap comes first."""
    if voltage_cap and vbus and math.isfinite(vbus) and vbus > 0:
        return min(modulation_ratio, voltage_cap / vbus)
    return modulation_ratio


class MotorController(ABC):
    """One motor on one controller, seen the same way whatever the hardware.

    Commands are in amps (torque), motor turns/s (speed) and motor turns (position). Settings the
    app changes are remembered on connect and put back on disconnect; nothing is saved permanently.
    """

    family = ""                        # odrive | simplefoc
    name = "controller"                # how messages name it: "The ODrive switched the motor off."
    capabilities = Capabilities()

    def __init__(self) -> None:
        self.motor_thermistor_enabled = False
        # The motor's torque constant from Setup [N·m/A]: controllers that take torque in N·m
        # convert with it when they can't read their own.
        self.motor_kt: float | None = None

    # --- connection ------------------------------------------------------------------------
    @property
    @abstractmethod
    def description(self) -> str:
        """How it is connected, e.g. ``USB serial (COM5)``."""

    @property
    def backend_kind(self) -> str:
        return ""

    @abstractmethod
    def close(self) -> None: ...

    @property
    def device_messages(self) -> list[str]:
        """Recent unsolicited output from the controller, for debug reports."""
        return []

    # --- reading ---------------------------------------------------------------------------
    @abstractmethod
    def read_info(self) -> DeviceInfo: ...

    @abstractmethod
    def read_fast(self) -> FastReading:
        """Current, position and speed: read on every sample."""

    @abstractmethod
    def read_slow(self) -> SlowReading:
        """Voltage, temperatures, state and errors: read a few times per second."""

    def read_errors(self) -> list[ErrorFlag]:
        return self.read_slow().errors

    def current_state(self) -> int | None:
        return self.read_slow().state

    @abstractmethod
    def readiness(self) -> Readiness: ...

    def config_snapshot(self) -> dict[str, object]:
        """Settings worth putting in a debug report."""
        return {}

    def motor_parameters(self) -> MotorParameters:
        return MotorParameters()

    @property
    @abstractmethod
    def modulation_ratio(self) -> float:
        """Largest phase voltage (peak) the controller makes from each volt of supply."""

    @property
    def voltage_cap(self) -> float | None:
        """A fixed limit on the phase voltage (peak) [V], if the controller has one (SimpleFOC's voltage_limit)."""
        return None

    def phase_voltage_ratio(self, vbus: float) -> float:
        """Largest phase voltage (peak) the controller applies, per volt of supply, at ``vbus``."""
        return phase_voltage_ratio(self.modulation_ratio, self.voltage_cap, vbus)

    # --- settings we may change ------------------------------------------------------------
    def remember_config(self) -> None:
        """Note the settings this session may change, to put them back on disconnect."""

    def restore_config(self) -> None:
        """Put back every setting this session changed."""

    # --- control ---------------------------------------------------------------------------
    @abstractmethod
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
        """Switch the motor on in ``mode`` without any jump in position or torque.

        ``vel_limit`` is in motor turns/s, ``current_lim`` in amps. Raises ArmError on failure.
        """

    @abstractmethod
    def change_mode(self, mode: ControlMode, *, torque_mode_vel_limit: bool = True) -> None:
        """Switch control mode while the motor is on."""

    @abstractmethod
    def set_limits(self, *, vel_limit: float, current_lim: float) -> None: ...

    @abstractmethod
    def disarm(self) -> None:
        """Switch the motor off and zero the setpoints."""

    @abstractmethod
    def estop(self) -> None:
        """Best effort: switch the motor off, ignoring communication problems."""

    @abstractmethod
    def set_current(self, amps: float) -> None:
        """Torque mode: the Iq setpoint."""

    @abstractmethod
    def set_velocity(self, turns_per_s: float) -> None: ...

    @abstractmethod
    def set_position(self, turns: float, vel_ff: float | None = None) -> None: ...

    # --- watchdog --------------------------------------------------------------------------
    def enable_watchdog(self, timeout_s: float) -> None:
        """Controllers without a watchdog ignore this (see ``capabilities.watchdog``)."""

    def feed_watchdog(self) -> None: ...

    def disable_watchdog(self) -> None: ...

    @property
    def watchdog_on(self) -> bool:
        return False

    # --- maintenance -----------------------------------------------------------------------
    def clear_errors(self) -> list[ErrorFlag]:
        """Clear the errors; returns the ones that remain."""
        return self.read_errors()

    def start_calibration(self) -> None:
        raise BackendError(f"the {self.name} can't run a motor calibration from this app")

    def apply_torque_constant(self, kt: float) -> None:
        raise BackendError(f"the {self.name} has no torque constant to set")
