"""Facts about the ODrive v3.x firmware 0.5.x API that the app relies on.

Property paths, enum values and error bits come from the official 0.5.6 API reference
(https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html) and the
firmware source. Only the subset this app needs is listed.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

ODRIVE_USB_VID = 0x1209
ODRIVE_USB_PIDS = (0x0D32,)  # ODrive v3.x running application firmware


class AxisState(IntEnum):
    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11
    ENCODER_HALL_POLARITY_CALIBRATION = 12
    ENCODER_HALL_PHASE_CALIBRATION = 13


class ControlMode(IntEnum):
    VOLTAGE = 0
    TORQUE = 1
    VELOCITY = 2
    POSITION = 3


class InputMode(IntEnum):
    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ = 5
    TORQUE_RAMP = 6
    MIRROR = 7
    TUNING = 8


CALIBRATION_STATES = {
    AxisState.FULL_CALIBRATION_SEQUENCE,
    AxisState.MOTOR_CALIBRATION,
    AxisState.ENCODER_INDEX_SEARCH,
    AxisState.ENCODER_OFFSET_CALIBRATION,
    AxisState.ENCODER_DIR_FIND,
    AxisState.ENCODER_HALL_POLARITY_CALIBRATION,
    AxisState.ENCODER_HALL_PHASE_CALIBRATION,
}

_STATE_LABELS = {
    AxisState.UNDEFINED: "Undefined",
    AxisState.IDLE: "Idle (motor off)",
    AxisState.STARTUP_SEQUENCE: "Starting up",
    AxisState.FULL_CALIBRATION_SEQUENCE: "Calibrating",
    AxisState.MOTOR_CALIBRATION: "Calibrating motor",
    AxisState.ENCODER_INDEX_SEARCH: "Searching encoder index",
    AxisState.ENCODER_OFFSET_CALIBRATION: "Calibrating encoder",
    AxisState.CLOSED_LOOP_CONTROL: "Closed loop (motor active)",
    AxisState.LOCKIN_SPIN: "Lock-in spin",
    AxisState.ENCODER_DIR_FIND: "Finding encoder direction",
    AxisState.HOMING: "Homing",
    AxisState.ENCODER_HALL_POLARITY_CALIBRATION: "Calibrating hall polarity",
    AxisState.ENCODER_HALL_PHASE_CALIBRATION: "Calibrating hall phase",
}


def state_label(value: int | None) -> str:
    if value is None:
        return "Unknown"
    try:
        return _STATE_LABELS[AxisState(int(value))]
    except ValueError:
        return f"State {int(value)}"


# --- Error codes -------------------------------------------------------------------------------
# Each table maps bit value -> (NAME, plain-English explanation).

SYSTEM_ERRORS = {
    0x1: ("CONTROL_ITERATION_MISSED", "The ODrive's control loop missed a cycle. Usually transient: clear errors and retry."),
    0x2: ("DC_BUS_UNDER_VOLTAGE", "Supply voltage dropped too low. Check the power supply or battery, its current rating and the wiring."),
    0x4: ("DC_BUS_OVER_VOLTAGE", "Supply voltage went too high, usually from energy fed back while braking. Connect/enable a brake resistor or decelerate more gently."),
    0x8: ("DC_BUS_OVER_REGEN_CURRENT", "Too much current flowed back into the supply while braking. Use a brake resistor or lower dc_max_negative_current."),
    0x10: ("DC_BUS_OVER_CURRENT", "Too much current drawn from the supply. Lower the current limit or use a stronger supply."),
    0x20: ("BRAKE_DEADTIME_VIOLATION", "Brake resistor timing violation. Check the brake resistor configuration."),
    0x40: ("BRAKE_DUTY_CYCLE_NAN", "Brake resistor duty cycle became invalid."),
    0x80: ("INVALID_BRAKE_RESISTANCE", "The configured brake resistance is invalid."),
}

AXIS_ERRORS = {
    0x1: ("INVALID_STATE", "The requested state isn't possible right now. Usually the motor or encoder isn't calibrated, or another error is active."),
    0x40: ("MOTOR_FAILED", "The motor reported a problem (see the motor error)."),
    0x80: ("SENSORLESS_ESTIMATOR_FAILED", "The sensorless estimator failed."),
    0x100: ("ENCODER_FAILED", "The encoder reported a problem (see the encoder error)."),
    0x200: ("CONTROLLER_FAILED", "The controller reported a problem (see the controller error)."),
    0x800: ("WATCHDOG_TIMER_EXPIRED", "The ODrive stopped hearing from this app (cable unplugged or app frozen) and switched the motor off for safety."),
    0x1000: ("MIN_ENDSTOP_PRESSED", "The minimum endstop switch was triggered."),
    0x2000: ("MAX_ENDSTOP_PRESSED", "The maximum endstop switch was triggered."),
    0x4000: ("ESTOP_REQUESTED", "An emergency stop was requested."),
    0x20000: ("HOMING_WITHOUT_ENDSTOP", "Homing was requested but no endstop is configured."),
    0x40000: ("OVER_TEMP", "Over temperature. Let the motor and ODrive cool down."),
    0x80000: ("UNKNOWN_POSITION", "Position unknown: the encoder isn't ready."),
}

MOTOR_ERRORS = {
    0x1: ("PHASE_RESISTANCE_OUT_OF_RANGE", "Motor calibration measured an unexpected phase resistance. Check the motor wires and calibration_current."),
    0x2: ("PHASE_INDUCTANCE_OUT_OF_RANGE", "Motor calibration measured an unexpected phase inductance. Check the motor wiring."),
    0x8: ("DRV_FAULT", "Gate driver fault, often caused by a supply voltage dip, overcurrent or a hardware problem."),
    0x10: ("CONTROL_DEADLINE_MISSED", "Internal timing problem in the current controller."),
    0x80: ("MODULATION_MAGNITUDE", "The motor needed more voltage than available: bus voltage too low for this speed/current, or gains too high."),
    0x400: ("CURRENT_SENSE_SATURATION", "The current sensors saturated: current exceeded the measurable range."),
    0x1000: ("CURRENT_LIMIT_VIOLATION", "Motor current exceeded the configured limit plus margin. Lower the gains or raise current_lim_margin."),
    0x10000: ("MODULATION_IS_NAN", "Invalid modulation value (internal error)."),
    0x20000: ("MOTOR_THERMISTOR_OVER_TEMP", "Motor temperature too high. Let it cool down."),
    0x40000: ("FET_THERMISTOR_OVER_TEMP", "ODrive power stage (FET) temperature too high. Let it cool down or improve cooling."),
    0x80000: ("TIMER_UPDATE_MISSED", "Internal timer update missed."),
    0x100000: ("CURRENT_MEASUREMENT_UNAVAILABLE", "Current measurement unavailable (internal error)."),
    0x200000: ("CONTROLLER_FAILED", "Controller failure inside the motor loop."),
    0x400000: ("I_BUS_OUT_OF_RANGE", "Bus current out of the allowed range. Check dc_max_positive_current / dc_max_negative_current."),
    0x800000: ("BRAKE_RESISTOR_DISARMED", "The brake resistor was disarmed (often follows another error)."),
    0x1000000: ("SYSTEM_LEVEL", "A system-level error occurred (see the ODrive system error)."),
    0x2000000: ("BAD_TIMING", "Bad timing (internal error)."),
    0x4000000: ("UNKNOWN_PHASE_ESTIMATE", "Rotor angle unknown: encoder not ready or not calibrated."),
    0x8000000: ("UNKNOWN_PHASE_VEL", "Rotor speed unknown."),
    0x10000000: ("UNKNOWN_TORQUE", "Torque setpoint unknown (internal error)."),
    0x20000000: ("UNKNOWN_CURRENT_COMMAND", "Current command unknown (internal error)."),
    0x40000000: ("UNKNOWN_CURRENT_MEASUREMENT", "Current measurement unknown (internal error)."),
    0x80000000: ("UNKNOWN_VBUS_VOLTAGE", "Bus voltage unknown (internal error)."),
    0x100000000: ("UNKNOWN_VOLTAGE_COMMAND", "Voltage command unknown (internal error)."),
    0x200000000: ("UNKNOWN_GAINS", "Current controller gains unknown: the motor isn't calibrated."),
    0x400000000: ("CONTROLLER_INITIALIZING", "The controller is still initializing."),
    0x800000000: ("UNBALANCED_PHASES", "The motor phases are unbalanced. Check the motor wiring and connectors."),
}

ENCODER_ERRORS = {
    0x1: ("UNSTABLE_GAIN", "Encoder estimator gains are unstable. Check the encoder bandwidth setting."),
    0x2: ("CPR_POLEPAIRS_MISMATCH", "Encoder CPR and pole pairs don't match what calibration measured. Check encoder cpr, pole_pairs and that the encoder isn't slipping."),
    0x4: ("NO_RESPONSE", "No movement seen from the encoder during calibration. Check encoder wiring and that the motor can turn freely."),
    0x8: ("UNSUPPORTED_ENCODER_MODE", "Unsupported encoder mode."),
    0x10: ("ILLEGAL_HALL_STATE", "Invalid hall sensor state. Check the hall wiring for noise."),
    0x20: ("INDEX_NOT_FOUND_YET", "Encoder index pulse not found yet. Run the index search."),
    0x40: ("ABS_SPI_TIMEOUT", "Absolute SPI encoder timed out. Check SPI wiring."),
    0x80: ("ABS_SPI_COM_FAIL", "Absolute SPI encoder communication failed. Check SPI wiring."),
    0x100: ("ABS_SPI_NOT_READY", "Absolute SPI encoder not ready."),
    0x200: ("HALL_NOT_CALIBRATED_YET", "Hall sensors are not calibrated yet."),
}

CONTROLLER_ERRORS = {
    0x1: ("OVERSPEED", "The motor exceeded the speed limit (vel_limit x tolerance), e.g. the arm slipped or broke free. Lower the torque or raise the speed limit."),
    0x2: ("INVALID_INPUT_MODE", "Invalid input mode."),
    0x4: ("UNSTABLE_GAIN", "Controller gains are unstable."),
    0x8: ("INVALID_MIRROR_AXIS", "Invalid mirror axis."),
    0x10: ("INVALID_LOAD_ENCODER", "Invalid load encoder."),
    0x20: ("INVALID_ESTIMATE", "Position/velocity estimate unavailable: the encoder isn't ready."),
    0x40: ("INVALID_CIRCULAR_RANGE", "Invalid circular setpoint range."),
    0x80: ("SPINOUT_DETECTED", "Spinout detected: the motor pushes but doesn't move as expected (encoder slipping or bad calibration)."),
}

ERROR_TABLES = {
    "system": SYSTEM_ERRORS,
    "axis": AXIS_ERRORS,
    "motor": MOTOR_ERRORS,
    "encoder": ENCODER_ERRORS,
    "controller": CONTROLLER_ERRORS,
}


@dataclass(frozen=True)
class ErrorFlag:
    component: str
    name: str
    value: int
    explanation: str

    def __str__(self) -> str:
        return f"{self.component}: {self.name}"


def decode_errors(component: str, value: int | float | None) -> list[ErrorFlag]:
    """Split an error register into its individual flags (unknown bits are reported too)."""
    if not value:
        return []
    value = int(value)
    table = ERROR_TABLES[component]
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


# --- Property paths ----------------------------------------------------------------------------

class SystemPaths:
    vbus_voltage = "vbus_voltage"
    ibus = "ibus"
    error = "error"
    fw_version_major = "fw_version_major"
    fw_version_minor = "fw_version_minor"
    fw_version_revision = "fw_version_revision"
    fw_version_unreleased = "fw_version_unreleased"
    hw_version_major = "hw_version_major"
    hw_version_minor = "hw_version_minor"
    hw_version_variant = "hw_version_variant"
    serial_number = "serial_number"
    clear_errors = "clear_errors"  # function


class AxisPaths:
    """Property paths for one axis (``axis0`` = M0, ``axis1`` = M1)."""

    def __init__(self, axis: int):
        if axis not in (0, 1):
            raise ValueError("axis must be 0 or 1")
        self.axis = axis
        a = f"axis{axis}"
        self.current_state = f"{a}.current_state"
        self.requested_state = f"{a}.requested_state"
        self.error = f"{a}.error"
        self.enable_watchdog = f"{a}.config.enable_watchdog"
        self.watchdog_timeout = f"{a}.config.watchdog_timeout"
        self.watchdog_feed = f"{a}.watchdog_feed"  # function

        m = f"{a}.motor"
        self.motor_error = f"{m}.error"
        self.motor_is_calibrated = f"{m}.is_calibrated"
        self.iq_measured = f"{m}.current_control.Iq_measured"
        self.iq_setpoint = f"{m}.current_control.Iq_setpoint"
        self.iq_filter_k = f"{m}.current_control.I_measured_report_filter_k"
        self.i_bus = f"{m}.I_bus"
        self.torque_constant = f"{m}.config.torque_constant"
        self.current_lim = f"{m}.config.current_lim"
        self.torque_lim = f"{m}.config.torque_lim"
        self.pole_pairs = f"{m}.config.pole_pairs"
        self.motor_type = f"{m}.config.motor_type"
        self.fet_temperature = f"{m}.fet_thermistor.temperature"
        self.motor_temperature = f"{m}.motor_thermistor.temperature"
        self.motor_thermistor_enabled = f"{m}.motor_thermistor.config.enabled"

        e = f"{a}.encoder"
        self.encoder_error = f"{e}.error"
        self.encoder_is_ready = f"{e}.is_ready"
        self.pos_estimate = f"{e}.pos_estimate"
        self.vel_estimate = f"{e}.vel_estimate"
        self.encoder_cpr = f"{e}.config.cpr"

        c = f"{a}.controller"
        self.controller_error = f"{c}.error"
        self.input_pos = f"{c}.input_pos"
        self.input_vel = f"{c}.input_vel"
        self.input_torque = f"{c}.input_torque"
        self.control_mode = f"{c}.config.control_mode"
        self.input_mode = f"{c}.config.input_mode"
        self.vel_limit = f"{c}.config.vel_limit"
        self.vel_limit_tolerance = f"{c}.config.vel_limit_tolerance"
        self.enable_vel_limit = f"{c}.config.enable_vel_limit"
        self.enable_torque_mode_vel_limit = f"{c}.config.enable_torque_mode_vel_limit"
        self.enable_overspeed_error = f"{c}.config.enable_overspeed_error"
        self.pos_gain = f"{c}.config.pos_gain"
        self.vel_gain = f"{c}.config.vel_gain"
        self.vel_integrator_gain = f"{c}.config.vel_integrator_gain"

    @property
    def fast(self) -> list[str]:
        """Properties read on every sample."""
        return [self.iq_measured, self.iq_setpoint, self.pos_estimate, self.vel_estimate]

    @property
    def error_registers(self) -> dict[str, str]:
        return {
            "axis": self.error,
            "motor": self.motor_error,
            "encoder": self.encoder_error,
            "controller": self.controller_error,
        }

    @property
    def config_snapshot(self) -> list[str]:
        """Configuration worth recording in debug reports."""
        return [
            self.torque_constant, self.current_lim, self.torque_lim, self.pole_pairs,
            self.motor_type, self.encoder_cpr, self.control_mode, self.input_mode,
            self.vel_limit, self.vel_limit_tolerance, self.enable_vel_limit,
            self.enable_torque_mode_vel_limit, self.enable_overspeed_error,
            self.pos_gain, self.vel_gain, self.vel_integrator_gain, self.iq_filter_k,
            self.enable_watchdog, self.watchdog_timeout, self.motor_is_calibrated,
            self.encoder_is_ready,
        ]

    @property
    def restorable(self) -> list[str]:
        """Settings this app may change during a session; restored on disconnect."""
        return [
            self.control_mode, self.input_mode, self.vel_limit,
            self.enable_torque_mode_vel_limit, self.torque_lim,
            self.enable_watchdog, self.watchdog_timeout,
        ]
