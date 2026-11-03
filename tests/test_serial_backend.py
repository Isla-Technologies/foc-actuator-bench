"""The ASCII-over-USB-serial backend, tested against a fake port that replies like the firmware."""

import math

import pytest

from actuator_bench.backends.base import BackendError, PropertyUnavailable
from actuator_bench.backends.serial_backend import SerialBackend, format_value, frame_command, parse_number, xor_checksum
from actuator_bench.backends.sim_serial import SimulatedSerialPort
from actuator_bench.backends.simulator import ManualClock, SimActuator, SimulatedODrive


@pytest.fixture
def setup():
    clock = ManualClock()
    sim = SimulatedODrive(SimActuator(scenario="locked"), clock=clock)
    ports = []

    def opener(_device):
        port = SimulatedSerialPort(sim)
        ports.append(port)
        return port

    backend = SerialBackend("FAKE", opener=opener, timeout=0.05)
    backend.connect()
    return backend, sim, ports[-1], clock


def test_checksum_matches_documentation_example():
    # docs.odriverobotics.com ASCII protocol: "r vbus_voltage *93"
    assert frame_command("r vbus_voltage", True) == "r vbus_voltage *93\n"
    assert frame_command("r vbus_voltage", False) == "r vbus_voltage\n"
    assert xor_checksum("") == 0


@pytest.mark.parametrize("text, expected", [
    ("24.087744", 24.087744), ("8", 8), ("1", 1), ("-0.5", -0.5), ("true", 1), ("false", 0),
    ("0d", 0),  # buggy 0.5.2/0.5.3 output
])
def test_parse_number(text, expected):
    assert parse_number(text) == expected


def test_parse_special_values():
    assert parse_number("inf") == math.inf
    assert parse_number("-inf") == -math.inf
    assert math.isnan(parse_number("nan"))
    assert parse_number("lu") is None
    assert parse_number("") is None


def test_format_value():
    assert format_value(True) == "1"
    assert format_value(3) == "3"
    assert format_value(0.1) == "0.1"
    assert format_value(math.inf) == "inf"


def test_connect_reads_identity(setup):
    backend, _, port, _ = setup
    ident = backend.identity()
    assert ident == {"hardware": "3.6-56V", "firmware": "0.5.6", "serial": "3348373D3432"}
    assert backend.use_checksums
    assert any(line.startswith("r fw_version_major *") for line in port.received_lines)


def test_read_many_and_invalid_property(setup):
    backend, sim, _, _ = setup
    values = backend.read_many(["vbus_voltage", "axis0.current_state", "axis0.nonexistent", "axis0.encoder.is_ready"])
    assert values[0] == pytest.approx(24.0, abs=0.5)
    assert values[1] == 1
    assert values[2] is None
    assert values[3] == 1
    with pytest.raises(PropertyUnavailable):
        backend.read("axis0.nonexistent")


def test_values_are_truncated_like_firmware(setup):
    backend, sim, _, _ = setup
    sim.write("axis0.controller.config.vel_limit", 123456.789)
    assert backend.read("axis0.controller.config.vel_limit") == pytest.approx(123456.78)


def test_write_and_call(setup):
    backend, sim, _, _ = setup
    backend.write("axis0.controller.config.vel_limit", 7.5)
    backend.write("axis0.config.enable_watchdog", False)
    assert sim.read("axis0.controller.config.vel_limit") == 7.5
    sim.inject_error(0, "controller", 0x1)
    assert backend.read("axis0.error") != 0
    backend.call("clear_errors")  # 'sc'
    assert backend.read("axis0.error") == 0
    backend.call("axis0.watchdog_feed")  # 'u 0'
    with pytest.raises(PropertyUnavailable):
        backend.call("axis0.unknown_function")


def test_stray_output_is_skipped(setup):
    backend, _, port, _ = setup
    port.inject_line("some debug output")
    assert backend.read("axis0.current_state") == 1
    assert "some debug output" in backend.device_messages


def test_failed_write_does_not_shift_later_replies(setup):
    backend, _, _, _ = setup
    backend.write("axis0.encoder.pos_estimate", 5.0)  # read-only: firmware replies "invalid property"
    values = backend.read_many(["fw_version_minor", "fw_version_revision"])
    assert values == [5, 6]
    assert "invalid property" in backend.device_messages


def test_missing_reply_times_out(setup):
    backend, _, port, _ = setup
    port.close()
    with pytest.raises(BackendError):
        backend.read("vbus_voltage")


def test_non_odrive_port_is_rejected():
    class Silent(SimulatedSerialPort):
        def write(self, data):
            return len(data)

    sim = SimulatedODrive(clock=ManualClock())
    backend = SerialBackend("SILENT", opener=lambda _d: Silent(sim), timeout=0.02)
    with pytest.raises(BackendError, match="didn't answer like an ODrive"):
        backend.connect()


def test_falls_back_without_checksums_for_old_firmware():
    class NoChecksums(SimulatedSerialPort):
        def _respond(self, use_checksum, text):
            super()._respond(False, text)

    sim = SimulatedODrive(clock=ManualClock())
    backend = SerialBackend("OLD", opener=lambda _d: NoChecksums(sim), timeout=0.02)
    backend.connect()
    assert not backend.use_checksums
    assert backend.read("fw_version_minor") == 5
