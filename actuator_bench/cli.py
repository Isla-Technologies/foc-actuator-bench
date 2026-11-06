"""Command line: start the app (default), list ports, show device info, or monitor torque.

``info`` and ``monitor`` only read from the controller; they never move the motor.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

from . import APP_NAME, __version__

log = logging.getLogger(__name__)


def _add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--controller", choices=["odrive", "simplefoc"], default="odrive",
                        help="the motor controller (default: odrive)")
    parser.add_argument("--connection", choices=["auto", "serial", "native", "simulator"], default=None,
                        help="how to reach it (default: auto for the ODrive, serial for SimpleFOC)")
    parser.add_argument("--port", help="serial port, e.g. COM5 or /dev/cu.usbmodem1234 (default: auto-detect)")
    parser.add_argument("--axis", type=int, choices=[0, 1], default=0, help="ODrive: 0 = M0, 1 = M1 (default: 0)")
    parser.add_argument("--baud", type=int, default=115200, help="SimpleFOC: the sketch's baud rate (default 115200)")
    parser.add_argument("--letter", default="M", help="SimpleFOC: the motor's Commander letter (default M)")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="foc-actuator-bench",
        description=f"{APP_NAME} {__version__}: measure the torque of an actuator on an ODrive V3.6 or a "
                    "SimpleFOC board.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument("--debug", action="store_true", help="verbose logging")
    parser.add_argument("--sim", action="store_true", help="start the app with the simulator selected")
    sub = parser.add_subparsers(dest="command")

    gui = sub.add_parser("gui", help="start the app (default)")
    gui.add_argument("--sim", action="store_true", help="start with the simulator selected")

    sub.add_parser("ports", help="list serial ports (ODrives and USB-serial adapters are marked)")

    info = sub.add_parser("info", help="show controller info, configuration and errors (read-only)")
    _add_connection_args(info)

    monitor = sub.add_parser("monitor", help="print live torque (read-only, never moves the motor)")
    _add_connection_args(monitor)
    constant = monitor.add_mutually_exclusive_group()
    constant.add_argument("--kv", type=float, help="motor KV [rpm/V]")
    constant.add_argument("--kt", type=float, help="motor torque constant [N·m/A] (default: the ODrive's value)")
    monitor.add_argument("--gear", type=float, default=1.0, help="gear ratio (default 1)")
    monitor.add_argument("--efficiency", type=float, default=100.0, help="gearbox efficiency in %% (default 100)")
    monitor.add_argument("--seconds", type=float, default=0.0, help="stop after N seconds (default: Ctrl+C)")
    monitor.add_argument("--rate", type=float, default=50.0, help="samples per second (default 50)")
    monitor.add_argument("--csv", help="also record to this CSV file")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command in (None, "gui"):
        from .app import run_gui

        return run_gui(sim=args.sim, debug=args.debug)

    from .diagnostics import setup_logging

    setup_logging(args.debug)
    if args.command == "ports":
        return cmd_ports()
    if args.command == "info":
        return cmd_info(args)
    if args.command == "monitor":
        return cmd_monitor(args)
    return 2


def cmd_ports() -> int:
    from .backends.serial_backend import list_serial_ports

    ports = list_serial_ports()
    if not ports:
        print("No serial ports found. Is the controller powered and the USB cable a data cable?")
        return 1
    for port in ports:
        print(port.label)
    return 0


def _connect(args):
    from .acquisition import ConnectRequest, open_controller

    mode = args.connection or ("auto" if args.controller == "odrive" else "serial")
    dev, _sim = open_controller(ConnectRequest(mode=mode, port=args.port, axis=args.axis, controller=args.controller,
                                               baud=args.baud, motor_letter=args.letter))
    return dev


def cmd_info(args) -> int:
    from .backends.base import BackendError
    from .control import state_label

    try:
        dev = _connect(args)
    except BackendError as exc:
        print(f"Could not connect: {exc}")
        return 1
    try:
        info = dev.read_info()
        readiness = dev.readiness()
        config = dev.config_snapshot()
        print(f"Connection : {info.connection}")
        print(f"Hardware   : {info.board}")
        print(f"Firmware   : {info.firmware}{'' if info.firmware_supported else '  (not 0.5.x: unsupported)'}")
        print(f"Serial     : {info.serial}")
        print(f"Motor      : {state_label(readiness.state)}; motor calibrated: {readiness.motor_calibrated}; "
              f"encoder ready: {readiness.encoder_ready}")
        for kind, text in readiness.checks:
            print(f"  [{kind}] {text}")
        if readiness.errors:
            print("Errors:")
            for flag in readiness.errors:
                print(f"  {flag.component}: {flag.name}: {flag.explanation}")
        else:
            print("Errors     : none")
        print("Configuration:")
        for path, value in config.items():
            print(f"  {path} = {'(not available)' if value is None else value}")
    except BackendError as exc:
        print(f"Communication error: {exc}")
        return 1
    finally:
        dev.close()
    return 0


def cmd_monitor(args) -> int:
    from .acquisition import make_sample
    from .backends.base import BackendError
    from .recorder import CsvRecorder
    from .samples import SampleBatch
    from .torque import TorqueModel, TorqueStats, kt_from_kv

    try:
        dev = _connect(args)
    except BackendError as exc:
        print(f"Could not connect: {exc}")
        return 1
    recorder = None
    try:
        info = dev.read_info()
        readiness = dev.readiness()
        kt = args.kt or (kt_from_kv(args.kv) if args.kv else readiness.kt_device)
        if not kt:
            print(f"The {dev.name} has no torque constant to read; pass --kv or --kt.")
            return 1
        if not (args.kt or args.kv):
            print(f"Using the {dev.name}'s torque constant = {kt:.5f} N·m/A (pass --kv or --kt to override).")
        model = TorqueModel(kt_motor=kt, gear_ratio=args.gear, efficiency=args.efficiency / 100.0)
        if args.csv:
            recorder = CsvRecorder(args.csv, {"source": "monitor", "device": info.board, "firmware": info.firmware,
                                              "kt_motor_Nm_per_A": kt, "gear_ratio": args.gear,
                                              "efficiency_pct": args.efficiency})
        print(f"Monitoring {info.board} ({info.firmware}) via {info.connection}. Ctrl+C to stop.")
        print(f"{'time s':>8} {'torque N·m':>11} {'Iq A':>8} {'speed rpm':>10} {'angle °':>9} {'Vbus V':>7} {'FET °C':>7}")
        stats = TorqueStats()
        period = 1.0 / max(1.0, args.rate)
        t0 = time.monotonic()
        slow = dev.read_slow()
        last_slow = last_print = t0
        pos_zero = None
        buffer = []
        try:
            while not args.seconds or time.monotonic() - t0 < args.seconds:
                now = time.monotonic()
                fast = dev.read_fast()
                if pos_zero is None:
                    pos_zero = fast.pos
                if now - last_slow >= 0.2:
                    slow, last_slow = dev.read_slow(), now
                sample = make_sample(model, now - t0, fast, slow, pos_zero, slow.state)
                buffer.append(sample)
                if len(buffer) >= 10:
                    batch = SampleBatch.from_samples(buffer)
                    stats.add(batch["t"], batch["torque_out"])
                    if recorder:
                        recorder.write(batch)
                    buffer = []
                if now - last_print >= 0.25:
                    last_print = now
                    print(f"{sample.t:8.2f} {sample.torque_out:11.4f} {sample.iq:8.3f} {sample.speed_out_rpm:10.2f} "
                          f"{sample.angle_out_deg:9.2f} {sample.vbus:7.2f} {sample.fet_temp:7.1f}")
                time.sleep(max(0.0, period - (time.monotonic() - now)))
        except KeyboardInterrupt:
            pass
        if buffer:
            batch = SampleBatch.from_samples(buffer)
            stats.add(batch["t"], batch["torque_out"])
            if recorder:
                recorder.write(batch)
        print(f"\nPeak {stats.peak_abs:.4f} N·m (+{stats.peak_pos:.4f} / {stats.peak_neg:.4f}), "
              f"mean {stats.mean:.4f} N·m, RMS {stats.rms:.4f} N·m over {stats.duration:.1f} s")
    except BackendError as exc:
        print(f"Communication error: {exc}")
        return 1
    finally:
        if recorder:
            print(f"Saved {recorder.close()['file']}")
        dev.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
