"""Logging setup and the debug report users can send when something goes wrong."""

from __future__ import annotations

import json
import logging
import logging.handlers
import platform
import sys
from collections import deque
from datetime import datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from . import APP_NAME, __version__
from .settings import AppSettings, config_dir


class MemoryLogHandler(logging.Handler):
    """Keeps the most recent log lines for the in-app log view and debug reports."""

    def __init__(self, capacity: int = 2000):
        super().__init__()
        self.lines: deque[str] = deque(maxlen=capacity)
        self.setFormatter(logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self.lines.append(self.format(record))
        except Exception:
            self.handleError(record)


MEMORY_LOG = MemoryLogHandler()


def log_dir() -> Path:
    return config_dir() / "logs"


def setup_logging(debug: bool = False) -> Path:
    """Log to a rotating file, the console and memory. Returns the log file path."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass
    root = logging.getLogger()
    root.setLevel(logging.DEBUG if debug else logging.INFO)
    for handler in list(root.handlers):
        root.removeHandler(handler)
    formatter = logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    path = log_dir() / "actuator_bench.log"
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.handlers.RotatingFileHandler(path, maxBytes=1_000_000, backupCount=3, encoding="utf-8")
        file_handler.setFormatter(formatter)
        root.addHandler(file_handler)
    except OSError:
        pass
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    console.setLevel(logging.INFO)
    root.addHandler(console)
    root.addHandler(MEMORY_LOG)
    logging.getLogger("PIL").setLevel(logging.WARNING)
    return path


def package_version(name: str) -> str:
    try:
        return version(name)
    except PackageNotFoundError:
        return "not installed"


def environment_info() -> dict[str, str]:
    return {
        "app": f"{APP_NAME} {__version__}",
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "machine": platform.machine(),
        "PySide6": package_version("PySide6-Essentials") if package_version("PySide6") == "not installed" else package_version("PySide6"),
        "pyqtgraph": package_version("pyqtgraph"),
        "numpy": package_version("numpy"),
        "pyserial": package_version("pyserial"),
        "odrive (official library)": package_version("odrive"),
    }


def serial_ports_text() -> list[str]:
    try:
        from .backends.serial_backend import list_serial_ports

        return [port.label for port in list_serial_ports()] or ["(none)"]
    except Exception as exc:  # pyserial missing or OS error
        return [f"(could not list ports: {exc})"]


def build_report(snapshot: dict | None, settings: AppSettings | None, extra: dict | None = None) -> str:
    lines = [f"{APP_NAME} debug report", f"Created: {datetime.now().isoformat(timespec='seconds')}", ""]

    def section(title: str) -> None:
        lines.extend(["", f"== {title} =="])

    section("Environment")
    lines += [f"{k}: {v}" for k, v in environment_info().items()]
    section("Serial ports")
    lines += serial_ports_text()
    if settings is not None:
        section("Settings")
        data = settings.to_dict()
        data["profiles"] = [p for p in data["profiles"] if p["name"] == settings.current_profile]
        lines.append(json.dumps(data, indent=2, ensure_ascii=False, default=str))
    section("Connection")
    lines.append(json.dumps(snapshot, indent=2, ensure_ascii=False, default=str) if snapshot else "(no snapshot)")
    if extra:
        section("Extra")
        lines.append(json.dumps(extra, indent=2, ensure_ascii=False, default=str))
    section("Recent log")
    lines += list(MEMORY_LOG.lines)[-400:]
    return "\n".join(lines) + "\n"
