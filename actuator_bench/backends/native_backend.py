"""Talk to the ODrive through the official ``odrive`` Python package (optional dependency).

Works with odrive 0.6.x ("basic compatibility" with ODrive v3.x) and 0.5.x. Only one program can
use this USB interface at a time, so close odrivetool while the app is connected this way.
"""

from __future__ import annotations

import logging
from functools import reduce

from .base import Backend, BackendError, ConnectionLost, PropertyUnavailable

log = logging.getLogger(__name__)


def odrive_library_version() -> str | None:
    """Installed version of the official library, or None if it isn't installed."""
    try:
        from importlib.metadata import PackageNotFoundError, version
    except ImportError:  # pragma: no cover
        return None
    try:
        return version("odrive")
    except PackageNotFoundError:
        return None


def _looks_like_disconnect(exc: BaseException) -> bool:
    name = type(exc).__name__.lower()
    return "lost" in name or "disconnect" in name or isinstance(exc, (ConnectionError, BrokenPipeError))


class NativeBackend(Backend):
    kind = "native"

    def __init__(self, timeout: float = 6.0, serial_number: str | None = None):
        self.timeout = timeout
        self.serial_number = serial_number
        self._dev = None
        self._parents: dict[str, tuple[object, str]] = {}
        self._version = odrive_library_version()

    @property
    def description(self) -> str:
        return f"USB (official odrive library {self._version})" if self._version else "USB (official odrive library)"

    def connect(self) -> None:
        try:
            import odrive
        except ImportError as exc:
            raise BackendError(
                "the official 'odrive' Python package is not installed (pip install odrive)"
            ) from exc
        kwargs = {"timeout": self.timeout}
        if self.serial_number:
            kwargs["serial_number"] = self.serial_number
        try:
            dev = odrive.find_any(**kwargs)
        except TimeoutError as exc:
            raise BackendError(f"no ODrive found via the official library within {self.timeout:g} s") from exc
        except Exception as exc:  # the library raises different types per version
            raise BackendError(f"official odrive library failed ({type(exc).__name__}: {exc})") from exc
        if dev is None:
            raise BackendError("no ODrive found via the official library")
        self._dev = dev
        self._parents.clear()
        log.info("Connected via official odrive library %s", self._version)

    def close(self) -> None:
        self._dev = None
        self._parents.clear()

    def _resolve(self, path: str) -> tuple[object, str]:
        cached = self._parents.get(path)
        if cached:
            return cached
        if self._dev is None:
            raise ConnectionLost("not connected")
        *parents, name = path.split(".")
        try:
            parent = reduce(getattr, parents, self._dev)
        except AttributeError as exc:
            raise PropertyUnavailable(path, "not in this firmware") from exc
        self._parents[path] = (parent, name)
        return parent, name

    def read(self, path: str):
        parent, name = self._resolve(path)
        try:
            value = getattr(parent, name)
        except AttributeError as exc:
            raise PropertyUnavailable(path, "not in this firmware") from exc
        except Exception as exc:
            raise self._translate(exc) from exc
        return _to_number(value, path)

    def write(self, path: str, value) -> None:
        parent, name = self._resolve(path)
        try:
            setattr(parent, name, value)
        except AttributeError as exc:
            raise PropertyUnavailable(path, "not writable in this firmware") from exc
        except Exception as exc:
            raise self._translate(exc) from exc

    def call(self, path: str) -> None:
        parent, name = self._resolve(path)
        try:
            func = getattr(parent, name)
        except AttributeError as exc:
            raise PropertyUnavailable(path, "function not in this firmware") from exc
        try:
            func()
        except Exception as exc:
            raise self._translate(exc) from exc

    def identity(self) -> dict[str, str]:
        if self._dev is None:
            return {}
        try:
            return {"serial": f"{int(self._dev.serial_number):012X}"}
        except Exception:
            return {}

    @staticmethod
    def _translate(exc: Exception) -> BackendError:
        if _looks_like_disconnect(exc):
            return ConnectionLost(f"ODrive disconnected ({type(exc).__name__})")
        return BackendError(f"{type(exc).__name__}: {exc}")


def _to_number(value, path: str):
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return int(value)  # also turns IntEnum into a plain int
    if isinstance(value, float):
        return float(value)
    try:
        return int(value)  # enums from the library
    except (TypeError, ValueError):
        pass
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise PropertyUnavailable(path, f"unexpected value type {type(value).__name__}") from exc
