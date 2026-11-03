"""Common interface for the ways the app can talk to an ODrive."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence

Number = float | int


class BackendError(Exception):
    """Communication with the ODrive failed (may be transient)."""


class ConnectionLost(BackendError):
    """The ODrive disappeared (unplugged, powered off, port closed)."""


class PropertyUnavailable(BackendError):
    """The firmware doesn't have this property (or can't read/write it over this connection)."""

    def __init__(self, path: str, detail: str = ""):
        super().__init__(f"{path}: {detail}" if detail else path)
        self.path = path


class Backend(ABC):
    """Read/write ODrive properties by their dotted path, e.g. ``axis0.encoder.pos_estimate``.

    A backend is used from a single thread (the acquisition worker).
    """

    kind: str = ""

    @property
    @abstractmethod
    def description(self) -> str:
        """Human-readable connection description, e.g. ``USB serial (COM5)``."""

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def read(self, path: str) -> Number:
        """Read one property. Raises PropertyUnavailable if it doesn't exist."""

    def read_many(self, paths: Sequence[str]) -> list[Number | None]:
        """Read several properties; unavailable ones come back as None."""
        values: list[Number | None] = []
        for path in paths:
            try:
                values.append(self.read(path))
            except PropertyUnavailable:
                values.append(None)
        return values

    @abstractmethod
    def write(self, path: str, value: Number | bool) -> None: ...

    @abstractmethod
    def call(self, path: str) -> None:
        """Call a device function such as ``clear_errors`` or ``axis0.watchdog_feed``."""

    def identity(self) -> dict[str, str]:
        """Optional extra identification (hardware/firmware/serial strings)."""
        return {}
