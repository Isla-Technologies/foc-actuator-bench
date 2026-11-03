"""Forward worker events (worker thread) to the UI as Qt signals (delivered on the GUI thread)."""

from __future__ import annotations

from PySide6.QtCore import QObject, Signal

from ..acquisition import WorkerListener


def _safe(emit):
    """Ignore events that arrive after the UI was closed (the Qt object is already deleted)."""
    try:
        emit()
    except RuntimeError:
        pass


class WorkerBridge(QObject, WorkerListener):
    connection = Signal(object)      # ConnectionStatus
    samples = Signal(object)         # SampleBatch
    slow = Signal(object)            # SlowReading
    readiness = Signal(object)       # Readiness
    armed = Signal(bool, object)     # armed, ControlMode | None
    procedure = Signal(object)       # ProcedureStatus
    message = Signal(str, str)       # level, text
    snapshot = Signal(object)        # dict

    def on_connection(self, status) -> None:
        _safe(lambda: self.connection.emit(status))

    def on_samples(self, batch) -> None:
        _safe(lambda: self.samples.emit(batch))

    def on_slow(self, slow) -> None:
        _safe(lambda: self.slow.emit(slow))

    def on_readiness(self, readiness) -> None:
        _safe(lambda: self.readiness.emit(readiness))

    def on_armed(self, armed, mode) -> None:
        _safe(lambda: self.armed.emit(armed, mode))

    def on_procedure(self, status) -> None:
        _safe(lambda: self.procedure.emit(status))

    def on_message(self, level, text) -> None:
        _safe(lambda: self.message.emit(level, text))

    def on_snapshot(self, snapshot) -> None:
        _safe(lambda: self.snapshot.emit(snapshot))
