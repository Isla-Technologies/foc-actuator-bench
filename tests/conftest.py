from __future__ import annotations

import pytest

from actuator_bench.acquisition import AcquisitionWorker, ConnectRequest, WorkerListener
from actuator_bench.backends.simulator import ManualClock, SimActuator, SimulatedODrive, SimulatorBackend
from actuator_bench.samples import Limits
from actuator_bench.torque import TorqueModel, kt_from_kv


class RecordingListener(WorkerListener):
    def __init__(self):
        self.connection = []
        self.batches = []
        self.slow = []
        self.armed = []
        self.procedures = []
        self.messages = []
        self.snapshots = []
        self.readiness = []

    def on_connection(self, status):
        self.connection.append(status)

    def on_samples(self, batch):
        self.batches.append(batch)

    def on_slow(self, slow):
        self.slow.append(slow)

    def on_readiness(self, readiness):
        self.readiness.append(readiness)

    def on_armed(self, armed, mode):
        self.armed.append((armed, mode))

    def on_procedure(self, status):
        self.procedures.append(status)

    def on_message(self, level, text):
        self.messages.append((level, text))

    def on_snapshot(self, snapshot):
        self.snapshots.append(snapshot)

    def column(self, name):
        import numpy as np

        return np.concatenate([b[name] for b in self.batches]) if self.batches else np.empty(0)


class Rig:
    """A worker connected to a simulated ODrive, stepped manually with a fake clock."""

    def __init__(self, actuator: SimActuator, *, backend_factory=None, device_factory=None,
                 model: TorqueModel | None = None, limits: Limits | None = None, dt: float = 0.01,
                 step_s: float = 0.001, expect_connected: bool = True):
        self.clock = ManualClock()
        self.sim = SimulatedODrive(actuator, clock=self.clock, seed=3, step_s=step_s)
        self.listener = RecordingListener()
        self.worker = AcquisitionWorker(self.listener, clock=self.clock, watchdog_timeout_s=1.0)
        self.dt = dt
        self.model = model or TorqueModel(kt_motor=kt_from_kv(actuator.kv), gear_ratio=actuator.gear_ratio)
        self.worker.set_model(self.model)
        self.worker.set_limits(limits or Limits(max_current=15.0, max_motor_speed=3.0))
        if device_factory:  # another controller (a simulated SimpleFOC board) around the same physics
            self.backend = None
            self.device = device_factory(self.sim)
            self.worker.connect(ConnectRequest(device=self.device))
        else:
            self.backend = backend_factory(self.sim) if backend_factory else SimulatorBackend(self.sim)
            self.worker.connect(ConnectRequest(backend=self.backend))
        self.worker.step()
        if expect_connected:
            assert self.listener.connection[-1].state == "connected", self.listener.connection[-1].message

    def run(self, seconds: float) -> None:
        for _ in range(int(round(seconds / self.dt))):
            self.clock.advance(self.dt)
            self.worker.step()

    def run_procedure(self, procedure, timeout: float = 60.0):
        self.worker.start_procedure(procedure)
        self.worker.step()
        elapsed = 0.0
        while not procedure.status.done and elapsed < timeout:
            self.clock.advance(self.dt)
            self.worker.step()
            elapsed += self.dt
        assert procedure.status.done, "procedure did not finish"
        return procedure.status


@pytest.fixture
def rig_factory():
    rigs = []

    def make(**kwargs):
        actuator = kwargs.pop("actuator", None) or SimActuator()
        rig = Rig(actuator, **kwargs)
        rigs.append(rig)
        return rig

    yield make
    for rig in rigs:
        rig.worker.shutdown()
