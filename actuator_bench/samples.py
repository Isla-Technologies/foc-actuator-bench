"""Measurement samples passed from the acquisition worker to the UI, recorder and procedures."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Units: t [s], iq/iq_set [A], pos [motor turns], vel [motor turns/s], torque_* [N*m],
# angle_out_deg [deg], speed_out_rpm [rpm], vbus [V], ibus [A], power_* [W], temps [degC].
SAMPLE_FIELDS = (
    "t", "iq", "iq_set", "pos", "vel",
    "torque_motor", "torque_out", "torque_cmd_out",
    "angle_out_deg", "speed_out_rpm",
    "vbus", "ibus", "power_elec", "power_mech",
    "fet_temp", "motor_temp", "state",
)


@dataclass(slots=True)
class Sample:
    t: float
    iq: float
    iq_set: float
    pos: float
    vel: float
    torque_motor: float
    torque_out: float
    torque_cmd_out: float
    angle_out_deg: float
    speed_out_rpm: float
    vbus: float
    ibus: float
    power_elec: float
    power_mech: float
    fet_temp: float
    motor_temp: float
    state: int

    def as_tuple(self) -> tuple:
        return tuple(getattr(self, name) for name in SAMPLE_FIELDS)


class SampleBatch:
    """A block of samples stored column-wise (one numpy array per field)."""

    def __init__(self, columns: dict[str, np.ndarray]):
        self.columns = columns

    @classmethod
    def from_samples(cls, samples: list[Sample]) -> SampleBatch:
        if not samples:
            return cls({name: np.empty(0) for name in SAMPLE_FIELDS})
        data = np.array([s.as_tuple() for s in samples], dtype=float)
        return cls({name: data[:, i] for i, name in enumerate(SAMPLE_FIELDS)})

    def __len__(self) -> int:
        return len(self.columns["t"])

    def __getitem__(self, name: str) -> np.ndarray:
        return self.columns[name]

    def rows(self):
        return zip(*(self.columns[name] for name in SAMPLE_FIELDS))


@dataclass
class Limits:
    """Safety limits entered in the Setup step."""

    max_current: float        # A (motor phase current, Iq)
    max_motor_speed: float    # motor turns/s
