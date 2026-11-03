"""Save measurements: CSV recordings with a metadata header, and per-test result folders."""

from __future__ import annotations

import csv
import json
import math
from datetime import datetime
from pathlib import Path

from .samples import SampleBatch
from .torque import TorqueStats

# (CSV column, Sample field)
COLUMNS = [
    ("time_s", "t"),
    ("torque_output_Nm", "torque_out"),
    ("torque_output_commanded_Nm", "torque_cmd_out"),
    ("torque_motor_Nm", "torque_motor"),
    ("iq_measured_A", "iq"),
    ("iq_setpoint_A", "iq_set"),
    ("output_angle_deg", "angle_out_deg"),
    ("output_speed_rpm", "speed_out_rpm"),
    ("motor_position_turns", "pos"),
    ("motor_velocity_turns_s", "vel"),
    ("bus_voltage_V", "vbus"),
    ("bus_current_A", "ibus"),
    ("electrical_power_W", "power_elec"),
    ("mechanical_power_W", "power_mech"),
    ("fet_temperature_C", "fet_temp"),
    ("motor_temperature_C", "motor_temp"),
    ("axis_state", "state"),
]


def timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def _format(value: float, field: str) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return ""
    if field == "state":
        return str(int(value))
    return f"{value:.6g}"


class CsvRecorder:
    """Writes samples as they arrive. Comment lines starting with '#' hold the settings used."""

    def __init__(self, path: Path, metadata: dict):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        for key, value in metadata.items():
            self._file.write(f"# {key}: {value}\n")
        self._writer = csv.writer(self._file)
        self._writer.writerow([name for name, _ in COLUMNS])
        self.rows = 0
        self.stats = TorqueStats()

    def write(self, batch: SampleBatch) -> None:
        if not len(batch):
            return
        columns = [(field, batch[field]) for _, field in COLUMNS]
        for i in range(len(batch)):
            self._writer.writerow([_format(float(values[i]), field) for field, values in columns])
        self.rows += len(batch)
        self.stats.add(batch["t"], batch["torque_out"])

    def close(self) -> dict:
        if not self._file.closed:
            self._file.close()
        summary = self.stats.summary()
        summary["file"] = str(self.path)
        return summary


def read_recording(path: Path) -> tuple[dict[str, str], list[str], list[list[str]]]:
    """Read a recording back: (metadata, header, rows)."""
    metadata: dict[str, str] = {}
    with open(path, newline="", encoding="utf-8") as f:
        lines = f.read().splitlines()
    data_lines = []
    for line in lines:
        if line.startswith("# "):
            key, _, value = line[2:].partition(": ")
            metadata[key] = value
        else:
            data_lines.append(line)
    rows = list(csv.reader(data_lines))
    return metadata, rows[0], rows[1:]


def _clean_for_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _clean_for_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean_for_json(v) for v in value]
    if hasattr(value, "to_dict"):
        return _clean_for_json(value.to_dict())
    return value


def save_test_result(base_dir: Path, name: str, result: dict, metadata: dict) -> Path:
    """Create ``<base_dir>/<timestamp>_<name>/`` with summary.json and data.csv."""
    folder = Path(base_dir) / f"{timestamp()}_{name}"
    folder.mkdir(parents=True, exist_ok=True)
    series = result.get("series") or {}
    summary = {"metadata": metadata, "result": {k: v for k, v in result.items() if k != "series"}}
    (folder / "summary.json").write_text(
        json.dumps(_clean_for_json(summary), indent=2, ensure_ascii=False, default=str), encoding="utf-8"
    )
    if series:
        keys = list(series)
        with open(folder / "data.csv", "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(keys)
            for row in zip(*(series[k] for k in keys)):
                writer.writerow([f"{v:.6g}" for v in row])
    return folder
