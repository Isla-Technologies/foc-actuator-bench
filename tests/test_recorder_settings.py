import json
import math

import numpy as np
import pytest

from actuator_bench.recorder import COLUMNS, CsvRecorder, read_recording, save_test_result
from actuator_bench.samples import SAMPLE_FIELDS, SampleBatch
from actuator_bench.settings import ActuatorProfile, AppSettings, load_settings, save_settings


def make_batch(n=5):
    columns = {name: np.arange(n, dtype=float) for name in SAMPLE_FIELDS}
    columns["torque_out"] = np.array([0.0, 1.0, -2.0, 3.0, 0.5])[:n]
    columns["vbus"] = np.full(n, np.nan)
    return SampleBatch(columns)


def test_csv_recording_round_trip(tmp_path):
    rec = CsvRecorder(tmp_path / "run.csv", {"profile": "Test", "gear_ratio": 9})
    rec.write(make_batch())
    summary = rec.close()
    assert summary["peak_abs_Nm"] == 3.0
    metadata, header, rows = read_recording(tmp_path / "run.csv")
    assert metadata == {"profile": "Test", "gear_ratio": "9"}
    assert header == [name for name, _ in COLUMNS]
    assert len(rows) == 5
    assert rows[3][header.index("torque_output_Nm")] == "3"
    assert rows[0][header.index("bus_voltage_V")] == ""  # NaN -> empty cell


def test_save_test_result(tmp_path):
    result = {"test": "stall", "peak_torque_Nm": 2.5, "fet_temp_end_C": math.nan,
              "series": {"t": [0.0, 0.01], "torque_meas": [0.1, 0.2]}}
    folder = save_test_result(tmp_path, "stall", result, {"profile": "Test"})
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    assert summary["result"]["peak_torque_Nm"] == 2.5
    assert summary["result"]["fet_temp_end_C"] is None
    assert (folder / "data.csv").read_text(encoding="utf-8").splitlines()[0] == "t,torque_meas"


def test_profile_validation_and_model():
    profile = ActuatorProfile(kv=100, gear_ratio=9, efficiency_pct=90, max_current_a=10, max_speed_rpm=60)
    assert profile.validate() == {}
    model = profile.torque_model()
    assert model.k_out == pytest.approx(0.0827 * 9 * 0.9)
    assert profile.limits().max_motor_speed == pytest.approx(9.0)
    bad = ActuatorProfile(kv=0, gear_ratio=-1, efficiency_pct=120, max_current_a=0, name=" ")
    assert set(bad.validate()) >= {"kv", "gear_ratio", "efficiency_pct", "max_current_a", "name"}
    calibrated = ActuatorProfile(calibration_k_out=0.66)
    assert calibrated.torque_model().k_out == 0.66
    calibrated.use_calibration = False
    assert calibrated.torque_model().k_out == pytest.approx(0.0827)


def test_settings_save_and_load(tmp_path):
    settings = AppSettings()
    settings.upsert_profile(ActuatorProfile(name="Leg", kv=150, gear_ratio=6))
    settings.connection_mode = "serial"
    path = tmp_path / "settings.json"
    save_settings(settings, path)
    loaded = load_settings(path)
    assert loaded.current_profile == "Leg"
    assert loaded.profile().gear_ratio == 6
    assert loaded.connection_mode == "serial"
    assert len(loaded.profiles) == 2


def test_corrupt_settings_fall_back_to_defaults(tmp_path):
    path = tmp_path / "settings.json"
    path.write_text("{not json", encoding="utf-8")
    loaded = load_settings(path)
    assert loaded.profile().name == "My actuator"
    assert (tmp_path / "settings.broken.json").exists()


def test_settings_from_before_the_rename_are_taken_over(tmp_path, monkeypatch):
    import actuator_bench.settings as settings_module

    monkeypatch.setattr(settings_module, "config_dir", lambda: tmp_path / "config" / "ODrive Actuator Bench")
    monkeypatch.setattr(settings_module.Path, "home", lambda: tmp_path)
    old = AppSettings()
    old.upsert_profile(ActuatorProfile(name="Knee", gear_ratio=36))
    save_settings(old, tmp_path / "config" / "ODrive Torque Meter" / "settings.json")
    (tmp_path / "Documents" / "ODrive Torque Meter").mkdir(parents=True)
    loaded = load_settings()
    assert loaded.current_profile == "Knee"
    assert loaded.data_path == tmp_path / "Documents" / "ODrive Torque Meter"
