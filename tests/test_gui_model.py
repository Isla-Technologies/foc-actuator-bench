"""Drive the Simulation model tab (offscreen): run a test, check the gear ratio, export the model."""

import json
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")

from test_gui_smoke import pump, window  # noqa: E402,F401  (the fixture)


def connect(win, app, scenario):
    setup = win.setup_page
    setup.kv.setValue(100)
    setup.gear.setValue(9)
    setup.max_current.setValue(10)
    setup.next_button.click()
    page = win.connect_page
    page.mode.setCurrentIndex(page.mode.findData("simulator"))
    page.scenario.setCurrentIndex(page.scenario.findData(scenario))
    page.connect_button.click()
    assert pump(app, 10, lambda: win.connected), page.status_text.text()
    page.proceed_button.click()
    assert pump(app, 5, lambda: len(win.measure_page.buffer) > 20)


def test_backlash_result_reaches_the_model_and_export(window, tmp_path, monkeypatch):
    holder, app = window
    win = holder[0]
    connect(win, app, "locked")
    model = win.measure_page.model
    panel = model.backlash
    assert "Simulator" in panel.sim_note.text() and "choose" not in panel.sim_note.text()
    assert "choose" in model.friction.sim_note.text()  # friction needs a free output
    panel.torque.setValue(2.0)
    panel.cycle.setValue(2.0)
    panel.start_button.click()
    assert pump(app, 2, lambda: win.procedure_running)
    assert pump(app, 20, lambda: not win.procedure_running), panel.message.text()
    assert panel.message.text().startswith("✔"), panel.message.text()
    stored = win.profile.model_results["backlash"]
    assert stored["backlash_deg"] == pytest.approx(0.2, abs=0.08)
    assert stored["date"]
    assert win.settings.profile(win.profile.name).model_results["backlash"] == stored
    assert list((tmp_path / "data").glob("*_backlash_test/result.png"))

    # The overview shows it as measured, and the robot's values are kept on the profile.
    overview = model.overview
    rows = {overview.table.item(r, 0).text(): overview.table.item(r, 2).text() for r in range(overview.table.rowCount())}
    assert rows["Backlash  (backlash)"].startswith("✔")
    assert rows["Reflected inertia (armature)  (armature)"].startswith("○")
    overview.battery.setValue(22.2)
    assert win.profile.battery_voltage == pytest.approx(22.2)
    assert win.setup_page.current_profile().model_results["backlash"] == stored

    target = tmp_path / "model.json"
    monkeypatch.setattr(QtWidgets.QFileDialog, "getSaveFileName", lambda *a, **k: (str(target), ""))
    overview.export_button.click()
    exported = json.loads(target.read_text(encoding="utf-8"))
    assert exported["actuator_type"]["measured"] == ["backlash", "stiffness"]
    assert exported["conditions"]["battery_voltage_V"] == pytest.approx(22.2)


def test_gear_ratio_check_with_a_simulated_hand(window):
    holder, app = window
    win = holder[0]
    connect(win, app, "free")
    gear = win.measure_page.model.gear
    assert not gear.hand_button.isHidden()
    gear.start_button.click()
    gear.turns.setValue(2)
    gear.hand_button.click()
    gear.hand_button.click()
    assert pump(app, 2, lambda: "18.0" in gear.live.text())
    gear.done_button.click()
    assert win.profile.model_results["gear_ratio"]["ratio"] == pytest.approx(9.0, rel=1e-3)
    assert gear.r_ratio.value.text().startswith("9")
