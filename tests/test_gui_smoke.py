"""Drive the real app (offscreen) through Setup -> Connect (simulator) -> Measure -> stall test."""

import gc
import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")
import shiboken6  # noqa: E402


@pytest.fixture
def window(tmp_path, monkeypatch):
    import actuator_bench.settings as settings_module

    monkeypatch.setattr(settings_module, "config_dir", lambda: tmp_path / "config")
    from actuator_bench.settings import AppSettings
    from actuator_bench.ui.main_window import MainWindow
    from actuator_bench.ui.theme import apply_theme

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    apply_theme(app)
    holder = [MainWindow(AppSettings(data_dir=str(tmp_path / "data"), sim_scenario="wall"))]
    holder[0].show()
    yield holder, app
    # Free the window while Qt is alive rather than during interpreter teardown. pytest caches what
    # a fixture yields, so the window lives in a list we can empty; deleting the C++ side breaks the
    # lambda connection cycles the GC can't see through.
    win = holder.pop()
    win.close()
    app.processEvents()
    shiboken6.delete(win)
    del win
    gc.collect()


def pump(app, seconds, until=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        if until and until():
            return True
        time.sleep(0.005)
    return bool(until and until())


def test_full_flow_with_simulator(window, tmp_path):
    holder, app = window
    win = holder[0]
    setup = win.setup_page
    assert win.stack.currentIndex() == 0

    # Step 1: enter the actuator's values (the fields only accept valid ranges).
    setup.kv.setValue(100)
    setup.gear.setValue(9)
    setup.max_current.setValue(10)
    setup.next_button.click()
    assert win.stack.currentIndex() == 1

    # Step 2: connect to the simulator.
    page = win.connect_page
    page.mode.setCurrentIndex(page.mode.findData("simulator"))
    page.scenario.setCurrentIndex(page.scenario.findData("wall"))
    page.connect_button.click()
    assert pump(app, 10, lambda: win.connected), page.status_text.text()
    assert page.proceed_button.isEnabled()
    assert "0.5.6" in page.info_labels["Firmware"].text()

    # Step 3: live data arrives.
    page.proceed_button.click()
    assert win.stack.currentIndex() == 2
    measure = win.measure_page
    assert pump(app, 5, lambda: len(measure.buffer) > 50)

    # Record while running a short stall test.
    measure.record_button.setChecked(True)
    stall = measure.stall
    stall.target.setValue(3.0)
    stall.ramp.setValue(0.6)
    stall.hold.setValue(0.4)
    stall.start_button.click()
    assert pump(app, 2, lambda: win.procedure_running)
    assert pump(app, 15, lambda: not win.procedure_running), stall.message.text()
    assert "complete" in stall.message.text(), stall.message.text()
    peak = float(stall.r_peak.value.text().split()[0])
    assert peak == pytest.approx(3.0, rel=0.1)
    measure.record_button.setChecked(False)
    recordings = list((tmp_path / "data").glob("*_recording.csv"))
    assert recordings and recordings[0].stat().st_size > 1000
    assert list((tmp_path / "data").glob("*_stall_test/summary.json"))

    # STOP is always available and leaves the motor off.
    win.emergency_stop()
    assert pump(app, 2, lambda: not win.worker.armed)


def test_simplefoc_flow_with_simulator(window):
    holder, app = window
    win = holder[0]
    setup = win.setup_page

    # Step 1: a BE4108-style motor on a SimpleFOC board.
    setup.controller.setCurrentIndex(setup.controller.findData("simplefoc"))
    assert setup.letter.isVisibleTo(setup) and not setup.axis.isVisibleTo(setup)
    setup.kv.setValue(380)
    setup.pole_pairs.setValue(11)
    setup.gear.setValue(1)
    setup.max_current.setValue(5)
    setup.max_speed.setValue(300)
    setup.next_button.click()
    assert win.stack.currentIndex() == 1
    assert win.profile.controller == "simplefoc" and win.profile.pole_pairs == 11

    # Step 2: the Connect page offers the SimpleFOC connections.
    page = win.connect_page
    assert "SimpleFOC" in page.title.text()
    modes = [page.mode.itemData(i) for i in range(page.mode.count())]
    assert modes == ["serial", "simulator"]
    page.mode.setCurrentIndex(page.mode.findData("simulator"))
    page.scenario.setCurrentIndex(page.scenario.findData("wall"))
    page.connect_button.click()
    assert pump(app, 10, lambda: win.connected), page.status_text.text()
    assert page.info_labels["Board"].text().startswith("SimpleFOC")
    assert not page.kt_button.isVisibleTo(page)  # SimpleFOC has no torque constant to write

    # Step 3: a stall test.
    page.proceed_button.click()
    measure = win.measure_page
    assert pump(app, 5, lambda: len(measure.buffer) > 50)
    stall = measure.stall
    stall.target.setValue(0.08)
    stall.ramp.setValue(0.6)
    stall.hold.setValue(0.4)
    stall.start_button.click()
    assert pump(app, 2, lambda: win.procedure_running)
    assert pump(app, 15, lambda: not win.procedure_running), stall.message.text()
    assert "complete" in stall.message.text(), stall.message.text()
    win.emergency_stop()
    assert pump(app, 2, lambda: not win.worker.armed)
