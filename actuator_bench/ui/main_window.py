"""Main window: the three steps (Setup → Connect → Measure), STOP, recording and reports."""

from __future__ import annotations

import logging
import sys
import weakref
from datetime import date
from pathlib import Path

from PySide6.QtCore import QEvent, QObject, Qt, QUrl
from PySide6.QtGui import QAction, QDesktopServices, QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QAbstractSpinBox,
    QApplication,
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from .. import APP_NAME, __version__
from ..acquisition import AcquisitionWorker, ConnectionStatus
from ..controllers import CONTROLLER_NAMES, CONTROLLERS
from ..diagnostics import build_report, log_dir
from ..model_export import record_result
from ..procedures import ProcedureStatus
from ..recorder import CsvRecorder, save_test_result, timestamp
from ..settings import ActuatorProfile, AppSettings, save_settings
from ..torque import TORQUE_UNITS
from . import theme
from .bridge import WorkerBridge
from .connect_page import ConnectPage
from .measure_page import MeasurePage
from .setup_page import SetupPage
from .widgets import StepIndicator, label

log = logging.getLogger(__name__)

STEP_SETUP, STEP_CONNECT, STEP_MEASURE = range(3)


class _SpaceStop(QObject):
    """Space bar = STOP, unless the user is typing in a text or number field."""

    def __init__(self, window: MainWindow):
        super().__init__(window)
        self._window = weakref.ref(window)  # the application must not keep the window alive

    def eventFilter(self, obj, event) -> bool:
        window = self._window()
        if window is None:
            return False
        if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Space and not event.isAutoRepeat():
            focus = QApplication.focusWidget()
            typing = isinstance(focus, (QLineEdit, QAbstractSpinBox, QPlainTextEdit)) or (
                isinstance(focus, QComboBox) and focus.isEditable())
            if not typing and QApplication.activeModalWidget() is None and window.isActiveWindow() \
                    and window.connected:
                window.emergency_stop()
                return True
        return False


class MainWindow(QMainWindow):
    def __init__(self, settings: AppSettings, log_path: Path | None = None):
        super().__init__()
        self.settings = settings
        self.log_path = log_path
        self.profile: ActuatorProfile = settings.profile()
        self.connected = False
        self.connection_status: ConnectionStatus | None = None
        self.kt_device: float | None = None   # the controller's own torque constant, if it has one
        self.recorder: CsvRecorder | None = None
        self.setup_done = False
        self.procedure_running = False
        self._closing = False
        self._report_pending = False

        self.bridge = WorkerBridge()
        self.worker = AcquisitionWorker(self.bridge, sample_rate_hz=settings.sample_rate_hz,
                                        watchdog_timeout_s=settings.watchdog_timeout_s if settings.watchdog_enabled else None)
        self.worker.start()

        self.setWindowTitle(APP_NAME)
        self.resize(1320, 840)
        self.setMinimumSize(1024, 660)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._header())

        self.stack = QStackedWidget()
        self.setup_page = SetupPage(settings)
        self.connect_page = ConnectPage(settings)
        self.measure_page = MeasurePage(self.worker, settings)
        for page in (self.setup_page, self.connect_page, self.measure_page):
            self.stack.addWidget(page)
        root.addWidget(self.stack, 1)
        self.statusBar().showMessage("Enter your actuator's values to begin.")

        self._wire()
        self._menus()
        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.emergency_stop, context=Qt.ApplicationShortcut)
        self._space_filter = _SpaceStop(self)
        QApplication.instance().installEventFilter(self._space_filter)
        self.go_to(STEP_SETUP)

    # --- layout ----------------------------------------------------------------------------
    def _header(self) -> QWidget:
        header = QWidget()
        header.setObjectName("header")
        header.setAttribute(Qt.WA_StyledBackground, True)
        header.setStyleSheet(f"QWidget#header {{ background: {theme.SURFACE}; border-bottom: 1px solid {theme.BORDER}; }}")
        row = QHBoxLayout(header)
        row.setContentsMargins(18, 8, 18, 8)
        title = label(APP_NAME, size=16, weight=QFont.DemiBold)
        row.addWidget(title)
        row.addSpacing(24)
        self.steps = StepIndicator(["Setup", "Connect", "Measure"])
        self.steps.clicked.connect(self._step_clicked)
        row.addWidget(self.steps, 1)
        self.stop_button = QPushButton("STOP   (Esc)")
        self.stop_button.setObjectName("stop")
        self.stop_button.setMinimumHeight(40)
        self.stop_button.setMinimumWidth(170)
        self.stop_button.setToolTip("Switch the motor off immediately (Esc or Space)")
        self.stop_button.clicked.connect(self.emergency_stop)
        row.addWidget(self.stop_button)
        return header

    def _menus(self) -> None:
        file_menu = self.menuBar().addMenu("&File")
        open_data = QAction("Open data folder", self)
        open_data.triggered.connect(self.open_data_folder)
        file_menu.addAction(open_data)
        choose_data = QAction("Change data folder…", self)
        choose_data.triggered.connect(self.choose_data_folder)
        file_menu.addAction(choose_data)
        file_menu.addSeparator()
        quit_action = QAction("Quit", self)
        quit_action.setShortcut(QKeySequence.Quit)
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        help_menu = self.menuBar().addMenu("&Help")
        report = QAction("Save debug report…", self)
        report.triggered.connect(self.save_debug_report)
        help_menu.addAction(report)
        logs = QAction("Open log folder", self)
        logs.triggered.connect(lambda: self._open_path(log_dir()))
        help_menu.addAction(logs)
        help_menu.addSeparator()
        about = QAction(f"About {APP_NAME}", self)
        about.triggered.connect(self.about)
        help_menu.addAction(about)

    def _wire(self) -> None:
        b = self.bridge
        b.connection.connect(self._on_connection)
        b.samples.connect(self._on_samples)
        b.slow.connect(self.measure_page.set_slow)
        b.readiness.connect(self._on_readiness)
        b.armed.connect(self._on_armed)
        b.procedure.connect(self._on_procedure)
        b.message.connect(self._on_message)
        b.snapshot.connect(self._on_snapshot)

        self.setup_page.completed.connect(self._setup_completed)
        c = self.connect_page
        c.back.connect(lambda: self.go_to(STEP_SETUP))
        c.proceed.connect(lambda: self.go_to(STEP_MEASURE))
        c.connect_requested.connect(self.worker.connect)
        c.disconnect_requested.connect(self.worker.disconnect)
        c.clear_errors_requested.connect(self.worker.clear_errors)
        c.calibrate_requested.connect(self.worker.start_motor_calibration)
        c.apply_kt_requested.connect(self.worker.apply_torque_constant)
        c.refresh_requested.connect(self.worker.refresh_readiness)

        m = self.measure_page
        m.back.connect(self._measure_back)
        m.record_toggled.connect(self._record_toggled)
        m.open_data_folder.connect(self.open_data_folder)
        m.clear_errors_requested.connect(self.worker.clear_errors)
        m.calibration.calibration_saved.connect(self._calibration_saved)
        m.model.result_measured.connect(self._record_model_result)
        m.model.profile_values_changed.connect(self._update_profile)

    # --- navigation ------------------------------------------------------------------------
    def go_to(self, step: int) -> None:
        if step == STEP_CONNECT and not self.setup_done:
            step = STEP_SETUP
        if step == STEP_MEASURE and not self.connected:
            step = STEP_CONNECT if self.setup_done else STEP_SETUP
        self.stack.setCurrentIndex(step)
        self.steps.set_state(step, enabled=[True, self.setup_done, self.connected],
                             done=[self.setup_done, self.connected, False])
        messages = {STEP_SETUP: "Enter your actuator's values, then press Next.",
                    STEP_CONNECT: "Connect to the controller or the simulator.",
                    STEP_MEASURE: "Measuring. Press STOP or Esc at any time to switch the motor off."}
        self.statusBar().showMessage(messages[step])

    def _step_clicked(self, step: int) -> None:
        if self.stack.currentIndex() == STEP_SETUP and step > STEP_SETUP:
            return  # leave the setup page through its Next button so values are validated and applied
        if step == STEP_SETUP:
            self._measure_back()
        else:
            self.go_to(step)

    def _setup_completed(self, profile: ActuatorProfile) -> None:
        self.profile = profile
        self.setup_done = True
        self._apply_profile()
        save_settings(self.settings)
        self.go_to(STEP_MEASURE if self.connected else STEP_CONNECT)

    def _apply_profile(self) -> None:
        profile, s = self.profile, self.settings
        self.worker.set_model(profile.torque_model())
        self.worker.set_limits(profile.limits())
        self.worker.set_sample_rate(s.sample_rate_hz)
        self.worker.set_watchdog(s.watchdog_timeout_s if s.watchdog_enabled else None)
        self.connect_page.set_profile(profile)
        self.measure_page.settings = s
        self.measure_page.set_profile(profile)

    def _measure_back(self) -> None:
        if self.procedure_running:
            QMessageBox.information(self, "Test running", "Stop the running test first.")
            return
        if self.measure_page.armed:
            answer = QMessageBox.question(self, "Switch the motor off?",
                                          "The motor is on. Switch it off and go back to Setup?")
            if answer != QMessageBox.Yes:
                return
            self.worker.disarm()
        self.go_to(STEP_SETUP)

    # --- worker events ---------------------------------------------------------------------
    def _on_connection(self, status: ConnectionStatus) -> None:
        was_connected = self.connected
        self.connection_status = status
        self.connected = status.state == "connected"
        self.connect_page.set_status(status)
        if self.connected:
            self.kt_device = status.readiness.kt_device if status.readiness else None
            sim = status.info.backend_kind == "simulator" if status.info else False
            self.measure_page.set_connection(True, status.info, sim, self.settings.sim_scenario)
            self.measure_page.set_profile(self.profile)
            save_settings(self.settings)
        elif status.state in ("disconnected", "failed"):
            self.measure_page.set_connection(False)
            self.procedure_running = False
            self.measure_page.set_busy(False)
            self._stop_recording()
            if was_connected and "lost" in status.message.lower() and not self._closing:
                QMessageBox.warning(self, "Connection lost", status.message)
            if was_connected and self.stack.currentIndex() == STEP_MEASURE:
                self.go_to(STEP_CONNECT)
        self.steps.set_state(self.stack.currentIndex(), enabled=[True, self.setup_done, self.connected],
                             done=[self.setup_done, self.connected, False])

    def _on_readiness(self, readiness) -> None:
        self.kt_device = readiness.kt_device
        self.connect_page.set_readiness(readiness)
        self.measure_page.set_profile(self.profile)

    def _on_samples(self, batch) -> None:
        self.measure_page.add_batch(batch)
        if self.recorder is not None:
            try:
                self.recorder.write(batch)
            except OSError as exc:
                self._stop_recording()
                QMessageBox.critical(self, "Recording stopped", f"Could not write the recording: {exc}")
                return
            self.measure_page.set_record_text(
                f"Recording {self.recorder.path.name} · {self.recorder.rows} samples · {self.recorder.stats.duration:.1f} s")

    def _on_armed(self, armed: bool, mode) -> None:
        self.measure_page.set_armed(armed, mode)

    def _on_message(self, level: str, text: str) -> None:
        log.log({"error": logging.ERROR, "warning": logging.WARNING}.get(level, logging.INFO), "%s", text)
        self.measure_page.show_message(level, text)
        self.statusBar().showMessage(text, 10000)
        if level == "error" and self.stack.currentIndex() == STEP_CONNECT:
            QMessageBox.warning(self, self._controller_title(), text)

    def _controller_title(self) -> str:
        info = self.connection_status.info if self.connection_status else None
        return info.controller if info else CONTROLLER_NAMES.get(self.profile.controller, "Controller")

    def _on_procedure(self, status: ProcedureStatus) -> None:
        m = self.measure_page
        panels = {p.procedure_name: p for p in (m.stall, m.hold, m.calibration, *m.model.test_panels)}
        panel = panels.get(status.name)
        if panel is None:
            return
        self.procedure_running = not status.done
        self.measure_page.set_busy(not status.done)
        panel.show_status(status)
        if status.done and status.result and status.name != "Calibration sweep":
            self._save_result(status, panel)
            test = getattr(panel, "test_key", "")
            if status.success and test:
                self._record_model_result(test, status.result)

    def _save_result(self, status: ProcedureStatus, panel) -> None:
        slug = status.name.lower().replace(" ", "_")
        try:
            folder = save_test_result(self.settings.data_path, slug, status.result, self._metadata())
            self.measure_page.plots.save_image(str(folder / "chart.png"))
            if hasattr(panel, "chart"):
                panel.chart.save_image(str(folder / "result.png"))
        except OSError as exc:
            panel.message.setText(panel.message.text() + f"\nCould not save the results: {exc}")
            return
        panel.show_saved(str(folder))

    # --- recording -------------------------------------------------------------------------
    def _metadata(self) -> dict:
        p = self.profile
        model = p.torque_model()
        info = self.connection_status.info if self.connection_status else None
        where = f"axis {p.axis} (M{p.axis})" if p.controller == "odrive" else f"Commander letter {p.motor_letter}"
        return {
            "app": f"{APP_NAME} {__version__}",
            "created": timestamp(),
            "profile": p.name,
            "torque_units_in_file": "N·m (display unit: " + p.unit + ")",
            "kt_motor_Nm_per_A": round(p.kt_motor, 6),
            "kt_source": "KV " + str(p.kv) if p.kt_source == "kv" else "entered Kt",
            "gear_ratio": p.gear_ratio,
            "efficiency_pct": p.efficiency_pct,
            "calibrated_k_out_Nm_per_A": p.calibration_k_out if model.is_calibrated else "not used",
            "output_torque_per_amp_Nm_per_A": round(model.k_out, 6),
            "controller": CONTROLLERS.get(p.controller, p.controller),
            "controller_torque_constant": self.kt_device,
            "max_current_A": p.max_current_a,
            "max_output_speed_rpm": p.max_speed_rpm,
            "motor_on": where,
            "device": f"{info.board} fw {info.firmware} serial {info.serial}" if info else "not connected",
            "connection": info.connection if info else "",
        }

    def _record_toggled(self, on: bool) -> None:
        if on:
            path = self.settings.data_path / f"{timestamp()}_recording.csv"
            try:
                self.recorder = CsvRecorder(path, self._metadata())
            except OSError as exc:
                QMessageBox.critical(self, "Can't record", f"Could not create {path}: {exc}")
                self.measure_page.record_button.setChecked(False)
                return
            self.measure_page.set_record_text(f"Recording to {path}")
        else:
            self._stop_recording()

    def _stop_recording(self) -> None:
        if self.recorder is None:
            return
        recorder, self.recorder = self.recorder, None
        summary = recorder.close()
        f = TORQUE_UNITS[self.profile.unit]
        u = self.profile.unit
        self.measure_page.set_record_text(
            f"Saved {recorder.path.name}: {recorder.rows} samples over {summary['duration_s']:.1f} s · "
            f"peak {summary['peak_abs_Nm'] * f:.4g} {u} · mean {summary['mean_Nm'] * f:.4g} {u} · "
            f"RMS {summary['rms_Nm'] * f:.4g} {u}")
        if self.measure_page.record_button.isChecked():
            self.measure_page.record_button.blockSignals(True)
            self.measure_page.record_button.setChecked(False)
            self.measure_page.record_button.setText("●  Record")
            self.measure_page.record_button.blockSignals(False)

    # --- calibration -----------------------------------------------------------------------
    def _calibration_saved(self, fit) -> None:
        details = fit.to_dict()
        self.setup_page.set_calibration(fit.k_out_nm_per_a, date.today().isoformat(), details)
        self.profile = self.setup_page.current_profile()
        self.settings.upsert_profile(self.profile)
        save_settings(self.settings)
        self._apply_profile()
        try:
            save_test_result(self.settings.data_path, "calibration", {"test": "calibration", **details}, self._metadata())
        except OSError as exc:
            log.warning("Could not save the calibration summary: %s", exc)

    # --- simulation model ------------------------------------------------------------------
    def _record_model_result(self, test: str, result: dict) -> None:
        record_result(self.profile, test, result)
        self._update_profile({"model_results": self.profile.model_results})

    def _update_profile(self, values: dict) -> None:
        """Keep values edited outside Setup (the simulation model's) on the profile, and save them."""
        self.setup_page.update_hidden(**values)
        self.profile = self.setup_page.current_profile()
        self.settings.upsert_profile(self.profile)
        save_settings(self.settings)
        self.measure_page.model.set_profile(self.profile)

    # --- STOP, reports, misc ---------------------------------------------------------------
    def emergency_stop(self) -> None:
        self.worker.estop()
        self.statusBar().showMessage("STOP: motor switched off.", 5000)

    def open_data_folder(self) -> None:
        self.settings.data_path.mkdir(parents=True, exist_ok=True)
        self._open_path(self.settings.data_path)

    def choose_data_folder(self) -> None:
        folder = QFileDialog.getExistingDirectory(self, "Choose the data folder", str(self.settings.data_path))
        if folder:
            self.settings.data_dir = folder
            save_settings(self.settings)

    @staticmethod
    def _open_path(path: Path) -> None:
        path.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def save_debug_report(self) -> None:
        self._report_pending = True
        self.worker.request_snapshot()

    def _on_snapshot(self, snapshot: dict) -> None:
        if not self._report_pending:
            return
        self._report_pending = False
        default = Path.home() / "Desktop" / f"odrive-actuator-bench-report-{timestamp()}.txt"
        if not default.parent.exists():
            default = Path.home() / default.name
        path, _ = QFileDialog.getSaveFileName(self, "Save debug report", str(default), "Text file (*.txt)")
        if not path:
            return
        extra = {"log_file": str(self.log_path) if self.log_path else None,
                 "recent_messages": self.measure_page.log_view.toPlainText().splitlines()[-100:]}
        Path(path).write_text(build_report(snapshot, self.settings, extra), encoding="utf-8")
        QMessageBox.information(self, "Debug report saved",
                                f"Saved {path}.\n\nSend this file to whoever helps you with the setup. "
                                "It contains the app/OS versions, controller settings and recent log messages.")

    def about(self) -> None:
        QMessageBox.about(self, f"About {APP_NAME}",
                          f"<b>{APP_NAME} {__version__}</b><br>Measures the torque of an actuator from the motor "
                          "current, driven by an ODrive V3.6 (firmware 0.5.x) or a SimpleFOC board.<br><br>"
                          "Torque = Kt × current × gear ratio × efficiency, or the weight calibration.<br>"
                          f"Python {sys.version.split()[0]} · MIT licence")

    def closeEvent(self, event) -> None:
        self._closing = True
        QApplication.instance().removeEventFilter(self._space_filter)
        self._stop_recording()
        self.worker.shutdown()
        try:
            save_settings(self.settings)
        except OSError as exc:
            log.warning("Could not save settings: %s", exc)
        event.accept()
