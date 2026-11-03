"""Start the desktop app."""

from __future__ import annotations

import gc
import logging
import os
import sys
import traceback
import weakref

from . import APP_NAME, __version__
from .diagnostics import setup_logging
from .settings import load_settings

log = logging.getLogger(__name__)


def run_gui(sim: bool = False, debug: bool = False) -> int:
    log_path = setup_logging(debug)
    log.info("%s %s starting (Python %s, %s)", APP_NAME, __version__, sys.version.split()[0], sys.platform)
    os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")
    import shiboken6
    from PySide6.QtWidgets import QApplication, QMessageBox

    from .ui.main_window import MainWindow
    from .ui.theme import apply_theme

    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setApplicationDisplayName(APP_NAME)
    app.setApplicationVersion(__version__)
    apply_theme(app)

    settings = load_settings()
    if sim:
        settings.connection_mode = "simulator"
    window = MainWindow(settings, log_path)
    window_ref = weakref.ref(window)  # the handler must not keep the window alive past cleanup

    def excepthook(exc_type, exc, tb):
        log.critical("Unhandled error:\n%s", "".join(traceback.format_exception(exc_type, exc, tb)))
        win = window_ref()
        if win is None:
            return
        win.emergency_stop()
        QMessageBox.critical(win, "Unexpected error",
                             f"{exc_type.__name__}: {exc}\n\nThe motor was switched off. Details are in the log "
                             "(Help → Save debug report).")

    sys.excepthook = excepthook
    shutdown_worker = window.worker.shutdown
    app.aboutToQuit.connect(shutdown_worker)
    window.show()
    code = app.exec()
    # Free the window (and its charts) while Qt is still alive instead of during interpreter
    # teardown. Signal connections to lambdas form reference cycles through C++ that Python's GC
    # can't break, so deleting the C++ side is what actually releases them.
    app.aboutToQuit.disconnect(shutdown_worker)
    sys.excepthook = sys.__excepthook__
    shiboken6.delete(window)
    del window, shutdown_worker
    gc.collect()
    return code
