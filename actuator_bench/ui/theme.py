"""Colors, fonts and the stylesheet. One explicit light theme so the app looks the same on
macOS and Windows (and stays legible when the OS is in dark mode)."""

from __future__ import annotations

from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import QApplication

# Chart chrome & ink (reference palette, light mode)
SURFACE = "#fcfcfb"        # cards and chart surface
PAGE = "#f4f4f1"           # window background (page plane)
INK = "#0b0b0b"            # primary text
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"      # axis text, hints
GRID = "#e1e0d9"           # hairline gridlines
AXIS = "#c3c2b7"           # baselines / axes
BORDER = "#e3e2dc"         # hairline card border
ACCENT = "#2a78d6"         # series slot 1 (measured values), primary buttons
ACCENT_DARK = "#256abf"
CONTEXT_SERIES = "#898781"  # commanded / setpoint lines (emphasis form: context in gray)

# Status palette: always paired with an icon + text label
GOOD = "#0ca30c"
WARNING = "#fab219"
SERIOUS = "#ec835a"
CRITICAL = "#d03b3b"
SUCCESS_TEXT = "#006300"


def apply_theme(app: QApplication) -> None:
    app.setStyle("Fusion")
    palette = QPalette()
    roles = {
        QPalette.Window: PAGE,
        QPalette.WindowText: INK,
        QPalette.Base: "#ffffff",
        QPalette.AlternateBase: SURFACE,
        QPalette.Text: INK,
        QPalette.Button: SURFACE,
        QPalette.ButtonText: INK,
        QPalette.ToolTipBase: "#ffffff",
        QPalette.ToolTipText: INK,
        QPalette.PlaceholderText: INK_MUTED,
        QPalette.Highlight: ACCENT,
        QPalette.HighlightedText: "#ffffff",
        QPalette.Link: ACCENT,
        QPalette.BrightText: "#ffffff",
    }
    for role, color in roles.items():
        palette.setColor(role, QColor(color))
    for role in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText):
        palette.setColor(QPalette.Disabled, role, QColor("#b5b3ad"))
    app.setPalette(palette)
    app.setStyleSheet(STYLESHEET)


def font(pixel_size: int, weight: QFont.Weight = QFont.Normal) -> QFont:
    f = QFont()
    f.setPixelSize(pixel_size)
    f.setWeight(weight)
    return f


STYLESHEET = f"""
QWidget {{ color: {INK}; }}
QFrame#card {{
    background: {SURFACE};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QLabel#muted {{ color: {INK_MUTED}; }}
QLabel#secondary {{ color: {INK_SECONDARY}; }}
QLabel#sectionTitle {{ color: {INK}; font-weight: 600; }}
QLabel#pageTitle {{ color: {INK}; font-weight: 600; }}
QLabel#error {{ color: {CRITICAL}; }}
QLabel#fieldError {{ color: {CRITICAL}; }}
QPushButton {{
    background: #ffffff;
    border: 1px solid #cfcdc6;
    border-radius: 6px;
    padding: 6px 14px;
}}
QPushButton:hover {{ background: #f1f0ec; }}
QPushButton:pressed {{ background: #e7e6e1; }}
QPushButton:disabled {{ color: #b5b3ad; border-color: #e3e2dc; background: #fafaf8; }}
QPushButton#primary {{
    background: {ACCENT}; color: #ffffff; border: 1px solid {ACCENT}; font-weight: 600;
}}
QPushButton#primary:hover {{ background: {ACCENT_DARK}; }}
QPushButton#primary:disabled {{ background: #b9d1ef; border-color: #b9d1ef; color: #ffffff; }}
QPushButton#stop {{
    background: {CRITICAL}; color: #ffffff; border: 1px solid #b52f2f; border-radius: 8px;
    font-weight: 700; padding: 8px 22px;
}}
QPushButton#stop:hover {{ background: #bb3030; }}
QPushButton#stop:pressed {{ background: #a12828; }}
QPushButton#stop:disabled {{ background: #e9b3b3; border-color: #e9b3b3; }}
QPushButton#record:checked {{ background: {CRITICAL}; color: #ffffff; border-color: #b52f2f; }}
QTabWidget::pane {{ border: 1px solid {BORDER}; border-radius: 8px; background: {SURFACE}; top: -1px; }}
QTabBar::tab {{
    background: transparent; padding: 7px 16px; border: 1px solid transparent;
    border-top-left-radius: 6px; border-top-right-radius: 6px; color: {INK_SECONDARY};
}}
QTabBar::tab:selected {{ background: {SURFACE}; border-color: {BORDER}; border-bottom-color: {SURFACE}; color: {INK}; }}
QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox {{
    background: #ffffff; border: 1px solid #cfcdc6; border-radius: 5px; padding: 4px 6px; min-height: 20px;
}}
QLineEdit:focus, QDoubleSpinBox:focus, QSpinBox:focus, QComboBox:focus {{ border: 1px solid {ACCENT}; }}
QDoubleSpinBox[invalid="true"], QLineEdit[invalid="true"] {{ border: 1px solid {CRITICAL}; }}
QProgressBar {{ border: 1px solid {BORDER}; border-radius: 4px; background: #ffffff; height: 10px; text-align: center; }}
QProgressBar::chunk {{ background: {ACCENT}; border-radius: 3px; }}
QPlainTextEdit {{ background: #ffffff; border: 1px solid {BORDER}; border-radius: 6px; }}
QTableWidget {{ background: #ffffff; border: 1px solid {BORDER}; gridline-color: {GRID}; }}
QHeaderView::section {{ background: {SURFACE}; border: none; border-bottom: 1px solid {BORDER}; padding: 4px; color: {INK_SECONDARY}; }}
QToolTip {{ background: #ffffff; color: {INK}; border: 1px solid {BORDER}; padding: 4px; }}
"""
