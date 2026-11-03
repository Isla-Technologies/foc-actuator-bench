"""Small reusable widgets: cards, stat tiles, the hero readout, status pill, form fields."""

from __future__ import annotations

import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from . import theme


def card(layout_cls=QVBoxLayout, margins=(14, 12, 14, 12), spacing=8) -> tuple[QFrame, QVBoxLayout]:
    frame = QFrame()
    frame.setObjectName("card")
    layout = layout_cls(frame)
    layout.setContentsMargins(*margins)
    layout.setSpacing(spacing)
    return frame, layout


def label(text: str = "", role: str | None = None, *, wrap: bool = False, size: int | None = None,
          weight: QFont.Weight | None = None) -> QLabel:
    widget = QLabel(text)
    if role:
        widget.setObjectName(role)
    if wrap:
        widget.setWordWrap(True)
    if size or weight:
        widget.setFont(theme.font(size or 13, weight or QFont.Normal))
    widget.setTextInteractionFlags(Qt.TextSelectableByMouse | Qt.LinksAccessibleByMouse)
    widget.setOpenExternalLinks(True)
    return widget


def section_title(text: str) -> QLabel:
    return label(text, "sectionTitle", size=14, weight=QFont.DemiBold)


def _no_negative_zero(text: str) -> str:
    return text[1:] if text.startswith("-") and not text.strip("-0.,") else text


def fmt(value: float, digits: int = 3) -> str:
    """Compact number: 3 significant digits from 1 up, `digits` decimals below 1."""
    if value is None or not math.isfinite(value):
        return "—"
    if abs(value) >= 1000:
        return _no_negative_zero(f"{value:,.0f}")
    if abs(value) >= 1:
        return f"{value:.{digits}g}"
    return _no_negative_zero(f"{value:.{digits}f}".rstrip("0").rstrip(".") or "0")


def fixed(value: float, decimals: int) -> str:
    if value is None or not math.isfinite(value):
        return "—"
    return _no_negative_zero(f"{value:.{decimals}f}")


class StatTile(QWidget):
    """Label above, value below (sentence-case label, no colon)."""

    def __init__(self, title: str, unit: str = "", parent=None, value_size: int = 19):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(1)
        self.title = label(title, "muted", size=12)
        self.value = label("—", size=value_size, weight=QFont.DemiBold)
        self.unit = unit
        layout.addWidget(self.title)
        layout.addWidget(self.value)

    def set(self, text: str) -> None:
        self.value.setText(f"{text} {self.unit}".strip() if text != "—" else "—")

    def set_unit(self, unit: str) -> None:
        self.unit = unit


class HeroReadout(QWidget):
    """The one big number of the measure page: output torque."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.title = label(title, "secondary", size=13, weight=QFont.DemiBold)
        row = QHBoxLayout()
        row.setSpacing(8)
        self.value = label("—", size=60, weight=QFont.DemiBold)
        self.value.setMinimumWidth(230)
        self.unit = label("N·m", "secondary", size=22)
        row.addWidget(self.value, 0, Qt.AlignBottom)
        row.addWidget(self.unit, 0, Qt.AlignBottom)
        row.addStretch(1)
        self.note = label("", "muted", size=12, wrap=True)
        layout.addWidget(self.title)
        layout.addLayout(row)
        layout.addWidget(self.note)


class StatusPill(QLabel):
    """State shown as icon + text on a tinted background (never color alone)."""

    STYLES = {
        "neutral": ("#efeee9", theme.INK_SECONDARY, "●"),
        "good": ("#e3f4e3", theme.SUCCESS_TEXT, "●"),
        "warning": ("#fdf1d6", "#7a5200", "▲"),
        "critical": ("#fae3e3", "#9b2222", "✖"),
        "info": ("#e5eefa", theme.ACCENT_DARK, "◆"),
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFont(theme.font(12, QFont.DemiBold))
        self.set("neutral", "Not connected")

    def set(self, kind: str, text: str) -> None:
        bg, fg, icon = self.STYLES[kind]
        self.setText(f"{icon}  {text}")
        self.setStyleSheet(f"background:{bg}; color:{fg}; border-radius:10px; padding:3px 10px;")


class NumberField(QDoubleSpinBox):
    """A spin box that tolerates typing and shows its unit."""

    def __init__(self, minimum: float, maximum: float, value: float, decimals: int = 2, step: float = 0.1,
                 suffix: str = "", parent=None):
        super().__init__(parent)
        self.setRange(minimum, maximum)
        self.setDecimals(decimals)
        self.setSingleStep(step)
        self.setValue(value)
        if suffix:
            self.setSuffix(f" {suffix}")
        self.setKeyboardTracking(False)
        self.setAccelerated(True)
        self.setMinimumWidth(120)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)

    def set_invalid(self, invalid: bool) -> None:
        self.setProperty("invalid", "true" if invalid else "false")
        self.style().unpolish(self)
        self.style().polish(self)


class StepIndicator(QWidget):
    """'1 Setup  ›  2 Connect  ›  3 Measure' with the current step highlighted."""

    clicked = Signal(int)

    def __init__(self, steps: list[str], parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        self.buttons: list[QPushButton] = []
        for i, name in enumerate(steps):
            if i:
                arrow = label("›", "muted", size=16)
                layout.addWidget(arrow)
            button = QPushButton(f"{i + 1}  {name}")
            button.setFlat(True)
            button.setCursor(Qt.PointingHandCursor)
            button.clicked.connect(lambda _=False, index=i: self.clicked.emit(index))
            self.buttons.append(button)
            layout.addWidget(button)
        layout.addStretch(1)

    def set_state(self, current: int, enabled: list[bool], done: list[bool]) -> None:
        for i, button in enumerate(self.buttons):
            name = button.text().split("  ", 1)[1]
            mark = "✓" if done[i] and i != current else str(i + 1)
            button.setText(f"{mark}  {name}")
            button.setEnabled(enabled[i])
            if i == current:
                style = f"color:{theme.ACCENT}; font-weight:700; border:none; border-bottom:2px solid {theme.ACCENT}; border-radius:0;"
            else:
                style = f"color:{theme.INK_SECONDARY}; border:none; border-bottom:2px solid transparent; border-radius:0;"
            button.setStyleSheet(f"QPushButton {{ background: transparent; padding: 6px 4px; {style} }}"
                                 f"QPushButton:disabled {{ color: #b5b3ad; }}")


def form_grid() -> QGridLayout:
    grid = QGridLayout()
    grid.setHorizontalSpacing(14)
    grid.setVerticalSpacing(8)
    grid.setColumnStretch(3, 1)
    return grid


def hline() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet(f"color: {theme.BORDER}; background: {theme.BORDER}; max-height: 1px;")
    return line
