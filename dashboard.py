# ==============================================================================
# dashboard.py — Main Application Entry Point
# Vehicle Sensor Suite Dashboard
# ==============================================================================
#
# USAGE:
#   python dashboard.py
#
# REQUIRES:
#   pip install PyQt6 pyqtgraph numpy labjack-ljm
#   LabJack LJM C driver installed from labjack.com
#
# ------------------------------------------------------------------------------
# CHANGELOG
# ------------------------------------------------------------------------------
# v2.5.2 — Spinbox up/down arrows visible in both themes (CSS border-triangle)
# v2.5.1 — Light mode color cycle (COLOR_CYCLE_LIGHT): muted professional tones
#           replace neon hues on light background; color reassignment on theme
#           switch propagates to cards, channel selector, and plot curves
# v2.5.0 — Light mode sidebar legibility; scrollable sidebar + cards row;
#           resizable sidebar splitter; fullscreen default; Travel XX labels;
#           theme-aware card recolor on theme switch; load cell default Med+EMA;
#           CSV browse button "Choose"; Acq Filter group sizing; RPM sample-hold
# v2.4.0 — RPM zero-on-stop fix (READ_A_AND_RESET); version tracking added
# v2.3.0 — Cascaded Median+EMA second EMA stage (ema2_alpha); EMA2 α spinbox
#           added to sidebar; live display-layer alpha push via set_ema2_alpha()
# v2.2.0 — Median+EMA filter implemented; Med+EMA option added to card dropdown
#           and sidebar acquisition filter; FILTER_EMA_ALPHA / FILTER_EMA2_ALPHA
#           constants added to config.py
# v2.1.0 — Two-layer filter architecture: acquisition-layer ChannelFilter
#           (affects CSV) + per-card display-layer filter (display/plot only);
#           FilterProfile enum (NONE/SMA/MEDIAN_EMA); sidebar Acq Filter group;
#           per-card filter QComboBox alongside Zero button
# v2.0.1 — Per-channel Zero button on ChannelValueCard; zero offset applied in
#           display layer only (CSV unaffected); offset cleared on stream stop
# v2.0.0 — Initial Python host-side acquisition architecture; PyQt6 + pyqtgraph
#           dashboard; dark/light theme; CSV recording; cross-plot tab
# ==============================================================================

from __future__ import annotations

import sys
import os
import time
import datetime
import collections
import numpy as np
from typing import List, Dict, Optional

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QCheckBox, QSpinBox,
    QDoubleSpinBox, QGroupBox, QSplitter, QFileDialog, QStatusBar,
    QScrollArea, QFrame, QLineEdit, QMessageBox, QSizePolicy, QTabWidget,
)
from PyQt6.QtCore import (
    Qt, QTimer, pyqtSignal, QObject, QThread
)
from PyQt6.QtGui import (
    QFont, QColor, QPalette, QIcon, QFontDatabase
)

import pyqtgraph as pg
import pyqtgraph.exporters

from config import (
    TestMode, ChannelConfig, MODE_CHANNELS, COLOR_CYCLE, COLOR_CYCLE_LIGHT,
    T7_IP_ADDRESS, T7_CONNECTION,
    SAMPLE_RATE_HZ_DEFAULT, SAMPLE_RATE_HZ_MIN, SAMPLE_RATE_HZ_MAX,
    PLOT_HISTORY_SECONDS, BUFFER_MAXLEN, PLOT_REFRESH_MS,
    CSV_DEFAULT_DIR, APP_TITLE, APP_VERSION,
    FILTER_EMA2_ALPHA,
)
from acquisition import (
    AcquisitionThread, RingBuffer, CSVLogger, AcqStatus, DataFrame,
    FilterProfile, ChannelFilter,
)


# ==============================================================================
# ==============================================================================
# THEME SYSTEM
# Two complete themes: DARK (default) and LIGHT.
# Active theme variables are set by apply_theme() and used by build_stylesheet().
# ==============================================================================

FONT_MONO    = "Consolas"
FONT_UI      = "Segoe UI"
PANEL_RADIUS = "8px"
BTN_RADIUS   = "5px"

# ---------- Dark theme (dark industrial) -------------------------------------
DARK = dict(
    BG_DARK      = "#0D0F12",
    BG_PANEL     = "#13161B",
    BG_WIDGET    = "#1A1E26",
    BG_INPUT     = "#1F2430",
    BORDER_COLOR = "#2A3040",
    ACCENT       = "#00D4FF",
    ACCENT_2     = "#FF6B35",
    TEXT_PRIMARY = "#E8EDF5",
    TEXT_MUTED   = "#6B7A99",
    TEXT_LABEL   = "#A0AABB",
    SUCCESS      = "#3DFF8F",
    WARNING      = "#FFD700",
    DANGER       = "#FF4444",
    PLOT_BG      = "#0A0C0F",
    GRID_COLOR   = "#1C2030",
    # Button active states (filled background when streaming/recording)
    BTN_STREAM_ACTIVE_BG  = "#0A3020",
    BTN_STREAM_ACTIVE_FG  = "#3DFF8F",
    BTN_STOP_ACTIVE_BG    = "#2B0D0D",
    BTN_STOP_ACTIVE_FG    = "#FF4444",
    BTN_REC_ACTIVE_BG     = "#2B1800",
    BTN_REC_ACTIVE_FG     = "#FFD700",
)

# ---------- Light theme (light gray industrial + blue accents) ---------------
LIGHT = dict(
    BG_DARK      = "#E8EAED",
    BG_PANEL     = "#D8DCE2",
    BG_WIDGET    = "#ECEEF1",
    BG_INPUT     = "#F4F5F7",
    BORDER_COLOR = "#B0B8C4",
    ACCENT       = "#1565C0",
    ACCENT_2     = "#E65100",
    TEXT_PRIMARY = "#1A1E2A",
    TEXT_MUTED   = "#546E7A",
    TEXT_LABEL   = "#37474F",
    SUCCESS      = "#1B5E20",
    WARNING      = "#E65100",
    DANGER       = "#B71C1C",
    PLOT_BG      = "#F8F9FA",
    GRID_COLOR   = "#CFD8DC",
    # Button active states
    BTN_STREAM_ACTIVE_BG  = "#C8E6C9",
    BTN_STREAM_ACTIVE_FG  = "#1B5E20",
    BTN_STOP_ACTIVE_BG    = "#FFCDD2",
    BTN_STOP_ACTIVE_FG    = "#B71C1C",
    BTN_REC_ACTIVE_BG     = "#FFF3E0",
    BTN_REC_ACTIVE_FG     = "#E65100",
)

# Active theme — module-level variables updated by apply_theme()
_theme = DARK.copy()

# Expose as module-level names so existing code references still work
BG_DARK      = _theme["BG_DARK"]
BG_PANEL     = _theme["BG_PANEL"]
BG_WIDGET    = _theme["BG_WIDGET"]
BG_INPUT     = _theme["BG_INPUT"]
BORDER_COLOR = _theme["BORDER_COLOR"]
ACCENT       = _theme["ACCENT"]
ACCENT_2     = _theme["ACCENT_2"]
TEXT_PRIMARY = _theme["TEXT_PRIMARY"]
TEXT_MUTED   = _theme["TEXT_MUTED"]
TEXT_LABEL   = _theme["TEXT_LABEL"]
SUCCESS      = _theme["SUCCESS"]
WARNING      = _theme["WARNING"]
DANGER       = _theme["DANGER"]
PLOT_BG      = _theme["PLOT_BG"]
GRID_COLOR   = _theme["GRID_COLOR"]


def build_stylesheet(t: dict) -> str:
    """Generate a complete Qt stylesheet from a theme dict."""
    return f"""
QMainWindow, QWidget {{
    background-color: {t['BG_DARK']};
    color: {t['TEXT_PRIMARY']};
    font-family: "{FONT_UI}", sans-serif;
    font-size: 13px;
}}
QGroupBox {{
    background-color: {t['BG_PANEL']};
    border: 1px solid {t['BORDER_COLOR']};
    border-radius: {PANEL_RADIUS};
    margin-top: 18px;
    padding: 8px;
    font-size: 11px;
    font-weight: 600;
    color: {t['TEXT_MUTED']};
    text-transform: uppercase;
    letter-spacing: 1px;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 4px;
    color: {t['TEXT_MUTED']};
}}
QPushButton {{
    background-color: {t['BG_WIDGET']};
    border: 1px solid {t['BORDER_COLOR']};
    border-radius: {BTN_RADIUS};
    padding: 7px 16px;
    color: {t['TEXT_PRIMARY']};
    font-weight: 500;
    min-height: 28px;
}}
QPushButton:hover {{
    background-color: {t['BG_INPUT']};
    border-color: {t['ACCENT']};
    color: {t['ACCENT']};
}}
QPushButton:pressed {{
    background-color: {t['BORDER_COLOR']};
}}
QPushButton:disabled {{
    color: {t['TEXT_MUTED']};
    border-color: {t['BG_WIDGET']};
}}
QPushButton#btn_start {{
    background-color: {t['BG_WIDGET']};
    border: 2px solid {t['SUCCESS']};
    color: {t['SUCCESS']};
    font-weight: 700;
    font-size: 13px;
    padding: 9px 24px;
}}
QPushButton#btn_start:hover {{
    background-color: {t['BTN_STREAM_ACTIVE_BG']};
}}
QPushButton#btn_start_active {{
    background-color: {t['BTN_STREAM_ACTIVE_BG']};
    border: 2px solid {t['SUCCESS']};
    color: {t['BTN_STREAM_ACTIVE_FG']};
    font-weight: 700;
    font-size: 13px;
    padding: 9px 24px;
}}
QPushButton#btn_stop {{
    background-color: {t['BG_WIDGET']};
    border: 2px solid {t['DANGER']};
    color: {t['DANGER']};
    font-weight: 700;
    font-size: 13px;
    padding: 9px 24px;
}}
QPushButton#btn_stop:hover {{
    background-color: {t['BTN_STOP_ACTIVE_BG']};
}}
QPushButton#btn_stop_active {{
    background-color: {t['BTN_STOP_ACTIVE_BG']};
    border: 2px solid {t['DANGER']};
    color: {t['BTN_STOP_ACTIVE_FG']};
    font-weight: 700;
    font-size: 13px;
    padding: 9px 24px;
}}
QPushButton#btn_record {{
    background-color: {t['BG_WIDGET']};
    border: 2px solid {t['WARNING']};
    color: {t['WARNING']};
    font-weight: 600;
    padding: 7px 16px;
}}
QPushButton#btn_record:hover {{
    background-color: {t['BTN_REC_ACTIVE_BG']};
}}
QPushButton#btn_record_active {{
    background-color: {t['BTN_REC_ACTIVE_BG']};
    border: 2px solid {t['WARNING']};
    color: {t['BTN_REC_ACTIVE_FG']};
    font-weight: 700;
    padding: 7px 16px;
}}
QComboBox {{
    background-color: {t['BG_INPUT']};
    border: 1px solid {t['BORDER_COLOR']};
    border-radius: {BTN_RADIUS};
    padding: 5px 10px;
    color: {t['TEXT_PRIMARY']};
    min-height: 28px;
}}
QComboBox:hover {{
    border-color: {t['ACCENT']};
}}
QComboBox::drop-down {{
    border: none;
    width: 20px;
}}
QComboBox QAbstractItemView {{
    background-color: {t['BG_INPUT']};
    border: 1px solid {t['BORDER_COLOR']};
    selection-background-color: {t['BG_WIDGET']};
    color: {t['TEXT_PRIMARY']};
}}
QSpinBox, QDoubleSpinBox {{
    background-color: {t['BG_INPUT']};
    border: 1px solid {t['BORDER_COLOR']};
    border-radius: {BTN_RADIUS};
    padding: 4px 24px 4px 8px;
    color: {t['TEXT_PRIMARY']};
    min-height: 26px;
}}
QSpinBox:hover, QDoubleSpinBox:hover {{
    border-color: {t['ACCENT']};
}}
QSpinBox::up-button, QDoubleSpinBox::up-button {{
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 20px;
    border-left: 1px solid {t['BORDER_COLOR']};
    border-bottom: 1px solid {t['BORDER_COLOR']};
    border-top-right-radius: {BTN_RADIUS};
    background-color: {t['BG_WIDGET']};
}}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover {{
    background-color: {t['BG_INPUT']};
}}
QSpinBox::up-button:pressed, QDoubleSpinBox::up-button:pressed {{
    background-color: {t['BORDER_COLOR']};
}}
QSpinBox::down-button, QDoubleSpinBox::down-button {{
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 20px;
    border-left: 1px solid {t['BORDER_COLOR']};
    border-top: 1px solid {t['BORDER_COLOR']};
    border-bottom-right-radius: {BTN_RADIUS};
    background-color: {t['BG_WIDGET']};
}}
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {{
    background-color: {t['BG_INPUT']};
}}
QSpinBox::down-button:pressed, QDoubleSpinBox::down-button:pressed {{
    background-color: {t['BORDER_COLOR']};
}}
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {{
    image: none;
    width: 0px;
    height: 0px;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-bottom: 5px solid {t['TEXT_MUTED']};
}}
QSpinBox::up-arrow:hover, QDoubleSpinBox::up-arrow:hover {{
    border-bottom-color: {t['TEXT_PRIMARY']};
}}
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {{
    image: none;
    width: 0px;
    height: 0px;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid {t['TEXT_MUTED']};
}}
QSpinBox::down-arrow:hover, QDoubleSpinBox::down-arrow:hover {{
    border-top-color: {t['TEXT_PRIMARY']};
}}
QCheckBox {{
    color: {t['TEXT_LABEL']};
    spacing: 6px;
}}
QCheckBox::indicator {{
    width: 14px;
    height: 14px;
    border: 1px solid {t['BORDER_COLOR']};
    border-radius: 3px;
    background: {t['BG_INPUT']};
}}
QCheckBox::indicator:checked {{
    background-color: {t['ACCENT']};
    border-color: {t['ACCENT']};
}}
QCheckBox:hover {{
    color: {t['TEXT_PRIMARY']};
}}
QLabel {{
    color: {t['TEXT_LABEL']};
}}
QLabel#value_label {{
    color: {t['TEXT_PRIMARY']};
    font-family: "{FONT_MONO}";
    font-size: 16px;
    font-weight: 600;
}}
QLabel#channel_name {{
    color: {t['TEXT_MUTED']};
    font-size: 11px;
    letter-spacing: 0.5px;
}}
QScrollArea {{
    border: none;
    background: transparent;
}}
QScrollBar:vertical {{
    background: {t['BG_PANEL']};
    width: 6px;
    border-radius: 3px;
}}
QScrollBar::handle:vertical {{
    background: {t['BORDER_COLOR']};
    border-radius: 3px;
    min-height: 20px;
}}
QScrollBar::handle:vertical:hover {{
    background: {t['TEXT_MUTED']};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}
QLineEdit {{
    background-color: {t['BG_INPUT']};
    border: 1px solid {t['BORDER_COLOR']};
    border-radius: {BTN_RADIUS};
    padding: 5px 8px;
    color: {t['TEXT_PRIMARY']};
}}
QLineEdit:hover {{
    border-color: {t['ACCENT']};
}}
QStatusBar {{
    background-color: {t['BG_PANEL']};
    color: {t['TEXT_MUTED']};
    border-top: 1px solid {t['BORDER_COLOR']};
    font-size: 11px;
}}
QSplitter::handle {{
    background-color: {t['BORDER_COLOR']};
    width: 2px;
}}
QTabWidget::pane {{
    border: 1px solid {t['BORDER_COLOR']};
    border-radius: {PANEL_RADIUS};
    background: {t['BG_PANEL']};
}}
QTabBar::tab {{
    background: {t['BG_WIDGET']};
    border: 1px solid {t['BORDER_COLOR']};
    border-bottom: none;
    padding: 6px 16px;
    color: {t['TEXT_MUTED']};
    margin-right: 2px;
    border-top-left-radius: 5px;
    border-top-right-radius: 5px;
}}
QTabBar::tab:selected {{
    background: {t['BG_PANEL']};
    color: {t['ACCENT']};
    border-color: {t['ACCENT']};
}}
QTabBar::tab:hover {{
    color: {t['TEXT_PRIMARY']};
}}
"""

STYLESHEET = build_stylesheet(DARK)

_CURRENT_THEME = DARK  # Track active theme for dynamic widget updates


def apply_theme(theme_dict: dict, app=None) -> str:
    """
    Switch the active theme. Updates module-level color variables so that
    dynamic inline stylesheets (value cards, plot axes, etc.) pick up the
    new colors on next redraw. Returns the new stylesheet string.
    """
    global _CURRENT_THEME
    global BG_DARK, BG_PANEL, BG_WIDGET, BG_INPUT, BORDER_COLOR
    global ACCENT, ACCENT_2, TEXT_PRIMARY, TEXT_MUTED, TEXT_LABEL
    global SUCCESS, WARNING, DANGER, PLOT_BG, GRID_COLOR
    _CURRENT_THEME = theme_dict
    BG_DARK      = theme_dict["BG_DARK"]
    BG_PANEL     = theme_dict["BG_PANEL"]
    BG_WIDGET    = theme_dict["BG_WIDGET"]
    BG_INPUT     = theme_dict["BG_INPUT"]
    BORDER_COLOR = theme_dict["BORDER_COLOR"]
    ACCENT       = theme_dict["ACCENT"]
    ACCENT_2     = theme_dict["ACCENT_2"]
    TEXT_PRIMARY = theme_dict["TEXT_PRIMARY"]
    TEXT_MUTED   = theme_dict["TEXT_MUTED"]
    TEXT_LABEL   = theme_dict["TEXT_LABEL"]
    SUCCESS      = theme_dict["SUCCESS"]
    WARNING      = theme_dict["WARNING"]
    DANGER       = theme_dict["DANGER"]
    PLOT_BG      = theme_dict["PLOT_BG"]
    GRID_COLOR   = theme_dict["GRID_COLOR"]
    ss = build_stylesheet(theme_dict)
    if app:
        app.setStyleSheet(ss)
    return ss


# ==============================================================================
# PYQTGRAPH GLOBAL CONFIGURATION
# ==============================================================================
pg.setConfigOptions(
    antialias=True,
    background=DARK["PLOT_BG"],
    foreground=DARK["TEXT_LABEL"],
    useOpenGL=False,  # Set True if GPU acceleration is available
)


# ==============================================================================
# NUMERIC VALUE DISPLAY WIDGET
# Small card showing channel name + live value + unit
# ==============================================================================
class ChannelValueCard(QFrame):
    """
    Live value display card for one channel.

    TOP ROW CONTROLS
    ----------------
    Zero button   — Latches the current live reading as a display offset.
                    Click again to clear.  Purely display-layer; CSV is unaffected.
    Filter combo  — Selects a display-layer filter applied on top of the
                    acquisition-layer filter already in the ring buffer data.
                    Use "None" to see the acquisition-filtered signal exactly.
                    Use "SMA" for additional visual smoothing without affecting
                    the logged data.

    FILTER LAYERING
    ---------------
    Acquisition layer (acquisition.py)  →  ring buffer  →  CSV log
                                                        ↓
                                           ChannelValueCard display filter
                                                        ↓
                                              card numeric display + plot
    """

    # Filter options shown in the dropdown.
    # Tuple of (display label, FilterProfile, sma_n).
    _FILTER_OPTIONS = [
        ("None",     FilterProfile.NONE,       5),
        ("SMA 5",    FilterProfile.SMA,        5),
        ("SMA 10",   FilterProfile.SMA,        10),
        ("Med+EMA",  FilterProfile.MEDIAN_EMA, 5),
    ]

    def __init__(self, channel: ChannelConfig, color: str, parent=None):
        super().__init__(parent)
        self.channel = channel
        self.color   = color

        self._display_offset: float = 0.0
        self._last_raw: float       = 0.0
        self._is_zeroed: bool       = False
        self._ema2_alpha: float     = FILTER_EMA2_ALPHA

        # Display-layer filter — default from ChannelConfig.default_display_filter
        _default_map = {
            "none":    (FilterProfile.NONE,       5),
            "sma5":    (FilterProfile.SMA,        5),
            "sma10":   (FilterProfile.SMA,        10),
            "med_ema": (FilterProfile.MEDIAN_EMA, 5),
        }
        _dp, _dn = _default_map.get(channel.default_display_filter,
                                    (FilterProfile.NONE, 5))
        self._display_filter = ChannelFilter(_dp, sma_n=_dn,
                                             ema2_alpha=self._ema2_alpha)
        # Index in _FILTER_OPTIONS that matches the default
        _default_idx = next(
            (i for i, (_, p, n) in enumerate(self._FILTER_OPTIONS)
             if p == _dp and n == _dn), 0
        )

        self.setFixedHeight(86)
        self.setStyleSheet(
            f"QFrame {{ background-color: {BG_WIDGET}; "
            f"border: 1px solid {BORDER_COLOR}; "
            f"border-left: 3px solid {color}; "
            f"border-radius: 6px; }}"
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 5, 8, 5)
        layout.setSpacing(2)

        # ── Row 1: channel name + filter combo + Zero button ─────────────────
        top_row = QHBoxLayout()
        top_row.setSpacing(4)

        self.name_label = QLabel(channel.label.upper())
        self.name_label.setObjectName("channel_name")
        self.name_label.setStyleSheet(
            f"color: {color}; font-size: 10px; "
            f"letter-spacing: 0.8px; font-weight: 600;"
        )

        # Filter selector combo
        self._filter_combo = QComboBox()
        self._filter_combo.setFixedSize(66, 18)
        self._filter_combo.setToolTip(
            "Display-layer filter (card + plot only).\n"
            "Does not affect CSV-logged data.\n"
            "For acquisition-layer filtering set the\n"
            "Filter Profile in the sidebar."
        )
        for label, _profile, _n in self._FILTER_OPTIONS:
            self._filter_combo.addItem(label)
        # Set default selection without triggering _on_filter_changed rebuild
        # (filter already built above from _default_idx)
        self._filter_combo.blockSignals(True)
        self._filter_combo.setCurrentIndex(_default_idx)
        self._filter_combo.blockSignals(False)
        self._filter_combo.setStyleSheet(
            f"QComboBox {{"
            f"  background-color: {BG_INPUT};"
            f"  border: 1px solid {BORDER_COLOR};"
            f"  border-radius: 3px;"
            f"  color: {TEXT_MUTED};"
            f"  font-size: 9px;"
            f"  padding: 0px 2px;"
            f"}}"
            f"QComboBox:hover {{ border-color: {color}; color: {color}; }}"
            f"QComboBox::drop-down {{ border: none; width: 12px; }}"
        )
        self._filter_combo.currentIndexChanged.connect(self._on_filter_changed)

        self._btn_zero = QPushButton("Zero")
        self._btn_zero.setFixedSize(42, 18)
        self._btn_zero.setToolTip(
            "Latch current reading as zero reference.\n"
            "Click again to clear the offset."
        )
        self._btn_zero.setStyleSheet(
            f"QPushButton {{"
            f"  background-color: {BG_INPUT};"
            f"  border: 1px solid {BORDER_COLOR};"
            f"  border-radius: 3px;"
            f"  color: {TEXT_MUTED};"
            f"  font-size: 9px;"
            f"  font-weight: 600;"
            f"  padding: 0px;"
            f"}}"
            f"QPushButton:hover {{"
            f"  border-color: {color}; color: {color};"
            f"}}"
        )
        self._btn_zero.clicked.connect(self._toggle_zero)

        top_row.addWidget(self.name_label)
        top_row.addStretch()
        top_row.addWidget(self._filter_combo)
        top_row.addWidget(self._btn_zero)

        # ── Row 2: numeric value + unit ──────────────────────────────────────
        self.value_label = QLabel("—")
        self.value_label.setObjectName("value_label")
        self.value_label.setStyleSheet(
            f"color: {TEXT_PRIMARY}; font-family: {FONT_MONO}; "
            f"font-size: 18px; font-weight: 700;"
        )
        self.unit_label = QLabel(channel.unit)
        self.unit_label.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 11px;"
        )

        val_row = QHBoxLayout()
        val_row.setSpacing(4)
        val_row.addWidget(self.value_label)
        val_row.addWidget(self.unit_label)
        val_row.addStretch()

        layout.addLayout(top_row)
        layout.addLayout(val_row)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def display_offset(self) -> float:
        return self._display_offset

    def update_value(self, acq_filtered_value: float) -> float:
        """
        Receive an acquisition-filtered value, run the display-layer filter,
        apply the zero offset, update the label, and return the final display
        value so _refresh_plots can pass it to the plot without recomputing.
        """
        # Display-layer filter pass
        display_val = self._display_filter.push(acq_filtered_value)
        self._last_raw = display_val   # Zero latches against post-display-filter value

        # Zero offset
        display_val -= self._display_offset
        self.value_label.setText(f"{display_val:,.4g}")
        return display_val

    def reset_zero(self):
        """Clear display offset (on stream stop)."""
        self._display_offset = 0.0
        self._is_zeroed = False
        self._display_filter.reset()
        self._update_zero_btn_style(active=False)

    def refresh_theme(self, theme_dict: dict, color: str):
        """
        Called by MainWindow._apply_theme() to recolor the card when the
        user switches between dark and light mode mid-session.
        Updates frame border, name label, filter combo, and zero button.
        """
        self.color = color
        bg_widget   = theme_dict["BG_WIDGET"]
        border      = theme_dict["BORDER_COLOR"]
        bg_input    = theme_dict["BG_INPUT"]
        text_muted  = theme_dict["TEXT_MUTED"]
        text_primary= theme_dict["TEXT_PRIMARY"]

        self.setStyleSheet(
            f"QFrame {{ background-color: {bg_widget}; "
            f"border: 1px solid {border}; "
            f"border-left: 3px solid {color}; "
            f"border-radius: 6px; }}"
        )
        self.name_label.setStyleSheet(
            f"color: {color}; font-size: 10px; "
            f"letter-spacing: 0.8px; font-weight: 600;"
        )
        self.value_label.setStyleSheet(
            f"color: {text_primary}; font-family: {FONT_MONO}; "
            f"font-size: 18px; font-weight: 700;"
        )
        self.unit_label.setStyleSheet(
            f"color: {text_muted}; font-size: 11px;"
        )
        self._filter_combo.setStyleSheet(
            f"QComboBox {{"
            f"  background-color: {bg_input};"
            f"  border: 1px solid {border};"
            f"  border-radius: 3px;"
            f"  color: {text_muted};"
            f"  font-size: 9px; padding: 0px 2px;"
            f"}}"
            f"QComboBox:hover {{ border-color: {color}; color: {color}; }}"
            f"QComboBox::drop-down {{ border: none; width: 12px; }}"
        )
        # Reapply zero button style with new colours
        self._update_zero_btn_style(active=self._is_zeroed)

    # ------------------------------------------------------------------
    # Internal slots
    # ------------------------------------------------------------------

    def set_ema2_alpha(self, alpha: float):
        """
        Called by MainWindow when the EMA2 α spinbox changes.
        Rebuilds the display filter with the new alpha so changes are
        reflected immediately in the live stream without a stream restart.
        """
        self._ema2_alpha = alpha
        # Rebuild with current profile/n so the new alpha takes effect now
        self._on_filter_changed(self._filter_combo.currentIndex())

    def _on_filter_changed(self, index: int):
        _label, profile, n = self._FILTER_OPTIONS[index]
        self._display_filter = ChannelFilter(
            profile, sma_n=n, ema2_alpha=self._ema2_alpha
        )

    def _toggle_zero(self):
        if self._is_zeroed:
            self._display_offset = 0.0
            self._is_zeroed = False
            self._update_zero_btn_style(active=False)
            self.value_label.setText(f"{self._last_raw:,.4g}")
        else:
            self._display_offset = self._last_raw
            self._is_zeroed = True
            self._update_zero_btn_style(active=True)
            self.value_label.setText("0.0000")

    def _update_zero_btn_style(self, active: bool):
        if active:
            self._btn_zero.setText("✕ Zero")
            self._btn_zero.setStyleSheet(
                f"QPushButton {{"
                f"  background-color: {BG_INPUT};"
                f"  border: 1px solid {self.color};"
                f"  border-radius: 3px;"
                f"  color: {self.color};"
                f"  font-size: 9px; font-weight: 700; padding: 0px;"
                f"}}"
                f"QPushButton:hover {{ background-color: {BG_WIDGET}; }}"
            )
        else:
            self._btn_zero.setText("Zero")
            self._btn_zero.setStyleSheet(
                f"QPushButton {{"
                f"  background-color: {BG_INPUT};"
                f"  border: 1px solid {BORDER_COLOR};"
                f"  border-radius: 3px;"
                f"  color: {TEXT_MUTED};"
                f"  font-size: 9px; font-weight: 600; padding: 0px;"
                f"}}"
                f"QPushButton:hover {{"
                f"  border-color: {self.color}; color: {self.color};"
                f"}}"
            )


# ==============================================================================
# MULTI-CHANNEL PLOT WIDGET
# One pyqtgraph PlotWidget with per-channel curves, legend, and cross-plot
# ==============================================================================
class MultiChannelPlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._curves: Dict[str, pg.PlotDataItem] = {}
        self._buffers: Dict[str, collections.deque] = {}
        self._time_buf: collections.deque = collections.deque(maxlen=BUFFER_MAXLEN)
        self._channels: List[ChannelConfig] = []
        self._colors: Dict[str, str] = {}
        self._history_s = PLOT_HISTORY_SECONDS
        self._x_mode = "rolling"   # "rolling" or "absolute"

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Toolbar row
        toolbar = QHBoxLayout()
        toolbar.setSpacing(8)

        lbl_history = QLabel("Window:")
        lbl_history.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        self._spin_history = QSpinBox()
        self._spin_history.setRange(5, 300)
        self._spin_history.setValue(PLOT_HISTORY_SECONDS)
        self._spin_history.setSuffix(" s")
        self._spin_history.setFixedWidth(75)
        self._spin_history.valueChanged.connect(self._on_history_changed)

        self._btn_autorange = QPushButton("Auto Range")
        self._btn_autorange.setMinimumWidth(100)
        self._btn_autorange.clicked.connect(self._auto_range)

        self._btn_clear = QPushButton("Clear")
        self._btn_clear.setMinimumWidth(70)
        self._btn_clear.clicked.connect(self.clear_data)

        self._combo_xmode = QComboBox()
        self._combo_xmode.addItems(["Rolling Window", "Absolute Time"])
        self._combo_xmode.setFixedWidth(140)
        self._combo_xmode.currentTextChanged.connect(self._on_xmode_changed)

        toolbar.addWidget(lbl_history)
        toolbar.addWidget(self._spin_history)
        toolbar.addWidget(self._combo_xmode)
        toolbar.addStretch()
        toolbar.addWidget(self._btn_autorange)
        toolbar.addWidget(self._btn_clear)

        # Plot widget
        self._plot_widget = pg.PlotWidget()
        self._plot_widget.showGrid(x=True, y=True,
                                    alpha=0.15)
        self._plot_widget.getAxis("bottom").setLabel("Time", units="s")
        self._plot_widget.getAxis("left").setStyle(tickFont=QFont(FONT_MONO, 9))
        self._plot_widget.getAxis("bottom").setStyle(tickFont=QFont(FONT_MONO, 9))
        self._plot_widget.setBackground(PLOT_BG)

        # Grid styling
        for axis in ["left", "bottom"]:
            ax = self._plot_widget.getAxis(axis)
            ax.setPen(pg.mkPen(GRID_COLOR, width=1))
            ax.setTextPen(pg.mkPen(TEXT_MUTED))

        # Legend
        self._legend = self._plot_widget.addLegend(
            offset=(10, 10),
            labelTextColor=TEXT_LABEL,
            brush=pg.mkBrush(BG_PANEL + "CC"),
            pen=pg.mkPen(BORDER_COLOR),
        )

        # Crosshair
        self._vline = pg.InfiniteLine(angle=90, movable=False,
                                       pen=pg.mkPen(TEXT_MUTED, width=1,
                                                     style=Qt.PenStyle.DashLine))
        self._hline = pg.InfiniteLine(angle=0, movable=False,
                                       pen=pg.mkPen(TEXT_MUTED, width=1,
                                                     style=Qt.PenStyle.DashLine))
        self._plot_widget.addItem(self._vline, ignoreBounds=True)
        self._plot_widget.addItem(self._hline, ignoreBounds=True)
        self._plot_widget.scene().sigMouseMoved.connect(self._on_mouse_moved)

        layout.addLayout(toolbar)
        layout.addWidget(self._plot_widget)

    def configure_channels(self, channels: List[ChannelConfig],
                            colors: Dict[str, str]):
        """Set up curves for the given list of channels."""
        # Remove old curves
        for key, curve in self._curves.items():
            self._plot_widget.removeItem(curve)
        self._curves.clear()
        self._buffers.clear()
        self._time_buf.clear()
        self._channels = channels
        self._colors = colors

        for ch in channels:
            color = colors.get(ch.key, "#FFFFFF")
            pen = pg.mkPen(color=color, width=2)
            curve = self._plot_widget.plot(
                [], [], pen=pen, name=f"{ch.label} ({ch.unit})"
            )
            self._curves[ch.key] = curve
            self._buffers[ch.key] = collections.deque(maxlen=BUFFER_MAXLEN)

        # Set Y range hint from first channel if available
        if channels and channels[0].y_range:
            self._plot_widget.setYRange(*channels[0].y_range, padding=0.05)

    def push_frame(self, frame: DataFrame):
        """Add a new data frame to the plot buffers."""
        t = frame.t7_timestamp_ms / 1000.0  # Convert ms → s
        self._time_buf.append(t)
        for ch in self._channels:
            if ch.key in frame.values:
                self._buffers[ch.key].append(frame.values[ch.key])

    def refresh(self):
        """Redraw all curves. Called by the UI timer."""
        if not self._time_buf:
            return

        t_arr = np.array(self._time_buf)
        now = t_arr[-1]

        if self._x_mode == "rolling":
            t_min = now - self._history_s
            mask = t_arr >= t_min
            t_plot = t_arr[mask] - now  # Relative: 0 = now, -N = N sec ago
        else:
            mask = np.ones(len(t_arr), dtype=bool)
            t_plot = t_arr

        for ch in self._channels:
            buf = self._buffers.get(ch.key)
            if buf is None:
                continue
            y_arr = np.array(buf)
            # Align lengths (t and y buffers should match but may diverge briefly)
            min_len = min(len(t_plot), len(y_arr))
            if min_len < 2:
                continue
            self._curves[ch.key].setData(
                t_plot[-min_len:], y_arr[-min_len:]
            )

    def clear_data(self):
        self._time_buf.clear()
        for key in self._buffers:
            self._buffers[key].clear()
        for curve in self._curves.values():
            curve.setData([], [])

    def _on_history_changed(self, value):
        self._history_s = value

    def _on_xmode_changed(self, text):
        self._x_mode = "rolling" if "Rolling" in text else "absolute"
        if self._x_mode == "rolling":
            self._plot_widget.getAxis("bottom").setLabel("Time", units="s")
        else:
            self._plot_widget.getAxis("bottom").setLabel("Elapsed Time", units="s")

    def _auto_range(self):
        self._plot_widget.enableAutoRange()

    def _on_mouse_moved(self, pos):
        if self._plot_widget.sceneBoundingRect().contains(pos):
            mp = self._plot_widget.getPlotItem().vb.mapSceneToView(pos)
            self._vline.setPos(mp.x())
            self._hline.setPos(mp.y())


# ==============================================================================
# CHANNEL SELECTOR PANEL
# Sidebar list of checkboxes for enabling/disabling channels on the active plot
# ==============================================================================
class ChannelSelectorPanel(QWidget):
    channels_changed = pyqtSignal(list)  # Emits list of enabled ChannelConfig

    def __init__(self, parent=None):
        super().__init__(parent)
        self._checkboxes: Dict[str, QCheckBox] = {}
        self._channels: List[ChannelConfig] = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        # Header
        hdr = QLabel("CHANNELS")
        hdr.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 10px; font-weight: 700; "
            f"letter-spacing: 1.5px; padding: 4px 0;"
        )
        layout.addWidget(hdr)

        # Scrollable checkbox area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._scroll_content = QWidget()
        self._scroll_layout = QVBoxLayout(self._scroll_content)
        self._scroll_layout.setSpacing(2)
        self._scroll_layout.setContentsMargins(0, 0, 0, 0)
        self._scroll_layout.addStretch()
        scroll.setWidget(self._scroll_content)

        # Select all / none
        btn_row = QHBoxLayout()
        btn_all = QPushButton("All")
        btn_none = QPushButton("None")
        btn_all.setMinimumHeight(28)
        btn_none.setMinimumHeight(28)
        btn_all.clicked.connect(self._select_all)
        btn_none.clicked.connect(self._select_none)
        btn_row.addWidget(btn_all)
        btn_row.addWidget(btn_none)
        btn_row.setContentsMargins(0, 0, 0, 6)

        layout.addLayout(btn_row)
        layout.addWidget(scroll)

    def load_channels(self, channels: List[ChannelConfig],
                       colors: Dict[str, str]):
        """Populate checkboxes for the given channel list."""
        # Clear existing
        for i in reversed(range(self._scroll_layout.count())):
            item = self._scroll_layout.itemAt(i)
            if item.widget():
                item.widget().deleteLater()
        self._checkboxes.clear()
        self._channels = [ch for ch in channels if not ch.is_timestamp]

        for ch in self._channels:
            color = colors.get(ch.key, TEXT_LABEL)
            cb = QCheckBox(f"  {ch.label}")
            cb.setChecked(ch.enabled_by_default)
            cb.setStyleSheet(
                f"QCheckBox {{ color: {color}; font-size: 12px; }}"
                f"QCheckBox::indicator:checked {{ background-color: {color}; "
                f"border-color: {color}; }}"
            )
            cb.stateChanged.connect(self._emit_changed)
            self._scroll_layout.insertWidget(
                self._scroll_layout.count() - 1, cb
            )
            self._checkboxes[ch.key] = cb

    def get_enabled_channels(self) -> List[ChannelConfig]:
        return [
            ch for ch in self._channels
            if self._checkboxes.get(ch.key, QCheckBox()).isChecked()
        ]

    def _select_all(self):
        for cb in self._checkboxes.values():
            cb.setChecked(True)

    def _select_none(self):
        for cb in self._checkboxes.values():
            cb.setChecked(False)

    def _emit_changed(self):
        self.channels_changed.emit(self.get_enabled_channels())


# ==============================================================================
# CROSS-PLOT WIDGET
# Plot any channel on X axis vs any other channel on Y axis
# ==============================================================================
class CrossPlotWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._x_key: Optional[str] = None
        self._y_key: Optional[str] = None
        self._all_channels: List[ChannelConfig] = []
        self._data_store: Dict[str, collections.deque] = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        # Channel selector row
        sel_row = QHBoxLayout()
        lbl_x = QLabel("X Axis:")
        lbl_x.setFixedWidth(50)
        self._combo_x = QComboBox()
        self._combo_x.setMinimumWidth(160)

        lbl_y = QLabel("Y Axis:")
        lbl_y.setFixedWidth(50)
        self._combo_y = QComboBox()
        self._combo_y.setMinimumWidth(160)

        btn_apply = QPushButton("Apply")
        btn_apply.setFixedWidth(70)
        btn_apply.clicked.connect(self._apply_selection)

        sel_row.addWidget(lbl_x)
        sel_row.addWidget(self._combo_x)
        sel_row.addSpacing(16)
        sel_row.addWidget(lbl_y)
        sel_row.addWidget(self._combo_y)
        sel_row.addSpacing(8)
        sel_row.addWidget(btn_apply)
        sel_row.addStretch()

        # Plot
        self._plot = pg.PlotWidget()
        self._plot.showGrid(x=True, y=True, alpha=0.15)
        self._plot.setBackground(PLOT_BG)
        for axis in ["left", "bottom"]:
            ax = self._plot.getAxis(axis)
            ax.setPen(pg.mkPen(GRID_COLOR, width=1))
            ax.setTextPen(pg.mkPen(TEXT_MUTED))

        self._scatter = pg.ScatterPlotItem(
            size=4,
            pen=pg.mkPen(None),
            brush=pg.mkBrush(ACCENT + "99"),
        )
        self._plot.addItem(self._scatter)

        layout.addLayout(sel_row)
        layout.addWidget(self._plot)

    def configure(self, channels: List[ChannelConfig]):
        """Load available channels into the combo boxes."""
        self._all_channels = [ch for ch in channels if not ch.is_timestamp]
        self._data_store = {
            ch.key: collections.deque(maxlen=BUFFER_MAXLEN)
            for ch in self._all_channels
        }
        names = [f"{ch.label} ({ch.unit})" for ch in self._all_channels]
        for combo in (self._combo_x, self._combo_y):
            combo.clear()
            combo.addItems(names)
        if len(self._all_channels) >= 2:
            self._combo_y.setCurrentIndex(1)
        self._apply_selection()

    def push_frame(self, frame: DataFrame):
        for ch in self._all_channels:
            if ch.key in frame.values:
                self._data_store[ch.key].append(frame.values[ch.key])

    def _apply_selection(self):
        xi = self._combo_x.currentIndex()
        yi = self._combo_y.currentIndex()
        if xi < 0 or yi < 0 or xi >= len(self._all_channels):
            return
        self._x_key = self._all_channels[xi].key
        self._y_key = self._all_channels[yi].key
        xch = self._all_channels[xi]
        ych = self._all_channels[yi]
        self._plot.getAxis("bottom").setLabel(
            f"{xch.label}", units=xch.unit
        )
        self._plot.getAxis("left").setLabel(
            f"{ych.label}", units=ych.unit
        )

    def refresh(self):
        if not self._x_key or not self._y_key:
            return
        xbuf = self._data_store.get(self._x_key)
        ybuf = self._data_store.get(self._y_key)
        if not xbuf or not ybuf:
            return
        n = min(len(xbuf), len(ybuf))
        if n < 2:
            return
        xarr = list(xbuf)[-n:]
        yarr = list(ybuf)[-n:]
        self._scatter.setData(x=xarr, y=yarr)

    def clear_data(self):
        for buf in self._data_store.values():
            buf.clear()
        self._scatter.setData([], [])


# ==============================================================================
# MAIN WINDOW
# ==============================================================================
class MainWindow(QMainWindow):
    # Signal from acquisition thread → UI (thread-safe)
    _acq_status_signal = pyqtSignal(AcqStatus, str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_TITLE}  v{APP_VERSION}")
        self.setMinimumSize(1280, 800)
        self.showMaximized()

        # State
        self._mode: TestMode = TestMode.SUSPENSION
        self._acq_thread: Optional[AcquisitionThread] = None
        self._ring_buffer = RingBuffer()
        self._csv_logger = CSVLogger()
        self._is_streaming = False
        self._is_recording = False
        self._sample_rate = SAMPLE_RATE_HZ_DEFAULT
        self._frame_count = 0
        self._session_start: Optional[float] = None

        # Acquisition filter state — set by sidebar filter controls
        self._acq_filter_profile:    FilterProfile = FilterProfile.NONE
        self._acq_filter_n:          int   = 5
        self._acq_filter_ema2_alpha: float = FILTER_EMA2_ALPHA

        # Channel state
        self._all_channels: List[ChannelConfig] = []
        self._active_channels: List[ChannelConfig] = []
        self._channel_colors: Dict[str, str] = {}

        # Connect status signal (thread → UI)
        self._acq_status_signal.connect(self._on_acq_status)

        self._build_ui()
        self._load_mode(TestMode.SUSPENSION)

        # Plot refresh timer
        self._plot_timer = QTimer()
        self._plot_timer.setInterval(PLOT_REFRESH_MS)
        self._plot_timer.timeout.connect(self._refresh_plots)
        self._plot_timer.start()

        # Status bar update timer
        self._status_timer = QTimer()
        self._status_timer.setInterval(500)
        self._status_timer.timeout.connect(self._update_status_bar)
        self._status_timer.start()

    # --------------------------------------------------------------------------
    # UI Construction
    # --------------------------------------------------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── ROOT SPLITTER (sidebar | content) ─────────────────────────────────
        # QSplitter gives a draggable handle between sidebar and plot area.
        self._root_splitter = QSplitter(Qt.Orientation.Horizontal)
        self._root_splitter.setHandleWidth(4)
        self._root_splitter.setChildrenCollapsible(False)
        root.addWidget(self._root_splitter)

        # ── LEFT SIDEBAR (wrapped in QScrollArea for full vertical scroll) ────
        sidebar_inner = QWidget()
        sidebar_inner.setObjectName("sidebar_inner")
        sidebar_inner.setStyleSheet(
            f"QWidget#sidebar_inner {{ background-color: {BG_PANEL}; }}"
        )
        sb_layout = QVBoxLayout(sidebar_inner)
        sb_layout.setContentsMargins(12, 16, 12, 12)
        sb_layout.setSpacing(10)

        sidebar_scroll = QScrollArea()
        sidebar_scroll.setWidget(sidebar_inner)
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        sidebar_scroll.setMinimumWidth(240)
        sidebar_scroll.setMaximumWidth(420)
        sidebar_scroll.setStyleSheet(
            f"QScrollArea {{ border: none; "
            f"background-color: {BG_PANEL}; "
            f"border-right: 1px solid {BORDER_COLOR}; }}"
        )
        self._sidebar_scroll = sidebar_scroll   # kept for theme updates

        # App title
        title_lbl = QLabel(APP_TITLE.upper())
        title_lbl.setStyleSheet(
            f"color: {ACCENT}; font-size: 13px; font-weight: 700; "
            f"letter-spacing: 2px; padding-bottom: 4px;"
        )
        version_lbl = QLabel(f"v{APP_VERSION}")
        version_lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
        sb_layout.addWidget(title_lbl)
        sb_layout.addWidget(version_lbl)

        # Theme toggle
        theme_row = QHBoxLayout()
        theme_row.setSpacing(6)
        theme_lbl = QLabel("Theme:")
        theme_lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        self._btn_theme_dark  = QPushButton("Dark")
        self._btn_theme_light = QPushButton("Light")
        for btn in (self._btn_theme_dark, self._btn_theme_light):
            btn.setFixedHeight(24)
            btn.setStyleSheet(f"font-size: 11px; padding: 2px 8px;")
        self._btn_theme_dark.clicked.connect(lambda: self._apply_theme(DARK))
        self._btn_theme_light.clicked.connect(lambda: self._apply_theme(LIGHT))
        theme_row.addWidget(theme_lbl)
        theme_row.addWidget(self._btn_theme_dark)
        theme_row.addWidget(self._btn_theme_light)
        theme_row.addStretch()
        sb_layout.addLayout(theme_row)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f"color: {BORDER_COLOR};")
        sb_layout.addWidget(sep)

        # Mode selector
        mode_grp = QGroupBox("Test Mode")
        mode_layout = QVBoxLayout(mode_grp)
        self._mode_combo = QComboBox()
        for mode in TestMode:
            self._mode_combo.addItem(mode.value, mode)
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_layout.addWidget(self._mode_combo)
        sb_layout.addWidget(mode_grp)

        # Connection settings
        conn_grp = QGroupBox("Connection")
        conn_layout = QGridLayout(conn_grp)
        conn_layout.setColumnStretch(1, 1)
        conn_layout.addWidget(QLabel("IP Address:"), 0, 0)
        self._ip_edit = QLineEdit(T7_IP_ADDRESS)
        conn_layout.addWidget(self._ip_edit, 0, 1)
        conn_layout.addWidget(QLabel("Interface:"), 1, 0)
        self._conn_combo = QComboBox()
        self._conn_combo.addItems(["Ethernet", "USB"])
        conn_layout.addWidget(self._conn_combo, 1, 1)
        conn_grp.setLayout(conn_layout)
        sb_layout.addWidget(conn_grp)

        # Sample rate
        rate_grp = QGroupBox("Sample Rate")
        rate_layout = QHBoxLayout(rate_grp)
        self._rate_spin = QSpinBox()
        self._rate_spin.setRange(SAMPLE_RATE_HZ_MIN, SAMPLE_RATE_HZ_MAX)
        self._rate_spin.setValue(SAMPLE_RATE_HZ_DEFAULT)
        self._rate_spin.setSuffix(" Hz")
        self._rate_spin.valueChanged.connect(self._on_rate_changed)
        rate_layout.addWidget(self._rate_spin)
        sb_layout.addWidget(rate_grp)

        # Acquisition filter — taller group with explicit row heights
        filt_grp = QGroupBox("Acq Filter (CSV)")
        filt_layout = QGridLayout(filt_grp)
        filt_layout.setColumnStretch(1, 1)
        filt_layout.setVerticalSpacing(8)
        filt_layout.setContentsMargins(8, 14, 8, 10)

        lbl_profile = QLabel("Profile:")
        lbl_profile.setMinimumHeight(26)
        filt_layout.addWidget(lbl_profile, 0, 0)
        self._filt_combo = QComboBox()
        self._filt_combo.setMinimumHeight(26)
        self._filt_options = [
            ("None",     FilterProfile.NONE,       5),
            ("SMA",      FilterProfile.SMA,        5),
            ("Med+EMA",  FilterProfile.MEDIAN_EMA, 5),
        ]
        for label, _p, _n in self._filt_options:
            self._filt_combo.addItem(label)
        self._filt_combo.setToolTip(
            "Acquisition-layer filter applied before ring buffer.\n"
            "Affects both live display AND CSV log.\n"
            "For display-only smoothing use the card filter dropdown."
        )
        self._filt_combo.currentIndexChanged.connect(self._on_filt_profile_changed)
        filt_layout.addWidget(self._filt_combo, 0, 1)

        lbl_n = QLabel("Window N:")
        lbl_n.setMinimumHeight(26)
        filt_layout.addWidget(lbl_n, 1, 0)
        self._filt_n_spin = QSpinBox()
        self._filt_n_spin.setMinimumHeight(26)
        self._filt_n_spin.setRange(2, 20)
        self._filt_n_spin.setValue(5)
        self._filt_n_spin.setToolTip(
            "Median/SMA window length (samples).\n"
            "At 10 Hz: N=5 → 500 ms window, N=10 → 1 s window.\n"
            "Forced odd internally for clean median."
        )
        filt_layout.addWidget(self._filt_n_spin, 1, 1)

        lbl_ema2 = QLabel("EMA2 \u03b1:")
        lbl_ema2.setMinimumHeight(26)
        filt_layout.addWidget(lbl_ema2, 2, 0)
        self._filt_ema2_spin = QDoubleSpinBox()
        self._filt_ema2_spin.setMinimumHeight(26)
        self._filt_ema2_spin.setRange(0.01, 1.00)
        self._filt_ema2_spin.setSingleStep(0.05)
        self._filt_ema2_spin.setDecimals(2)
        self._filt_ema2_spin.setValue(FILTER_EMA2_ALPHA)
        self._filt_ema2_spin.setToolTip(
            "Second EMA stage alpha (Med+EMA only).\n"
            "Lower = stronger low-pass, slower step response.\n"
            "At 10 Hz: 0.15→fc≈0.26 Hz, 0.10→fc≈0.17 Hz, 0.05→fc≈0.08 Hz\n"
            "Set to 1.0 to disable second stage."
        )
        self._filt_ema2_spin.setEnabled(False)
        self._filt_ema2_spin.valueChanged.connect(self._on_filt_ema2_changed)
        filt_layout.addWidget(self._filt_ema2_spin, 2, 1)

        sb_layout.addWidget(filt_grp)

        # Transport controls
        ctrl_grp = QGroupBox("Acquisition")
        ctrl_layout = QVBoxLayout(ctrl_grp)
        self._btn_start = QPushButton("▶  Start Stream")
        self._btn_start.setObjectName("btn_start")
        self._btn_stop = QPushButton("■  Stop Stream")
        self._btn_stop.setObjectName("btn_stop")
        self._btn_stop.setEnabled(False)
        self._btn_start.clicked.connect(self._start_streaming)
        self._btn_stop.clicked.connect(self._stop_streaming)
        ctrl_layout.addWidget(self._btn_start)
        ctrl_layout.addWidget(self._btn_stop)
        sb_layout.addWidget(ctrl_grp)

        # Recording controls — "Choose" button + shorter path field
        rec_grp = QGroupBox("CSV Recording")
        rec_layout = QVBoxLayout(rec_grp)
        path_row = QHBoxLayout()
        self._path_edit = QLineEdit(CSV_DEFAULT_DIR)
        self._path_edit.setPlaceholderText("Output directory...")
        self._path_edit.setMaximumWidth(140)
        btn_browse = QPushButton("Choose")
        btn_browse.setFixedWidth(58)
        btn_browse.clicked.connect(self._browse_output)
        path_row.addWidget(self._path_edit, stretch=1)
        path_row.addWidget(btn_browse)

        self._btn_record = QPushButton("⏺  Start Recording")
        self._btn_record.setObjectName("btn_record")
        self._btn_record.setEnabled(False)
        self._btn_record.clicked.connect(self._toggle_recording)

        self._rec_status_lbl = QLabel("Not recording")
        self._rec_status_lbl.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 11px;")
        rec_layout.addLayout(path_row)
        rec_layout.addWidget(self._btn_record)
        rec_layout.addWidget(self._rec_status_lbl)
        sb_layout.addWidget(rec_grp)

        # Channel selector — stretch to fill remaining sidebar space
        ch_grp = QGroupBox("Active Channels")
        ch_layout = QVBoxLayout(ch_grp)
        self._channel_selector = ChannelSelectorPanel()
        self._channel_selector.channels_changed.connect(self._on_channels_changed)
        ch_layout.addWidget(self._channel_selector)
        ch_grp.setMinimumHeight(220)
        sb_layout.addWidget(ch_grp, stretch=1)

        sb_layout.addStretch()

        # ── MAIN CONTENT AREA ─────────────────────────────────────────────────
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(12, 12, 12, 8)
        content_layout.setSpacing(8)

        # Value cards row — horizontal scroll so cards never compress
        self._cards_scroll = QScrollArea()
        self._cards_scroll.setWidgetResizable(True)
        self._cards_scroll.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._cards_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self._cards_scroll.setFixedHeight(100)
        self._cards_scroll.setStyleSheet("QScrollArea { border: none; }")

        self._cards_widget = QWidget()
        self._cards_layout = QHBoxLayout(self._cards_widget)
        self._cards_layout.setContentsMargins(0, 0, 0, 0)
        self._cards_layout.setSpacing(8)
        self._cards_layout.setSizeConstraint(
            QHBoxLayout.SizeConstraint.SetMinimumSize)
        self._value_cards: Dict[str, ChannelValueCard] = {}
        self._cards_scroll.setWidget(self._cards_widget)
        content_layout.addWidget(self._cards_scroll)

        # Tabs: Time Series | Cross Plot
        self._tabs = QTabWidget()

        ts_tab = QWidget()
        ts_layout = QVBoxLayout(ts_tab)
        ts_layout.setContentsMargins(4, 4, 4, 4)
        self._multi_plot = MultiChannelPlot()
        ts_layout.addWidget(self._multi_plot)
        self._tabs.addTab(ts_tab, "Time Series")

        xp_tab = QWidget()
        xp_layout = QVBoxLayout(xp_tab)
        xp_layout.setContentsMargins(4, 4, 4, 4)
        self._cross_plot = CrossPlotWidget()
        xp_layout.addWidget(self._cross_plot)
        self._tabs.addTab(xp_tab, "Cross Plot")

        content_layout.addWidget(self._tabs, stretch=1)

        # Assemble splitter: sidebar_scroll | content
        self._root_splitter.addWidget(sidebar_scroll)
        self._root_splitter.addWidget(content)
        self._root_splitter.setStretchFactor(0, 0)   # sidebar doesn't stretch
        self._root_splitter.setStretchFactor(1, 1)   # content takes all extra space
        self._root_splitter.setSizes([280, 1200])

        # Status bar
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_lbl = QLabel("Ready")
        self._status_lbl.setObjectName("status_connected")
        self._frames_lbl = QLabel("Frames: 0")
        self._frames_lbl.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 11px;")
        self._rate_lbl = QLabel("")
        self._rate_lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        self._status_bar.addWidget(self._status_lbl)
        self._status_bar.addPermanentWidget(self._rate_lbl)
        self._status_bar.addPermanentWidget(self._frames_lbl)

    # --------------------------------------------------------------------------
    # Mode Management
    # --------------------------------------------------------------------------
    def _load_mode(self, mode: TestMode):
        """Configure UI for the given test mode."""
        self._mode = mode
        channels = MODE_CHANNELS[mode]

        # Assign colors — use theme-appropriate cycle
        cycle = COLOR_CYCLE_LIGHT if _CURRENT_THEME is LIGHT else COLOR_CYCLE
        self._channel_colors.clear()
        color_idx = 0
        for ch in channels:
            if not ch.is_timestamp:
                if ch.color:
                    self._channel_colors[ch.key] = ch.color
                else:
                    self._channel_colors[ch.key] = cycle[
                        color_idx % len(cycle)
                    ]
                    color_idx += 1

        self._all_channels = channels

        # Rebuild value cards
        for card in self._value_cards.values():
            card.deleteLater()
        self._value_cards.clear()
        while self._cards_layout.count():
            item = self._cards_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        data_channels = [ch for ch in channels if not ch.is_timestamp]
        for ch in data_channels:
            if ch.enabled_by_default:
                color = self._channel_colors.get(ch.key, ACCENT)
                card = ChannelValueCard(ch, color)
                self._value_cards[ch.key] = card
                self._cards_layout.addWidget(card)
        self._cards_layout.addStretch()

        # Update channel selector
        self._channel_selector.load_channels(channels, self._channel_colors)
        self._active_channels = self._channel_selector.get_enabled_channels()

        # Configure plots
        self._multi_plot.configure_channels(
            self._active_channels, self._channel_colors
        )
        self._multi_plot.clear_data()
        self._cross_plot.configure(self._active_channels)
        self._cross_plot.clear_data()

    def _on_mode_changed(self, index: int):
        if self._is_streaming:
            QMessageBox.warning(
                self, "Mode Change",
                "Stop the active stream before changing test modes."
            )
            # Revert combo
            for i in range(self._mode_combo.count()):
                if self._mode_combo.itemData(i) == self._mode:
                    self._mode_combo.setCurrentIndex(i)
                    break
            return
        new_mode = self._mode_combo.itemData(index)
        self._load_mode(new_mode)

    def _on_channels_changed(self, enabled: List[ChannelConfig]):
        """Called when user toggles a channel checkbox."""
        self._active_channels = enabled

        # Rebuild value cards for newly enabled/disabled channels
        for ch in [c for c in self._all_channels if not c.is_timestamp]:
            is_enabled = any(e.key == ch.key for e in enabled)
            if is_enabled and ch.key not in self._value_cards:
                color = self._channel_colors.get(ch.key, ACCENT)
                card = ChannelValueCard(ch, color)
                self._value_cards[ch.key] = card
                self._cards_layout.insertWidget(
                    self._cards_layout.count() - 1, card
                )
            elif not is_enabled and ch.key in self._value_cards:
                self._value_cards[ch.key].deleteLater()
                del self._value_cards[ch.key]

        # Reconfigure plots with new channel set
        self._multi_plot.configure_channels(enabled, self._channel_colors)
        self._cross_plot.configure(enabled)

    # --------------------------------------------------------------------------
    # Streaming Control
    # --------------------------------------------------------------------------
    def _start_streaming(self):
        if self._is_streaming:
            return

        # Validate IP
        ip = self._ip_edit.text().strip()
        if not ip:
            QMessageBox.warning(self, "Configuration",
                                "Please enter the T7 IP address.")
            return

        # Ensure we have active channels
        if not self._active_channels:
            QMessageBox.warning(self, "Configuration",
                                "Please enable at least one channel.")
            return

        # Pass all channels for the mode — acquisition.py handles timestamp
        # internally and filters is_timestamp channels from sensor reads.
        acq_channels = self._all_channels

        self._ring_buffer = RingBuffer()
        self._acq_thread = AcquisitionThread(
            mode=self._mode,
            active_channels=acq_channels,
            sample_rate_hz=self._sample_rate,
            buffer=self._ring_buffer,
            status_callback=self._status_signal_emit,
            ip_address=ip,
            connection_type=self._conn_combo.currentText(),
            filter_profile=self._acq_filter_profile,
            filter_n=self._filt_n_spin.value(),
            filter_ema2_alpha=self._filt_ema2_spin.value(),
        )
        self._acq_thread.start()

        self._is_streaming = True
        self._session_start = time.time()
        self._frame_count = 0
        self._set_streaming_ui(True)
        self._status_bar.showMessage(f"Connecting to T7 at {ip}...")

    def _stop_streaming(self):
        if not self._is_streaming:
            return
        if self._is_recording:
            self._stop_recording()
        if self._acq_thread:
            self._acq_thread.stop()
            self._acq_thread.join(timeout=3.0)
            self._acq_thread = None

        self._is_streaming = False
        self._set_streaming_ui(False)
        self._status_lbl.setText("Stopped")
        self._status_lbl.setStyleSheet(f"color: {WARNING}; font-weight: 600;")
        self._status_bar.showMessage("Stream stopped")

        # Clear any per-channel display zeros so the next stream starts clean
        for card in self._value_cards.values():
            card.reset_zero()

    def _status_signal_emit(self, status: AcqStatus, msg: str):
        """Called from acquisition thread — emit Qt signal to cross to UI thread."""
        self._acq_status_signal.emit(status, msg)

    def _on_acq_status(self, status: AcqStatus, msg: str):
        """Receive status updates on the UI thread."""
        color_map = {
            AcqStatus.DISCONNECTED: TEXT_MUTED,
            AcqStatus.CONNECTING:   WARNING,
            AcqStatus.CONNECTED:    SUCCESS,
            AcqStatus.STREAMING:    ACCENT,
            AcqStatus.ERROR:        DANGER,
            AcqStatus.STOPPED:      TEXT_MUTED,
        }
        color = color_map.get(status, TEXT_MUTED)
        self._status_lbl.setText(status.value)
        self._status_lbl.setStyleSheet(
            f"color: {color}; font-weight: 600;"
        )
        # Persist message in status bar — errors stay until next action
        # (no timeout on errors so they are not missed)
        if status == AcqStatus.ERROR:
            self._status_bar.showMessage(f"Error: {msg}")
            # If streaming was active, reset button states
            if self._is_streaming:
                self._is_streaming = False
                self._set_streaming_ui(False)
        elif status == AcqStatus.DISCONNECTED and self._is_streaming:
            # Unexpected disconnect — reset UI
            self._is_streaming = False
            self._set_streaming_ui(False)
            self._status_bar.showMessage(
                f"Disconnected: {msg} — check T7 connection and retry")
        else:
            self._status_bar.showMessage(msg, 6000)

    def _set_streaming_ui(self, streaming: bool):
        """
        Update all button states and visual feedback for streaming/stopped state.
        streaming=True  → Start button filled/active, Stop enabled, inputs locked.
        streaming=False → Start button restored, Stop disabled, inputs unlocked.
        """
        if streaming:
            # Start button — filled active background
            self._btn_start.setObjectName("btn_start_active")
            self._btn_start.setEnabled(False)
            # Stop button — armed and ready
            self._btn_stop.setObjectName("btn_stop")
            self._btn_stop.setEnabled(True)
            self._btn_record.setEnabled(True)
            self._mode_combo.setEnabled(False)
            self._ip_edit.setEnabled(False)
            self._conn_combo.setEnabled(False)
        else:
            # Start button — back to idle outline style
            self._btn_start.setObjectName("btn_start")
            self._btn_start.setEnabled(True)
            # Stop button — back to idle
            self._btn_stop.setObjectName("btn_stop")
            self._btn_stop.setEnabled(False)
            self._btn_record.setEnabled(False)
            self._mode_combo.setEnabled(True)
            self._ip_edit.setEnabled(True)
            self._conn_combo.setEnabled(True)
        # Force Qt to re-evaluate the objectName stylesheet selectors
        self._btn_start.style().unpolish(self._btn_start)
        self._btn_start.style().polish(self._btn_start)
        self._btn_stop.style().unpolish(self._btn_stop)
        self._btn_stop.style().polish(self._btn_stop)

    def _set_recording_ui(self, recording: bool):
        """Update record button visual state."""
        if recording:
            self._btn_record.setText("⏹  Stop Recording")
            self._btn_record.setObjectName("btn_record_active")
        else:
            self._btn_record.setText("⏺  Start Recording")
            self._btn_record.setObjectName("btn_record")
        self._btn_record.style().unpolish(self._btn_record)
        self._btn_record.style().polish(self._btn_record)

    def _apply_theme(self, theme_dict: dict):
        """Switch between dark and light themes at runtime."""
        ss = apply_theme(theme_dict)
        QApplication.instance().setStyleSheet(ss)

        bg  = theme_dict["BG_PANEL"]
        bdr = theme_dict["BORDER_COLOR"]

        # Reassign channel colors using the theme-appropriate cycle, then
        # recolor cards and plot curves to match.
        cycle = COLOR_CYCLE_LIGHT if theme_dict is LIGHT else COLOR_CYCLE
        color_idx = 0
        for ch in [c for c in self._all_channels if not c.is_timestamp]:
            if not ch.color:
                self._channel_colors[ch.key] = cycle[
                    color_idx % len(cycle)
                ]
                color_idx += 1

        # Sidebar scroll background + border
        self._sidebar_scroll.setStyleSheet(
            f"QScrollArea {{ border: none; "
            f"background-color: {bg}; "
            f"border-right: 1px solid {bdr}; }}"
        )
        self._sidebar_scroll.widget().setStyleSheet(
            f"QWidget#sidebar_inner {{ background-color: {bg}; }}"
        )

        # Splitter handle colour
        self._root_splitter.setStyleSheet(
            f"QSplitter::handle {{ background-color: {bdr}; }}"
        )

        # Recolor all live value cards with updated colors
        for key, card in self._value_cards.items():
            color = self._channel_colors.get(key, theme_dict["ACCENT"])
            card.refresh_theme(theme_dict, color)

        # Recolor plot curves with updated colors
        self._multi_plot.configure_channels(
            self._active_channels, self._channel_colors
        )
        self._cross_plot.configure(self._active_channels)

        # Update pyqtgraph plot backgrounds
        plot_bg    = theme_dict["PLOT_BG"]
        text_muted = theme_dict["TEXT_MUTED"]
        grid_color = theme_dict["GRID_COLOR"]
        accent     = theme_dict["ACCENT"]

        for plot_widget in [self._multi_plot._plot_widget,
                            self._cross_plot._plot]:
            plot_widget.setBackground(plot_bg)
            for axis_name in ["left", "bottom"]:
                ax = plot_widget.getAxis(axis_name)
                ax.setPen(pg.mkPen(grid_color, width=1))
                ax.setTextPen(pg.mkPen(text_muted))

        self._cross_plot._scatter.setBrush(pg.mkBrush(accent + "99"))

        try:
            self._multi_plot._legend.setBrush(
                pg.mkBrush(theme_dict["BG_PANEL"] + "CC"))
            self._multi_plot._legend.setPen(pg.mkPen(bdr))
        except Exception:
            pass

        # Highlight active theme button
        dark_active = theme_dict is DARK
        self._btn_theme_dark.setStyleSheet(
            f"font-size:11px; padding:2px 8px; "
            f"font-weight:{'700' if dark_active else '400'}; "
            f"border: {'2' if dark_active else '1'}px solid {accent};"
        )
        self._btn_theme_light.setStyleSheet(
            f"font-size:11px; padding:2px 8px; "
            f"font-weight:{'700' if not dark_active else '400'}; "
            f"border: {'2' if not dark_active else '1'}px solid {accent};"
        )

    def _on_rate_changed(self, value: int):
        self._sample_rate = value

    def _on_filt_ema2_changed(self, value: float):
        """
        Called when the EMA2 α spinbox changes.
        Pushes the new alpha to every live value card so the display-layer
        filter updates immediately without requiring a stream restart.
        The acquisition-layer filter is unaffected — it is locked to the
        parameters captured at stream start and only updates on next start.
        """
        self._acq_filter_ema2_alpha = value
        for card in self._value_cards.values():
            card.set_ema2_alpha(value)

    def _on_filt_profile_changed(self, index: int):
        """Update acquisition filter state from sidebar combo selection."""
        _label, profile, default_n = self._filt_options[index]
        self._acq_filter_profile = profile
        self._filt_n_spin.setValue(default_n)
        # N spinbox only relevant when a windowed filter is active
        self._filt_n_spin.setEnabled(profile != FilterProfile.NONE)
        # EMA2 spinbox only relevant for Med+EMA cascade
        self._filt_ema2_spin.setEnabled(profile == FilterProfile.MEDIAN_EMA)

    # --------------------------------------------------------------------------
    # Recording
    # --------------------------------------------------------------------------
    def _browse_output(self):
        d = QFileDialog.getExistingDirectory(
            self, "Select Output Directory", self._path_edit.text()
        )
        if d:
            self._path_edit.setText(d)

    def _toggle_recording(self):
        if self._is_recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        out_dir = self._path_edit.text().strip()
        if not out_dir:
            QMessageBox.warning(self, "Recording",
                                "Please set an output directory.")
            return

        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        mode_slug = self._mode.name.lower()
        filename = os.path.join(out_dir, f"{mode_slug}_{ts}.csv")

        self._csv_logger.open(
            path=filename,
            channels=self._active_channels,
            mode=self._mode,
            session_meta={
                "mode": self._mode.value,
                "sample_rate_hz": str(self._sample_rate),
                "ip_address": self._ip_edit.text(),
            },
        )
        self._is_recording = True
        self._set_recording_ui(True)
        self._rec_status_lbl.setText(
            f"● REC  {os.path.basename(filename)}"
        )
        self._rec_status_lbl.setStyleSheet(
            f"color: {DANGER}; font-size: 11px; font-weight: 700;")

    def _stop_recording(self):
        frames = self._csv_logger.frame_count
        self._csv_logger.close()
        self._is_recording = False
        self._set_recording_ui(False)
        self._rec_status_lbl.setText(f"Saved {frames} frames")
        self._rec_status_lbl.setStyleSheet(
            f"color: {SUCCESS}; font-size: 11px;")

    # --------------------------------------------------------------------------
    # Plot Refresh (runs on timer, UI thread only)
    # --------------------------------------------------------------------------
    def _refresh_plots(self):
        if not self._is_streaming:
            return

        frames = self._ring_buffer.drain()
        if not frames:
            return

        self._frame_count += len(frames)

        for frame in frames:
            # Pass each acquisition-filtered value through the card's display
            # filter and zero offset.  update_value() returns the final display
            # value (display-filtered, zero-offset-applied) so we can build the
            # plot frame from it without a separate offset snapshot dict.
            #
            # Cards without a value in this frame (channel not active in this
            # mode) are skipped; their plot channel just gets no new point.
            display_values: Dict[str, float] = {}
            for key, card in self._value_cards.items():
                if key in frame.values:
                    display_values[key] = card.update_value(frame.values[key])

            # Plot frame uses display-filtered, zero-offset values so the
            # plot stays in sync with what the cards are showing.
            plot_frame = DataFrame(
                wall_time       = frame.wall_time,
                t7_timestamp_ms = frame.t7_timestamp_ms,
                values          = {
                    # Start with all frame values (covers channels without cards,
                    # e.g. channels active in acq but not currently shown)
                    **frame.values,
                    # Overwrite with display-layer values for carded channels
                    **display_values,
                },
            )

            self._multi_plot.push_frame(plot_frame)
            self._cross_plot.push_frame(plot_frame)

            # CSV logger always receives the acquisition-filtered frame —
            # display-layer filter and zero offset are never written to disk.
            if self._is_recording:
                self._csv_logger.write_frame(frame)

        # Redraw
        self._multi_plot.refresh()
        if self._tabs.currentIndex() == 1:
            self._cross_plot.refresh()

    def _update_status_bar(self):
        if self._is_streaming:
            elapsed = time.time() - (self._session_start or time.time())
            elapsed_str = str(datetime.timedelta(seconds=int(elapsed)))
            self._frames_lbl.setText(
                f"Frames: {self._frame_count:,}  |  Elapsed: {elapsed_str}"
            )
            if self._is_recording:
                self._rate_lbl.setText(
                    f"● REC  {self._csv_logger.frame_count:,} rows logged"
                )
                self._rate_lbl.setStyleSheet(
                    f"color: {DANGER}; font-size: 11px; font-weight: 700;")
            else:
                self._rate_lbl.setText("")

    # --------------------------------------------------------------------------
    # Cleanup on close
    # --------------------------------------------------------------------------
    def closeEvent(self, event):
        self._plot_timer.stop()
        self._status_timer.stop()
        if self._is_recording:
            self._csv_logger.close()
        if self._acq_thread is not None:
            try:
                self._acq_thread.stop()
                if self._acq_thread.is_alive():
                    self._acq_thread.join(timeout=2.0)
            except Exception:
                pass
            self._acq_thread = None
        event.accept()


# ==============================================================================
# ENTRY POINT
# ==============================================================================
def main():
    app = QApplication(sys.argv)
    app.setApplicationName(APP_TITLE)
    app.setStyleSheet(STYLESHEET)

    # Use Fusion style as the base (best for custom stylesheets on Windows)
    app.setStyle("Fusion")

    window = MainWindow()
    window.show()
    # Apply initial theme to highlight the active theme button
    window._apply_theme(DARK)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
