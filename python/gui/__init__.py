"""Stark SDK GUI Package - Modern Control Interface"""

from .styles import DARK_THEME, COLORS
from .i18n import tr, get_i18n
from .main_window import MainWindow
from .connection_panel import ConnectionPanel

__all__ = [
    "DARK_THEME",
    "COLORS", 
    "tr",
    "get_i18n",
    "MainWindow",
    "ConnectionPanel",
]
