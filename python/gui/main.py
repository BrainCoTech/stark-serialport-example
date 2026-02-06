#!/usr/bin/env python3
"""
Stark SDK GUI - Modern Control Interface
Supports all protocols and device types
"""

import signal
import sys
import warnings
from pathlib import Path

# Suppress pyqtgraph disconnect warnings (PySide6 compatibility issue)
warnings.filterwarnings("ignore", message="Failed to disconnect.*", category=RuntimeWarning)

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QPalette, QColor

from gui.main_window import MainWindow
from gui.styles import DARK_THEME


def main():
    """Main entry point"""
    # Enable high DPI scaling
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )
    
    app = QApplication(sys.argv)
    app.setApplicationName("Stark SDK")
    app.setOrganizationName("BrainCo")
    app.setApplicationVersion("1.0.7")
    
    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    # Dark theme has compatibility issues on macOS, use system default
    # app.setStyleSheet(DARK_THEME)
    
    # Create and show main window
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
