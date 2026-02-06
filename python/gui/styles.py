"""Modern Dark Theme Styles for Stark SDK GUI"""

# Color palette
COLORS = {
    # Primary colors
    "primary": "#3498db",
    "primary_hover": "#2980b9",
    "primary_pressed": "#1f6dad",
    
    # Accent colors
    "accent": "#2ecc71",
    "accent_hover": "#27ae60",
    "warning": "#f39c12",
    "danger": "#e74c3c",
    "danger_hover": "#c0392b",
    
    # Background colors
    "bg_dark": "#1a1a2e",
    "bg_medium": "#16213e",
    "bg_light": "#0f3460",
    "bg_card": "#3d4a5c",       # Lighter card background for better contrast
    "bg_input": "#1e2633",      # Darker input background
    "bg_dropdown": "#2d3748",   # Dropdown popup background
    
    # Text colors
    "text_primary": "#ecf0f1",
    "text_secondary": "#bdc3c7",
    "text_muted": "#7f8c8d",
    
    # Border colors
    "border": "#34495e",
    "border_light": "#4a6785",
    
    # Status colors
    "success": "#2ecc71",
    "info": "#3498db",
    "error": "#e74c3c",
    "offline": "#95a5a6",
}

# Main application stylesheet
DARK_THEME = f"""
/* Main Window */
QMainWindow {{
    background-color: {COLORS['bg_dark']};
}}

QWidget {{
    background-color: transparent;
    color: {COLORS['text_primary']};
    font-size: 13px;
}}

/* Menu Bar */
QMenuBar {{
    background-color: {COLORS['bg_medium']};
    color: {COLORS['text_primary']};
    padding: 4px 8px;
    border-bottom: 1px solid {COLORS['border']};
}}

QMenuBar::item {{
    padding: 6px 12px;
    border-radius: 4px;
}}

QMenuBar::item:selected {{
    background-color: {COLORS['bg_light']};
}}

QMenu {{
    background-color: {COLORS['bg_card']};
    border: 1px solid {COLORS['border']};
    border-radius: 8px;
    padding: 4px;
}}

QMenu::item {{
    padding: 8px 24px;
    border-radius: 4px;
}}

QMenu::item:selected {{
    background-color: {COLORS['primary']};
}}

/* Status Bar */
QStatusBar {{
    background-color: {COLORS['bg_medium']};
    color: {COLORS['text_secondary']};
    border-top: 1px solid {COLORS['border']};
    padding: 4px 12px;
}}

/* Group Box */
QGroupBox {{
    background-color: {COLORS['bg_card']};
    border: 1px solid {COLORS['border']};
    border-radius: 12px;
    margin-top: 16px;
    padding: 16px;
    font-weight: 600;
    color: {COLORS['text_primary']};
}}

QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 16px;
    padding: 0 8px;
    color: {COLORS['text_primary']};
    background-color: {COLORS['bg_card']};
}}

QGroupBox QLabel {{
    color: {COLORS['text_primary']};
    background-color: transparent;
}}

/* Tab Widget */
QTabWidget::pane {{
    background-color: {COLORS['bg_card']};
    border: 1px solid {COLORS['border']};
    border-radius: 12px;
    padding: 8px;
}}

QTabBar::tab {{
    background-color: {COLORS['bg_medium']};
    color: {COLORS['text_secondary']};
    padding: 10px 20px;
    margin-right: 4px;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    border: 1px solid {COLORS['border']};
    border-bottom: none;
}}

QTabBar::tab:selected {{
    background-color: {COLORS['bg_card']};
    color: {COLORS['text_primary']};
    border-bottom: 2px solid {COLORS['primary']};
}}

QTabBar::tab:hover:!selected {{
    background-color: {COLORS['bg_light']};
}}

/* Push Button */
QPushButton {{
    background-color: {COLORS['primary']};
    color: white;
    border: none;
    border-radius: 8px;
    padding: 10px 24px;
    font-weight: 600;
    min-width: 100px;
}}

QPushButton:hover {{
    background-color: {COLORS['primary_hover']};
}}

QPushButton:pressed {{
    background-color: {COLORS['primary_pressed']};
}}

QPushButton:disabled {{
    background-color: {COLORS['bg_light']};
    color: {COLORS['text_muted']};
}}

QPushButton[class="danger"] {{
    background-color: {COLORS['danger']};
}}

QPushButton[class="danger"]:hover {{
    background-color: {COLORS['danger_hover']};
}}

QPushButton[class="success"] {{
    background-color: {COLORS['accent']};
}}

QPushButton[class="success"]:hover {{
    background-color: {COLORS['accent_hover']};
}}

QPushButton[class="secondary"] {{
    background-color: {COLORS['bg_light']};
    color: {COLORS['text_primary']};
}}

QPushButton[class="secondary"]:hover {{
    background-color: {COLORS['border_light']};
}}

/* Line Edit */
QLineEdit {{
    background-color: {COLORS['bg_input']};
    color: {COLORS['text_primary']};
    border: 1px solid {COLORS['border']};
    border-radius: 8px;
    padding: 10px 12px;
    selection-background-color: {COLORS['primary']};
}}

QLineEdit:focus {{
    border-color: {COLORS['primary']};
}}

QLineEdit:disabled {{
    background-color: {COLORS['bg_medium']};
    color: {COLORS['text_muted']};
}}

/* Combo Box */
QComboBox {{
    background-color: {COLORS['bg_card']};
    color: {COLORS['text_primary']};
    border: 1px solid {COLORS['border']};
    border-radius: 8px;
    padding: 10px 12px;
    min-width: 150px;
}}

QComboBox:hover {{
    border-color: {COLORS['border_light']};
}}

QComboBox:focus {{
    border-color: {COLORS['primary']};
}}

QComboBox::drop-down {{
    border: none;
    width: 30px;
}}

QComboBox::down-arrow {{
    image: none;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 6px solid {COLORS['text_secondary']};
    margin-right: 10px;
}}

QComboBox QAbstractItemView {{
    background-color: {COLORS['bg_card']};
    color: {COLORS['text_primary']};
    border: 1px solid {COLORS['border_light']};
    border-radius: 8px;
    selection-background-color: {COLORS['primary']};
    selection-color: white;
    outline: none;
    padding: 4px;
}}

QComboBox QAbstractItemView::item {{
    color: {COLORS['text_primary']};
    background-color: {COLORS['bg_card']};
    padding: 8px 12px;
    min-height: 24px;
}}

QComboBox QAbstractItemView::item:selected {{
    background-color: {COLORS['primary']};
    color: white;
}}

QComboBox QAbstractItemView::item:hover {{
    background-color: {COLORS['bg_light']};
    color: {COLORS['text_primary']};
}}

/* Spin Box */
QSpinBox, QDoubleSpinBox {{
    background-color: {COLORS['bg_input']};
    color: {COLORS['text_primary']};
    border: 1px solid {COLORS['border']};
    border-radius: 8px;
    padding: 10px 12px;
}}

QSpinBox:focus, QDoubleSpinBox:focus {{
    border-color: {COLORS['primary']};
}}

QSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
    background-color: {COLORS['bg_light']};
    border: none;
    width: 20px;
}}

QSpinBox::up-button:hover, QSpinBox::down-button:hover,
QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {{
    background-color: {COLORS['border_light']};
}}

/* Slider */
QSlider::groove:horizontal {{
    background-color: {COLORS['bg_light']};
    height: 8px;
    border-radius: 4px;
}}

QSlider::handle:horizontal {{
    background-color: {COLORS['primary']};
    width: 20px;
    height: 20px;
    margin: -6px 0;
    border-radius: 10px;
}}

QSlider::handle:horizontal:hover {{
    background-color: {COLORS['primary_hover']};
}}

QSlider::sub-page:horizontal {{
    background-color: {COLORS['primary']};
    border-radius: 4px;
}}

/* Progress Bar */
QProgressBar {{
    background-color: {COLORS['bg_light']};
    border: none;
    border-radius: 6px;
    height: 12px;
    text-align: center;
    color: {COLORS['text_primary']};
}}

QProgressBar::chunk {{
    background-color: {COLORS['primary']};
    border-radius: 6px;
}}

/* Scroll Bar */
QScrollBar:vertical {{
    background-color: {COLORS['bg_medium']};
    width: 12px;
    border-radius: 6px;
    margin: 0;
}}

QScrollBar::handle:vertical {{
    background-color: {COLORS['border']};
    border-radius: 6px;
    min-height: 30px;
}}

QScrollBar::handle:vertical:hover {{
    background-color: {COLORS['border_light']};
}}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}

QScrollBar:horizontal {{
    background-color: {COLORS['bg_medium']};
    height: 12px;
    border-radius: 6px;
    margin: 0;
}}

QScrollBar::handle:horizontal {{
    background-color: {COLORS['border']};
    border-radius: 6px;
    min-width: 30px;
}}

QScrollBar::handle:horizontal:hover {{
    background-color: {COLORS['border_light']};
}}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0;
}}

/* Check Box */
QCheckBox {{
    spacing: 8px;
    color: {COLORS['text_primary']};
}}

QCheckBox::indicator {{
    width: 20px;
    height: 20px;
    border-radius: 4px;
    border: 2px solid {COLORS['border']};
    background-color: {COLORS['bg_input']};
}}

QCheckBox::indicator:checked {{
    background-color: {COLORS['primary']};
    border-color: {COLORS['primary']};
}}

QCheckBox::indicator:hover {{
    border-color: {COLORS['primary']};
}}

/* Radio Button */
QRadioButton {{
    spacing: 8px;
    color: {COLORS['text_primary']};
}}

QRadioButton::indicator {{
    width: 20px;
    height: 20px;
    border-radius: 10px;
    border: 2px solid {COLORS['border']};
    background-color: {COLORS['bg_input']};
}}

QRadioButton::indicator:checked {{
    background-color: {COLORS['primary']};
    border-color: {COLORS['primary']};
}}

QRadioButton::indicator:hover {{
    border-color: {COLORS['primary']};
}}

/* Label */
QLabel {{
    color: {COLORS['text_primary']};
    background-color: transparent;
}}

/* Form Layout Labels */
QFormLayout QLabel {{
    color: {COLORS['text_primary']};
}}

QLabel[class="title"] {{
    font-size: 18px;
    font-weight: 700;
    color: {COLORS['text_primary']};
}}

QLabel[class="subtitle"] {{
    font-size: 14px;
    color: {COLORS['text_secondary']};
}}

QLabel[class="muted"] {{
    color: {COLORS['text_muted']};
}}

QLabel[class="success"] {{
    color: {COLORS['success']};
}}

QLabel[class="error"] {{
    color: {COLORS['error']};
}}

QLabel[class="warning"] {{
    color: {COLORS['warning']};
}}

/* Text Edit / Plain Text Edit */
QTextEdit, QPlainTextEdit {{
    background-color: {COLORS['bg_input']};
    color: {COLORS['text_primary']};
    border: 1px solid {COLORS['border']};
    border-radius: 8px;
    padding: 8px;
    selection-background-color: {COLORS['primary']};
}}

QTextEdit:focus, QPlainTextEdit:focus {{
    border-color: {COLORS['primary']};
}}

/* Table Widget */
QTableWidget {{
    background-color: {COLORS['bg_card']};
    alternate-background-color: {COLORS['bg_medium']};
    border: 1px solid {COLORS['border']};
    border-radius: 8px;
    gridline-color: {COLORS['border']};
}}

QTableWidget::item {{
    padding: 8px;
    color: {COLORS['text_primary']};
}}

QTableWidget::item:selected {{
    background-color: {COLORS['primary']};
}}

QHeaderView::section {{
    background-color: {COLORS['bg_medium']};
    color: {COLORS['text_primary']};
    padding: 10px;
    border: none;
    border-bottom: 1px solid {COLORS['border']};
    font-weight: 600;
}}

/* Tool Tip */
QToolTip {{
    background-color: {COLORS['bg_card']};
    color: {COLORS['text_primary']};
    border: 1px solid {COLORS['border']};
    border-radius: 6px;
    padding: 8px;
}}

/* Message Box */
QMessageBox {{
    background-color: {COLORS['bg_card']};
}}

QMessageBox QLabel {{
    color: {COLORS['text_primary']};
}}

/* Splitter */
QSplitter::handle {{
    background-color: {COLORS['border']};
}}

QSplitter::handle:hover {{
    background-color: {COLORS['primary']};
}}
"""

# Connection status indicator styles
CONNECTION_STATUS_STYLES = {
    "connected": f"""
        QLabel {{
            background-color: {COLORS['success']};
            color: white;
            padding: 6px 12px;
            border-radius: 12px;
            font-weight: 600;
        }}
    """,
    "disconnected": f"""
        QLabel {{
            background-color: {COLORS['offline']};
            color: white;
            padding: 6px 12px;
            border-radius: 12px;
            font-weight: 600;
        }}
    """,
    "connecting": f"""
        QLabel {{
            background-color: {COLORS['warning']};
            color: white;
            padding: 6px 12px;
            border-radius: 12px;
            font-weight: 600;
        }}
    """,
    "error": f"""
        QLabel {{
            background-color: {COLORS['error']};
            color: white;
            padding: 6px 12px;
            border-radius: 12px;
            font-weight: 600;
        }}
    """,
}

# Card style for panels
CARD_STYLE = f"""
    background-color: {COLORS['bg_card']};
    border: 1px solid {COLORS['border']};
    border-radius: 12px;
    padding: 16px;
"""
