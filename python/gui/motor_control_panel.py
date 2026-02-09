"""Motor Control Panel - Position/Speed/Current modes"""

import asyncio
import sys
from pathlib import Path
from typing import Optional, TYPE_CHECKING
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QSlider, QSpinBox, QPushButton, QLabel, QComboBox, QGridLayout,
    QProgressBar, QFrame, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer

from .i18n import tr
from .styles import COLORS
from .constants import (
    FINGER_IDS, MOTOR_NAMES_EN, MOTOR_COUNT, MOTOR_BUFFER_SIZE,
    format_motor_state
)

# Add parent directory to path for SDK import
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import sdk, is_protobuf_device

if TYPE_CHECKING:
    from .shared_data import SharedDataManager


def run_async(coro):
    """Run async coroutine in a new event loop (for Qt callbacks)"""
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


# Control modes
MODE_POSITION = 0
MODE_SPEED = 1
MODE_CURRENT = 2


class StatusProgressBar(QWidget):
    """Progress bar with label for status display
    
    Supports both unidirectional (0-1000) and bidirectional (-1000 to 1000) modes.
    """
    
    def __init__(self, label_text, color, bidirectional=False):
        super().__init__()
        self.bidirectional = bidirectional
        self.color = color
        self._value = 0
        
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.setLayout(layout)
        
        # Label
        self.label = QLabel(label_text)
        self.label.setFixedWidth(28)
        self.label.setStyleSheet("font-size: 11px;")
        layout.addWidget(self.label)
        
        # Progress bar
        self.progress = QProgressBar()
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(4)  # Slightly thicker for visibility
        self.progress.setMinimumWidth(120)
        
        if bidirectional:
            self.progress.setRange(0, 2000)  # Map -1000~1000 to 0~2000
            self.progress.setValue(1000)  # Center (0)
        else:
            self.progress.setRange(0, 1000)
            self.progress.setValue(0)
        
        self._update_style()
        layout.addWidget(self.progress, 1)
        
        # Value label
        self.value_label = QLabel("--")
        self.value_label.setFixedWidth(45)
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value_label.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: bold;")
        layout.addWidget(self.value_label)
    
    def _update_style(self):
        """Update progress bar style based on mode"""
        if self.bidirectional:
            # Bidirectional: different colors for positive/negative
            if self._value >= 0:
                chunk_color = self.color
            else:
                chunk_color = COLORS.get('error', '#e74c3c')
            
            self.progress.setStyleSheet(f"""
                QProgressBar {{
                    border: none;
                    background-color: #3a3a3a;
                }}
                QProgressBar::chunk {{
                    background-color: {chunk_color};
                }}
            """)
        else:
            self.progress.setStyleSheet(f"""
                QProgressBar {{
                    border: none;
                    background-color: #3a3a3a;
                }}
                QProgressBar::chunk {{
                    background-color: {self.color};
                }}
            """)
    
    def setValue(self, value):
        """Set value and update display"""
        self._value = value
        self.value_label.setText(str(value))
        
        if self.bidirectional:
            # Map -1000~1000 to 0~2000
            mapped = value + 1000
            self.progress.setValue(max(0, min(2000, mapped)))
        else:
            self.progress.setValue(max(0, min(1000, value)))
        
        self._update_style()


class FingerControl(QGroupBox):
    """Single finger control with position/speed/current modes
    
    Uses getter functions to access device and slave_id from parent panel,
    ensuring it always uses the latest values from SharedDataManager.
    """
    
    def __init__(self, finger_id, mode_getter, device_getter, slave_id_getter, shared_data_getter):
        super().__init__()
        self.finger_id = finger_id
        self.finger_name = MOTOR_NAMES_EN[finger_id]
        self.mode_getter = mode_getter  # Function to get current mode
        self.device_getter = device_getter  # Function to get device
        self.slave_id_getter = slave_id_getter  # Function to get slave_id
        self.shared_data_getter = shared_data_getter  # Function to get shared_data (for hw_type)
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup UI"""
        layout = QVBoxLayout()
        layout.setSpacing(8)
        layout.setContentsMargins(12, 16, 12, 12)
        self.setLayout(layout)
        self.setTitle(self.finger_name)
        # Remove fixed minimum width to allow flexible sizing
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Control slider/spinbox row
        control_layout = QHBoxLayout()
        control_layout.setSpacing(8)
        
        self.control_label = QLabel("Pos:")
        self.control_label.setFixedWidth(35)
        control_layout.addWidget(self.control_label)
        
        self.control_slider = QSlider(Qt.Horizontal)
        self.control_slider.setRange(0, 1000)
        self.control_slider.setValue(0)
        self.control_slider.valueChanged.connect(self._on_value_changed)
        control_layout.addWidget(self.control_slider, 1)
        
        self.control_spin = QSpinBox()
        self.control_spin.setRange(-1000, 1000)
        self.control_spin.setFixedWidth(70)
        self.control_spin.valueChanged.connect(self._on_spin_changed)
        control_layout.addWidget(self.control_spin)
        
        layout.addLayout(control_layout)
        
        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: #444;")
        separator.setFixedHeight(1)
        layout.addWidget(separator)
        
        # Status display with progress bars
        # Position (0-1000, unidirectional)
        self.pos_bar = StatusProgressBar("Pos:", COLORS['primary'], bidirectional=False)
        layout.addWidget(self.pos_bar)
        
        # Speed (-1000 to 1000, bidirectional)
        self.speed_bar = StatusProgressBar("Spd:", COLORS['accent'], bidirectional=True)
        layout.addWidget(self.speed_bar)
        
        # Current (-1000 to 1000, bidirectional)
        self.current_bar = StatusProgressBar("Cur:", COLORS['warning'], bidirectional=True)
        layout.addWidget(self.current_bar)
        
        # State (text only)
        state_layout = QHBoxLayout()
        state_layout.setContentsMargins(0, 2, 0, 0)
        state_label = QLabel("State:")
        state_label.setFixedWidth(35)
        state_label.setStyleSheet("font-size: 11px;")
        state_layout.addWidget(state_label)
        self.state_value = QLabel("--")
        self.state_value.setStyleSheet("font-size: 11px;")
        state_layout.addWidget(self.state_value)
        state_layout.addStretch()
        layout.addLayout(state_layout)
    
    def update_mode(self, mode):
        """Update control mode and sync slider to current value"""
        # Store current actual values before changing mode
        current_pos = 0
        current_spd = 0
        current_cur = 0
        
        try:
            current_pos = self.pos_bar._value if hasattr(self.pos_bar, '_value') else 0
            current_spd = self.speed_bar._value if hasattr(self.speed_bar, '_value') else 0
            current_cur = self.current_bar._value if hasattr(self.current_bar, '_value') else 0
        except (ValueError, AttributeError):
            pass
        
        # Block signals to prevent sending commands during update
        self.control_slider.blockSignals(True)
        self.control_spin.blockSignals(True)
        
        if mode == MODE_POSITION:
            self.control_label.setText("Pos:")
            self.control_slider.setRange(0, 1000)
            self.control_spin.setRange(0, 1000)
            # Set to current position
            self.control_slider.setValue(max(0, min(1000, current_pos)))
            self.control_spin.setValue(max(0, min(1000, current_pos)))
        elif mode == MODE_SPEED:
            self.control_label.setText("Spd:")
            self.control_slider.setRange(-1000, 1000)
            self.control_spin.setRange(-1000, 1000)
            # Set to current speed
            self.control_slider.setValue(max(-1000, min(1000, current_spd)))
            self.control_spin.setValue(max(-1000, min(1000, current_spd)))
        elif mode == MODE_CURRENT:
            self.control_label.setText("Cur:")
            self.control_slider.setRange(-1000, 1000)
            self.control_spin.setRange(-1000, 1000)
            # Set to current current
            self.control_slider.setValue(max(-1000, min(1000, current_cur)))
            self.control_spin.setValue(max(-1000, min(1000, current_cur)))
        
        self.control_slider.blockSignals(False)
        self.control_spin.blockSignals(False)
    
    def sync_to_current_value(self, mode):
        """Sync slider/spinbox to current actual value without sending command"""
        current_pos = 0
        current_spd = 0
        current_cur = 0
        
        try:
            current_pos = self.pos_bar._value if hasattr(self.pos_bar, '_value') else 0
            current_spd = self.speed_bar._value if hasattr(self.speed_bar, '_value') else 0
            current_cur = self.current_bar._value if hasattr(self.current_bar, '_value') else 0
        except (ValueError, AttributeError):
            pass
        
        # Block signals to prevent sending commands
        self.control_slider.blockSignals(True)
        self.control_spin.blockSignals(True)
        
        if mode == MODE_POSITION:
            val = max(0, min(1000, current_pos))
            self.control_slider.setValue(val)
            self.control_spin.setValue(val)
        elif mode == MODE_SPEED:
            val = max(-1000, min(1000, current_spd))
            self.control_slider.setValue(val)
            self.control_spin.setValue(val)
        elif mode == MODE_CURRENT:
            val = max(-1000, min(1000, current_cur))
            self.control_slider.setValue(val)
            self.control_spin.setValue(val)
        
        self.control_slider.blockSignals(False)
        self.control_spin.blockSignals(False)
    
    def _on_spin_changed(self, value):
        """Spinbox changed"""
        self.control_slider.blockSignals(True)
        self.control_slider.setValue(value)
        self.control_slider.blockSignals(False)
    
    def _on_value_changed(self, value):
        """Slider value changed"""
        self.control_spin.blockSignals(True)
        self.control_spin.setValue(value)
        self.control_spin.blockSignals(False)
        
        device = self.device_getter()
        if device and FINGER_IDS:
            run_async(self._send_command(value))
    
    async def _send_command(self, value):
        """Send command based on current mode
        
        For Protobuf protocol (Revo1Protobuf), single finger control is not supported.
        We collect all slider values and send them together.
        """
        try:
            device = self.device_getter()
            slave_id = self.slave_id_getter()
            finger_enum = FINGER_IDS[self.finger_id]
            mode = self.mode_getter()
            shared_data = self.shared_data_getter()
            hw_type = shared_data.hw_type if shared_data else None
            
            # Check if this is Protobuf hardware (doesn't support single finger control)
            is_protobuf = is_protobuf_device(hw_type)
            
            if mode == MODE_POSITION:
                if is_protobuf:
                    # Protobuf: collect all slider values and send together
                    positions = self._get_all_slider_values()
                    print(f"[Motor] set_finger_positions(slave={slave_id}, positions={positions})")
                    await device.set_finger_positions(slave_id, positions)
                else:
                    print(f"[Motor] set_finger_position(slave={slave_id}, finger={self.finger_name}, pos={value})")
                    await device.set_finger_position(slave_id, finger_enum, value)
            elif mode == MODE_SPEED:
                if is_protobuf:
                    speeds = self._get_all_slider_values()
                    print(f"[Motor] set_finger_speeds(slave={slave_id}, speeds={speeds})")
                    await device.set_finger_speeds(slave_id, speeds)
                else:
                    print(f"[Motor] set_finger_speed(slave={slave_id}, finger={self.finger_name}, speed={value})")
                    await device.set_finger_speed(slave_id, finger_enum, value)
            elif mode == MODE_CURRENT:
                if is_protobuf:
                    currents = self._get_all_slider_values()
                    print(f"[Motor] set_finger_currents(slave={slave_id}, currents={currents})")
                    await device.set_finger_currents(slave_id, currents)
                else:
                    print(f"[Motor] set_finger_current(slave={slave_id}, finger={self.finger_name}, current={value})")
                    await device.set_finger_current(slave_id, finger_enum, value)
        except Exception as e:
            print(f"[Motor] Send command failed: {e}")
    
    def _get_all_slider_values(self):
        """Get all finger slider values from parent panel
        
        Returns list of 6 values for all fingers.
        """
        # Access parent's finger_controls to get all slider values
        parent = self.parent()
        while parent and not hasattr(parent, 'finger_controls'):
            parent = parent.parent()
        
        if parent and hasattr(parent, 'finger_controls'):
            return [ctrl.control_slider.value() for ctrl in parent.finger_controls]
        else:
            # Fallback: return current value for all fingers
            return [self.control_slider.value()] * 6
    
    def update_status(self, position, speed, current, state):
        """Update status display with progress bars"""
        self.pos_bar.setValue(position)
        self.speed_bar.setValue(speed)
        self.current_bar.setValue(current)
        
        # Format state and set color
        state_text = format_motor_state(state)
        
        # Color based on state
        if 'Running' in state_text:
            color = COLORS['accent']  # Green - running
        elif 'Stall' in state_text:
            color = COLORS.get('error', '#e74c3c')  # Red - stalled
        elif 'Turbo' in state_text:
            color = COLORS['warning']  # Orange - turbo
        elif 'Idle' in state_text:
            color = '#888888'  # Gray - idle
        else:
            color = '#666666'  # Dark gray - unknown
        
        self.state_value.setText(state_text)
        self.state_value.setStyleSheet(f"font-size: 11px; color: {color}; font-weight: bold;")


class MotorControlPanel(QWidget):
    """Motor Control Panel with Position/Speed/Current modes
    
    Uses SharedDataManager for motor status display and device state.
    """
    
    def __init__(self):
        super().__init__()
        self.shared_data: Optional['SharedDataManager'] = None
        self.current_mode = MODE_POSITION
        
        self._setup_ui()
        self.update_texts()
        
        # Timer for reading from shared data
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_status_from_shared)
        self.update_timer.setInterval(50)  # 20Hz UI update
    
    # Properties to get device state from shared_data
    @property
    def device(self):
        return self.shared_data.device if self.shared_data else None
    
    @property
    def slave_id(self):
        return self.shared_data.slave_id if self.shared_data else 1
    
    @property
    def device_info(self):
        return self.shared_data.device_info if self.shared_data else None
    
    def _setup_ui(self):
        """Setup UI"""
        layout = QVBoxLayout()
        layout.setSpacing(12)
        layout.setContentsMargins(12, 12, 12, 12)
        self.setLayout(layout)
        
        # Control mode and global buttons in one row
        top_layout = QHBoxLayout()
        top_layout.setSpacing(16)
        
        # Mode selection
        self.mode_label = QLabel(tr("mode") + ":")
        top_layout.addWidget(self.mode_label)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([tr("mode_position"), tr("mode_speed"), tr("mode_current")])
        self.mode_combo.setMinimumWidth(100)
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        top_layout.addWidget(self.mode_combo)
        
        top_layout.addWidget(QLabel("|"))
        
        # Global buttons
        self.open_all_btn = QPushButton(tr("btn_open_all"))
        self.open_all_btn.clicked.connect(self._open_all)
        top_layout.addWidget(self.open_all_btn)
        
        self.close_all_btn = QPushButton(tr("btn_close_all"))
        self.close_all_btn.clicked.connect(self._close_all)
        top_layout.addWidget(self.close_all_btn)
        
        self.zero_all_btn = QPushButton(tr("btn_zero_all"))
        self.zero_all_btn.clicked.connect(self._zero_all)
        top_layout.addWidget(self.zero_all_btn)
        
        top_layout.addStretch()
        layout.addLayout(top_layout)
        
        # Finger controls in grid (6 fingers: 2 rows x 3 cols)
        fingers_layout = QGridLayout()
        fingers_layout.setSpacing(12)
        
        self.finger_controls = []
        for i in range(MOTOR_COUNT):
            control = FingerControl(
                i, 
                lambda: self.current_mode,
                lambda: self.device,
                lambda: self.slave_id,
                lambda: self.shared_data
            )
            self.finger_controls.append(control)
            row = i // 3
            col = i % 3
            fingers_layout.addWidget(control, row, col)
        
        # Make columns stretch equally to fill space
        for col in range(3):
            fingers_layout.setColumnStretch(col, 1)
        # Make rows stretch equally
        for row in range(2):
            fingers_layout.setRowStretch(row, 1)
        
        layout.addLayout(fingers_layout, 1)  # Give it stretch factor
    
    def update_texts(self):
        """Update texts for i18n"""
        self.mode_label.setText(tr("mode") + ":")
        self.mode_combo.setItemText(0, tr("mode_position"))
        self.mode_combo.setItemText(1, tr("mode_speed"))
        self.mode_combo.setItemText(2, tr("mode_current"))
        self.open_all_btn.setText(tr("btn_open_all"))
        self.close_all_btn.setText(tr("btn_close_all"))
        self.zero_all_btn.setText(tr("btn_zero_all"))
    
    def _on_mode_changed(self, index):
        """Mode changed"""
        self.current_mode = index
        for control in self.finger_controls:
            control.update_mode(index)
    
    def sync_sliders_to_current(self):
        """Sync all sliders to current motor values (call when tab becomes visible)"""
        for control in self.finger_controls:
            control.sync_to_current_value(self.current_mode)
    
    def set_device(self, device, slave_id, device_info, shared_data=None):
        """Set device for motor control - uses SharedDataManager
        
        FingerControl instances use getter functions to access device/slave_id
        from this panel's properties, which in turn get them from shared_data.
        """
        self.shared_data = shared_data
        
        if device and shared_data:
            self.update_timer.start()
        else:
            self.update_timer.stop()
    
    def _on_slave_id_updated(self, new_id):
        """Handle slave_id change from shared_data
        
        FingerControl instances use getter functions, so they automatically
        get the updated slave_id from shared_data.
        """
        pass
    
    def clear_device(self):
        """Clear device when disconnected"""
        self.update_timer.stop()
        self.shared_data = None
    
    def _update_status_from_shared(self):
        """Update motor status from shared data manager (non-blocking)"""
        if not self.shared_data:
            return
        
        try:
            # Get latest status from shared data
            status = self.shared_data.get_latest_motor()
            if status:
                num_positions = len(status.positions) if status.positions else 0
                num_speeds = len(status.speeds) if status.speeds else 0
                num_currents = len(status.currents) if status.currents else 0
                num_states = len(status.states) if status.states else 0
                
                for i, control in enumerate(self.finger_controls):
                    pos = status.positions[i] if i < num_positions else 0
                    spd = status.speeds[i] if i < num_speeds else 0
                    cur = status.currents[i] if i < num_currents else 0
                    state = status.states[i] if i < num_states else 0
                    control.update_status(pos, spd, cur, state)
        except Exception as e:
            print(f"Update status from shared data failed: {e}")
    
    def _open_all(self):
        """Open all fingers (position 0)"""
        if self.current_mode == MODE_POSITION:
            for control in self.finger_controls:
                control.control_slider.setValue(0)
    
    def _close_all(self):
        """Close all fingers (position 1000)"""
        if self.current_mode == MODE_POSITION:
            for control in self.finger_controls:
                control.control_slider.setValue(1000)
    
    def _zero_all(self):
        """Set all to zero (for speed/current modes)"""
        for control in self.finger_controls:
            control.control_slider.setValue(0)
