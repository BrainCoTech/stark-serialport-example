"""System Configuration Panel - Extended with Motor and Communication Settings"""

import asyncio
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional, TYPE_CHECKING
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QSpinBox, QTextEdit,
    QFormLayout, QMessageBox, QComboBox, QCheckBox,
    QScrollArea, QFrame, QTabWidget
)
from PySide6.QtCore import Signal

# Add parent directory to path for SDK import
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import sdk

if TYPE_CHECKING:
    from .shared_data import SharedDataManager

from .i18n import tr
from .styles import COLORS


def run_async(coro):
    """Run async coroutine in a new event loop (for Qt callbacks)"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


class SystemConfigPanel(QWidget):
    """System Configuration Panel with Motor and Communication Settings
    
    Uses SharedDataManager for device state.
    Note: This panel emits slave_id_changed signal when slave_id is modified.
    """
    
    # Signal emitted when slave_id is changed (new_id)
    slave_id_changed = Signal(int)
    
    def __init__(self):
        super().__init__()
        self.shared_data: Optional['SharedDataManager'] = None
        self.protocol = None
        self._loading_settings = False  # Prevent triggering saves during load
        
        self._setup_ui()
        self.update_texts()
    
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
    
    @property
    def hw_type(self):
        if self.device_info:
            return getattr(self.device_info, 'hardware_type', None)
        return None
    
    def _uses_revo1_motor_api(self):
        """Check if device uses Revo1 Motor API via SDK."""
        if self.device_info:
            return self.device_info.uses_revo1_motor_api()
        return False
    
    def _uses_revo2_motor_api(self):
        """Check if device uses Revo2 Motor API via SDK."""
        if self.device_info:
            return self.device_info.uses_revo2_motor_api()
        return False
    
    def _supports_finger_settings(self):
        """Check if device supports finger settings API (requires Modbus or CANFD)"""
        if not self.device or not self._uses_revo2_motor_api():
            return False
        # get_all_finger_settings only works with Modbus or CANFD
        try:
            from common_imports import sdk
            protocol = self.device.get_protocol_type()
            return protocol in [sdk.StarkProtocolType.Modbus, sdk.StarkProtocolType.CanFd]
        except:
            return False
    
    def _setup_ui(self):
        """Setup UI with tabs"""
        layout = QVBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        self.setLayout(layout)
        
        # Tab widget
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)
        
        # Tab 1: Device Info & Basic Settings
        self._setup_basic_tab()
        
        # Tab 2: Motor Settings
        self._setup_motor_tab()
        
        # Tab 3: Communication Settings
        self._setup_comm_tab()
        
        # Tab 4: Revo2 Finger Settings
        self._setup_finger_settings_tab()
        
        # Log at bottom
        self.log_group = QGroupBox()
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        log_layout.addWidget(self.log_text)
        self.log_group.setLayout(log_layout)
        layout.addWidget(self.log_group)
    
    def _setup_basic_tab(self):
        """Setup basic settings tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)
        
        # Device info
        self.info_group = QGroupBox()
        info_layout = QFormLayout()
        
        self.sn_title = QLabel()
        self.sn_label = QLabel("--")
        info_layout.addRow(self.sn_title, self.sn_label)
        
        self.fw_title = QLabel()
        self.fw_label = QLabel("--")
        info_layout.addRow(self.fw_title, self.fw_label)
        
        self.hw_title = QLabel()
        self.hw_label = QLabel("--")
        info_layout.addRow(self.hw_title, self.hw_label)
        
        self.sku_title = QLabel("SKU:")
        self.sku_label = QLabel("--")
        info_layout.addRow(self.sku_title, self.sku_label)
        
        self.protocol_title = QLabel("Protocol:")
        self.protocol_label = QLabel("--")
        info_layout.addRow(self.protocol_title, self.protocol_label)
        
        self.info_group.setLayout(info_layout)
        layout.addWidget(self.info_group)
        
        # Slave ID settings
        self.slave_id_group = QGroupBox()
        slave_id_layout = QHBoxLayout()
        
        self.new_slave_id_label = QLabel()
        slave_id_layout.addWidget(self.new_slave_id_label)
        
        self.new_slave_id_spin = QSpinBox()
        self.new_slave_id_spin.setRange(1, 247)
        slave_id_layout.addWidget(self.new_slave_id_spin)
        
        self.set_slave_id_btn = QPushButton()
        self.set_slave_id_btn.clicked.connect(self._set_slave_id)
        slave_id_layout.addWidget(self.set_slave_id_btn)
        
        slave_id_layout.addStretch()
        self.slave_id_group.setLayout(slave_id_layout)
        layout.addWidget(self.slave_id_group)
        
        # System control
        self.system_group = QGroupBox()
        system_layout = QHBoxLayout()
        
        self.reboot_btn = QPushButton()
        self.reboot_btn.clicked.connect(self._reboot)
        system_layout.addWidget(self.reboot_btn)
        
        self.factory_reset_btn = QPushButton()
        self.factory_reset_btn.clicked.connect(self._factory_reset)
        system_layout.addWidget(self.factory_reset_btn)
        
        system_layout.addStretch()
        self.system_group.setLayout(system_layout)
        layout.addWidget(self.system_group)
        
        layout.addStretch()
        self.tabs.addTab(widget, "📋 " + tr("device_info"))
    
    def _setup_motor_tab(self):
        """Setup motor settings tab"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)
        
        # Turbo Mode (all devices)
        self.turbo_group = QGroupBox("Turbo Mode")
        turbo_layout = QVBoxLayout()
        
        turbo_row1 = QHBoxLayout()
        self.turbo_check = QCheckBox("Enable Turbo Mode")
        self.turbo_check.stateChanged.connect(self._on_turbo_changed)
        turbo_row1.addWidget(self.turbo_check)
        turbo_row1.addStretch()
        turbo_layout.addLayout(turbo_row1)
        
        turbo_row2 = QHBoxLayout()
        turbo_row2.addWidget(QLabel("Interval (ms):"))
        self.turbo_interval_spin = QSpinBox()
        self.turbo_interval_spin.setRange(0, 10000)
        self.turbo_interval_spin.setValue(1000)
        turbo_row2.addWidget(self.turbo_interval_spin)
        turbo_row2.addWidget(QLabel("Duration (ms):"))
        self.turbo_duration_spin = QSpinBox()
        self.turbo_duration_spin.setRange(0, 10000)
        self.turbo_duration_spin.setValue(500)
        turbo_row2.addWidget(self.turbo_duration_spin)
        self.turbo_apply_btn = QPushButton("Apply")
        self.turbo_apply_btn.clicked.connect(self._apply_turbo_config)
        turbo_row2.addWidget(self.turbo_apply_btn)
        turbo_row2.addStretch()
        turbo_layout.addLayout(turbo_row2)
        
        self.turbo_group.setLayout(turbo_layout)
        layout.addWidget(self.turbo_group)
        
        # Calibration (all devices)
        self.calib_group = QGroupBox("Position Calibration")
        calib_layout = QVBoxLayout()
        
        calib_row1 = QHBoxLayout()
        self.auto_calib_check = QCheckBox("Auto Calibration on Power-up")
        self.auto_calib_check.stateChanged.connect(self._on_auto_calib_changed)
        calib_row1.addWidget(self.auto_calib_check)
        calib_row1.addStretch()
        calib_layout.addLayout(calib_row1)
        
        calib_row2 = QHBoxLayout()
        self.manual_calib_btn = QPushButton("🔧 Manual Calibration")
        self.manual_calib_btn.clicked.connect(self._manual_calibrate)
        calib_row2.addWidget(self.manual_calib_btn)
        calib_row2.addStretch()
        calib_layout.addLayout(calib_row2)
        
        self.calib_group.setLayout(calib_layout)
        layout.addWidget(self.calib_group)
        
        # Force Level (Revo1 Basic only)
        self.force_level_group = QGroupBox("Force Level (Revo1 Basic)")
        force_layout = QHBoxLayout()
        force_layout.addWidget(QLabel("Level:"))
        self.force_level_combo = QComboBox()
        self.force_level_combo.addItems(["Small", "Normal", "Full"])
        self.force_level_combo.currentIndexChanged.connect(self._on_force_level_changed)
        force_layout.addWidget(self.force_level_combo)
        force_layout.addStretch()
        self.force_level_group.setLayout(force_layout)
        layout.addWidget(self.force_level_group)
        
        # Unit Mode (Revo2 only)
        self.unit_mode_group = QGroupBox("Unit Mode (Revo2)")
        unit_layout = QHBoxLayout()
        unit_layout.addWidget(QLabel("Mode:"))
        self.unit_mode_combo = QComboBox()
        self.unit_mode_combo.addItems(["Normalized (0-1000)", "Physical (degrees/mA)"])
        self.unit_mode_combo.currentIndexChanged.connect(self._on_unit_mode_changed)
        unit_layout.addWidget(self.unit_mode_combo)
        unit_layout.addStretch()
        self.unit_mode_group.setLayout(unit_layout)
        layout.addWidget(self.unit_mode_group)
        
        # Peripheral Settings
        self.peripheral_group = QGroupBox("Peripheral Settings")
        periph_layout = QVBoxLayout()
        
        periph_row = QHBoxLayout()
        self.led_check = QCheckBox("LED")
        self.led_check.stateChanged.connect(self._on_led_changed)
        periph_row.addWidget(self.led_check)
        
        self.buzzer_check = QCheckBox("Buzzer")
        self.buzzer_check.stateChanged.connect(self._on_buzzer_changed)
        periph_row.addWidget(self.buzzer_check)
        
        self.vibration_check = QCheckBox("Vibration")
        self.vibration_check.stateChanged.connect(self._on_vibration_changed)
        periph_row.addWidget(self.vibration_check)
        
        periph_row.addStretch()
        periph_layout.addLayout(periph_row)
        self.peripheral_group.setLayout(periph_layout)
        layout.addWidget(self.peripheral_group)
        
        # Refresh button
        refresh_layout = QHBoxLayout()
        self.refresh_motor_btn = QPushButton("🔄 Refresh Settings")
        self.refresh_motor_btn.clicked.connect(self._load_motor_settings)
        refresh_layout.addWidget(self.refresh_motor_btn)
        refresh_layout.addStretch()
        layout.addLayout(refresh_layout)
        
        layout.addStretch()
        scroll.setWidget(widget)
        self.tabs.addTab(scroll, "⚙️ Motor")
    
    def _setup_comm_tab(self):
        """Setup communication settings tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)
        
        # Modbus Baudrate
        self.modbus_group = QGroupBox("Modbus/RS485 Baudrate")
        modbus_layout = QHBoxLayout()
        modbus_layout.addWidget(QLabel("Baudrate:"))
        self.modbus_baud_combo = QComboBox()
        self.modbus_baud_combo.addItems(["115200", "460800", "1000000", "2000000", "5000000"])
        modbus_layout.addWidget(self.modbus_baud_combo)
        self.modbus_baud_btn = QPushButton("Set")
        self.modbus_baud_btn.clicked.connect(self._set_modbus_baudrate)
        modbus_layout.addWidget(self.modbus_baud_btn)
        modbus_layout.addStretch()
        self.modbus_group.setLayout(modbus_layout)
        layout.addWidget(self.modbus_group)
        
        # CANFD Baudrate
        self.canfd_group = QGroupBox("CANFD Data Baudrate")
        canfd_layout = QHBoxLayout()
        canfd_layout.addWidget(QLabel("Data Rate:"))
        self.canfd_baud_combo = QComboBox()
        self.canfd_baud_combo.addItems(["1 Mbps", "2 Mbps", "4 Mbps", "5 Mbps"])
        canfd_layout.addWidget(self.canfd_baud_combo)
        self.canfd_baud_btn = QPushButton("Set")
        self.canfd_baud_btn.clicked.connect(self._set_canfd_baudrate)
        canfd_layout.addWidget(self.canfd_baud_btn)
        canfd_layout.addStretch()
        self.canfd_group.setLayout(canfd_layout)
        layout.addWidget(self.canfd_group)
        
        # Current baudrate display
        self.current_baud_group = QGroupBox("Current Settings")
        current_layout = QFormLayout()
        self.current_modbus_label = QLabel("--")
        current_layout.addRow("Modbus Baudrate:", self.current_modbus_label)
        self.current_canfd_label = QLabel("--")
        current_layout.addRow("CANFD Data Rate:", self.current_canfd_label)
        self.current_baud_group.setLayout(current_layout)
        layout.addWidget(self.current_baud_group)
        
        # Refresh button
        refresh_layout = QHBoxLayout()
        self.refresh_comm_btn = QPushButton("🔄 Refresh Settings")
        self.refresh_comm_btn.clicked.connect(self._load_comm_settings)
        refresh_layout.addWidget(self.refresh_comm_btn)
        refresh_layout.addStretch()
        layout.addLayout(refresh_layout)
        
        layout.addStretch()
        self.tabs.addTab(widget, "📡 Communication")
    
    def _setup_finger_settings_tab(self):
        """Setup Revo2 finger settings tab"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)
        
        # Finger names for display
        self.finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Thumb Aux"]
        
        # Finger Settings Group
        self.finger_settings_group = QGroupBox(tr("finger_settings"))
        finger_layout = QVBoxLayout()
        
        # Create spinboxes for each finger's settings
        self.finger_min_pos_spins = []
        self.finger_max_pos_spins = []
        self.finger_max_speed_spins = []
        self.finger_max_current_spins = []
        
        # Header row
        header_layout = QHBoxLayout()
        header_layout.addWidget(QLabel("Finger"), 2)
        header_layout.addWidget(QLabel(tr("min_position")), 1)
        header_layout.addWidget(QLabel(tr("max_position")), 1)
        header_layout.addWidget(QLabel(tr("max_speed")), 1)
        header_layout.addWidget(QLabel(tr("max_current")), 1)
        finger_layout.addLayout(header_layout)
        
        # Create row for each finger
        for i, name in enumerate(self.finger_names):
            row = QHBoxLayout()
            row.addWidget(QLabel(name), 2)
            
            min_pos = QSpinBox()
            min_pos.setRange(0, 65535)
            min_pos.setMaximumWidth(80)
            self.finger_min_pos_spins.append(min_pos)
            row.addWidget(min_pos, 1)
            
            max_pos = QSpinBox()
            max_pos.setRange(0, 65535)
            max_pos.setMaximumWidth(80)
            self.finger_max_pos_spins.append(max_pos)
            row.addWidget(max_pos, 1)
            
            max_speed = QSpinBox()
            max_speed.setRange(0, 65535)
            max_speed.setMaximumWidth(80)
            self.finger_max_speed_spins.append(max_speed)
            row.addWidget(max_speed, 1)
            
            max_current = QSpinBox()
            max_current.setRange(0, 65535)
            max_current.setMaximumWidth(80)
            self.finger_max_current_spins.append(max_current)
            row.addWidget(max_current, 1)
            
            finger_layout.addLayout(row)
        
        # Apply button
        apply_row = QHBoxLayout()
        self.apply_finger_settings_btn = QPushButton(tr("apply_all"))
        self.apply_finger_settings_btn.clicked.connect(self._apply_all_finger_settings)
        apply_row.addStretch()
        apply_row.addWidget(self.apply_finger_settings_btn)
        finger_layout.addLayout(apply_row)
        
        self.finger_settings_group.setLayout(finger_layout)
        layout.addWidget(self.finger_settings_group)
        
        # Protected Currents Group
        self.protected_currents_group = QGroupBox(tr("protected_currents"))
        protected_layout = QVBoxLayout()
        
        self.protected_current_spins = []
        
        # Header
        prot_header = QHBoxLayout()
        prot_header.addWidget(QLabel("Finger"), 2)
        prot_header.addWidget(QLabel(tr("protected_current")), 1)
        protected_layout.addLayout(prot_header)
        
        # Create row for each finger
        for i, name in enumerate(self.finger_names):
            row = QHBoxLayout()
            row.addWidget(QLabel(name), 2)
            
            prot_spin = QSpinBox()
            prot_spin.setRange(0, 1000)
            prot_spin.setMaximumWidth(80)
            self.protected_current_spins.append(prot_spin)
            row.addWidget(prot_spin, 1)
            
            protected_layout.addLayout(row)
        
        # Apply button
        prot_apply_row = QHBoxLayout()
        self.apply_protected_currents_btn = QPushButton(tr("apply_all"))
        self.apply_protected_currents_btn.clicked.connect(self._apply_protected_currents)
        prot_apply_row.addStretch()
        prot_apply_row.addWidget(self.apply_protected_currents_btn)
        protected_layout.addLayout(prot_apply_row)
        
        self.protected_currents_group.setLayout(protected_layout)
        layout.addWidget(self.protected_currents_group)
        
        # Refresh button
        refresh_layout = QHBoxLayout()
        self.refresh_finger_btn = QPushButton("🔄 " + tr("refresh_settings"))
        self.refresh_finger_btn.clicked.connect(self._load_finger_settings)
        refresh_layout.addWidget(self.refresh_finger_btn)
        refresh_layout.addStretch()
        layout.addLayout(refresh_layout)
        
        layout.addStretch()
        scroll.setWidget(widget)
        
        # Store tab index for later show/hide
        self.finger_settings_tab_index = self.tabs.addTab(scroll, "🖐️ Revo2 Finger")
    
    def update_texts(self):
        """Update texts for i18n"""
        self.info_group.setTitle(tr("device_info"))
        self.sn_title.setText(tr("serial_number") + ":")
        self.fw_title.setText(tr("firmware_version") + ":")
        self.hw_title.setText(tr("hardware_type") + ":")
        
        self.slave_id_group.setTitle(tr("slave_id_settings"))
        self.new_slave_id_label.setText(tr("new_slave_id") + ":")
        self.set_slave_id_btn.setText(tr("btn_set"))
        
        self.system_group.setTitle(tr("system_control"))
        self.reboot_btn.setText(tr("btn_reboot"))
        self.factory_reset_btn.setText(tr("btn_factory_reset"))
        
        self.log_group.setTitle(tr("operation_log"))

    def set_device(self, device, slave_id, device_info, protocol=None, shared_data=None):
        """Set device"""
        self.shared_data = shared_data
        self.protocol = protocol
        
        # Update connection info
        if protocol:
            self.protocol_label.setText(protocol)
        
        # Update device info
        if device_info:
            self.sn_label.setText(device_info.serial_number)
            self.fw_label.setText(device_info.firmware_version)
            hw_type = device_info.hardware_type.name if hasattr(device_info.hardware_type, 'name') else str(device_info.hardware_type)
            self.hw_label.setText(hw_type.replace("StarkHardwareType.", ""))
            
            sku_type = device_info.sku_type.name if hasattr(device_info.sku_type, 'name') else str(device_info.sku_type)
            self.sku_label.setText(sku_type.replace("SkuType.", ""))
        
        self.new_slave_id_spin.setValue(slave_id)
        
        # Show/hide device-specific settings
        self._update_device_specific_ui()
        
        # Load current settings
        self._load_motor_settings()
        self._load_comm_settings()
        
        # Load Revo2 finger settings if applicable (only for Modbus/CANFD)
        if self._supports_finger_settings():
            self._load_finger_settings()
    
    def _update_device_specific_ui(self):
        """Show/hide UI elements based on device type"""
        uses_revo1_api = self._uses_revo1_motor_api()
        uses_revo2_api = self._uses_revo2_motor_api()
        
        # Debug: print device type detection
        hw_name = self.hw_type.name if self.hw_type and hasattr(self.hw_type, 'name') else str(self.hw_type)
        print(f"[SystemConfig] Device type: {hw_name}, uses_revo1_motor_api={uses_revo1_api}, uses_revo2_motor_api={uses_revo2_api}")
        
        # Force level only for Revo1 Basic
        self.force_level_group.setVisible(uses_revo1_api)
        
        # Unit mode only for Revo2
        self.unit_mode_group.setVisible(uses_revo2_api)
        
        # Finger settings tab only for Revo2
        if hasattr(self, 'finger_settings_tab_index'):
            self.tabs.setTabVisible(self.finger_settings_tab_index, uses_revo2_api)
    
    def _load_motor_settings(self):
        """Load motor settings from device"""
        if not self.device:
            return
        self._loading_settings = True
        run_async(self._async_load_motor_settings())
        self._loading_settings = False
    
    async def _async_load_motor_settings(self):
        """Async load motor settings"""
        if not self.device:
            return
        try:
            # Check if Protobuf protocol (limited feature support)
            is_protobuf = self.protocol and "Protobuf" in self.protocol
            
            if is_protobuf:
                # Protobuf only supports basic motor control, skip config reads
                self._log("Protobuf protocol: motor config settings not available")
                self.turbo_group.setEnabled(False)
                self.calib_group.setEnabled(False)
                self.peripheral_group.setEnabled(False)
                return
            
            # Enable groups for non-Protobuf protocols
            self.turbo_group.setEnabled(True)
            self.calib_group.setEnabled(True)
            self.peripheral_group.setEnabled(True)
            
            # Turbo mode
            turbo_enabled = await self.device.get_turbo_mode_enabled(self.slave_id)
            self.turbo_check.setChecked(turbo_enabled)
            
            turbo_config = await self.device.get_turbo_config(self.slave_id)
            self.turbo_interval_spin.setValue(turbo_config.interval)
            self.turbo_duration_spin.setValue(turbo_config.duration)
            
            # Auto calibration
            auto_calib = await self.device.get_auto_calibration_enabled(self.slave_id)
            self.auto_calib_check.setChecked(auto_calib)
            
            # Peripheral settings
            led_enabled = await self.device.get_led_enabled(self.slave_id)
            self.led_check.setChecked(led_enabled)
            
            buzzer_enabled = await self.device.get_buzzer_enabled(self.slave_id)
            self.buzzer_check.setChecked(buzzer_enabled)
            
            vibration_enabled = await self.device.get_vibration_enabled(self.slave_id)
            self.vibration_check.setChecked(vibration_enabled)
            
            # Force level (Revo1 only)
            if self._uses_revo1_motor_api():
                try:
                    force_level = await self.device.get_force_level(self.slave_id)
                    level_name = force_level.name if hasattr(force_level, 'name') else str(force_level)
                    if 'Small' in level_name:
                        self.force_level_combo.setCurrentIndex(0)
                    elif 'Normal' in level_name:
                        self.force_level_combo.setCurrentIndex(1)
                    elif 'Full' in level_name:
                        self.force_level_combo.setCurrentIndex(2)
                except Exception as e:
                    self._log(f"Force level not available: {e}")
            
            # Unit mode (Revo2 only)
            if self._uses_revo2_motor_api():
                try:
                    unit_mode = await self.device.get_finger_unit_mode(self.slave_id)
                    mode_name = unit_mode.name if hasattr(unit_mode, 'name') else str(unit_mode)
                    if 'Normalized' in mode_name:
                        self.unit_mode_combo.setCurrentIndex(0)
                    elif 'Physical' in mode_name:
                        self.unit_mode_combo.setCurrentIndex(1)
                except Exception as e:
                    self._log(f"Unit mode not available: {e}")
            
            self._log("Motor settings loaded")
        except Exception as e:
            self._log(f"Failed to load motor settings: {e}")
    
    def _load_comm_settings(self):
        """Load communication settings from device"""
        if not self.device:
            return
        run_async(self._async_load_comm_settings())
    
    async def _async_load_comm_settings(self):
        """Async load communication settings"""
        if not self.device:
            return
        try:
            # Check protocol type - only load baudrate for Modbus/CANFD protocols
            is_modbus = self.protocol and "Modbus" in self.protocol
            is_canfd = self.protocol and "CANFD" in self.protocol
            
            # Modbus baudrate - only for Modbus protocol
            if is_modbus:
                try:
                    baudrate = await self.device.get_serialport_baudrate(self.slave_id)
                    baud_name = baudrate.name if hasattr(baudrate, 'name') else str(baudrate)
                    baud_display = baud_name.replace("Baudrate.", "").replace("Baud", "")
                    self.current_modbus_label.setText(baud_display)
                    
                    # Select current value in combo box
                    baud_map = {
                        "115200": 0, "460800": 1, "1Mbps": 2, "2Mbps": 3, "5Mbps": 4
                    }
                    idx = baud_map.get(baud_display, -1)
                    if idx >= 0:
                        self.modbus_baud_combo.setCurrentIndex(idx)
                except Exception:
                    self.current_modbus_label.setText("N/A")
            else:
                self.current_modbus_label.setText("N/A (CAN)")
            
            # CANFD baudrate - only for CANFD protocol
            if is_canfd:
                try:
                    canfd_baud = await self.device.get_canfd_baudrate(self.slave_id)
                    baud_name = canfd_baud.name if hasattr(canfd_baud, 'name') else str(canfd_baud)
                    baud_display = baud_name.replace("BaudrateCAN.", "").replace("Baud", "")
                    self.current_canfd_label.setText(baud_display)
                    
                    # Select current value in combo box
                    canfd_map = {
                        "1Mbps": 0, "2Mbps": 1, "4Mbps": 2, "5Mbps": 3
                    }
                    idx = canfd_map.get(baud_display, -1)
                    if idx >= 0:
                        self.canfd_baud_combo.setCurrentIndex(idx)
                except Exception:
                    self.current_canfd_label.setText("N/A")
            else:
                self.current_canfd_label.setText("N/A (CAN)")
            
            self._log("Communication settings loaded")
        except Exception as e:
            self._log(f"Failed to load comm settings: {e}")
    
    # ========== Motor Settings Handlers ==========
    
    def _on_turbo_changed(self, state):
        """Turbo mode checkbox changed"""
        print(f"[DEBUG] _on_turbo_changed: state={state}, _loading_settings={self._loading_settings}, device={self.device}")
        if self._loading_settings or not self.device:
            print(f"[DEBUG] Skipped: _loading_settings={self._loading_settings}, device={self.device}")
            return
        enabled = state == 2  # Qt.Checked
        self._log(f"Turbo mode checkbox: {'checked' if enabled else 'unchecked'}")
        run_async(self._async_set_turbo_enabled(enabled))
    
    async def _async_set_turbo_enabled(self, enabled):
        if not self.device:
            return
        try:
            self._log(f"Setting turbo mode: {'enabled' if enabled else 'disabled'}...")
            await self.device.set_turbo_mode_enabled(self.slave_id, enabled)
            self._log(f"Turbo mode {'enabled' if enabled else 'disabled'} ✓")
        except Exception as e:
            self._log(f"Failed to set turbo mode: {e}")
    
    def _apply_turbo_config(self):
        """Apply turbo config"""
        if not self.device:
            return
        interval = self.turbo_interval_spin.value()
        duration = self.turbo_duration_spin.value()
        run_async(self._async_set_turbo_config(interval, duration))
    
    async def _async_set_turbo_config(self, interval, duration):
        if not self.device:
            return
        try:
            self._log(f"Setting turbo config: interval={interval}ms, duration={duration}ms...")
            config = sdk.TurboConfig(interval, duration)
            await self.device.set_turbo_config(self.slave_id, config)
            self._log(f"Turbo config applied ✓")
        except Exception as e:
            self._log(f"Failed to set turbo config: {e}")
    
    def _on_auto_calib_changed(self, state):
        """Auto calibration checkbox changed"""
        if self._loading_settings or not self.device:
            return
        enabled = state == 2
        run_async(self._async_set_auto_calib(enabled))
    
    async def _async_set_auto_calib(self, enabled):
        if not self.device:
            return
        try:
            self._log(f"Setting auto calibration: {'enabled' if enabled else 'disabled'}...")
            await self.device.set_auto_calibration(self.slave_id, enabled)
            self._log(f"Auto calibration {'enabled' if enabled else 'disabled'} ✓")
        except Exception as e:
            self._log(f"Failed to set auto calibration: {e}")
    
    def _manual_calibrate(self):
        """Manual position calibration"""
        if not self.device:
            self._log("No device connected")
            return
        reply = QMessageBox.question(
            self,
            "Confirm Calibration",
            "This will calibrate all finger positions.\nMake sure fingers can move freely.\n\nContinue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self._log("Manual calibration requested...")
            run_async(self._async_manual_calibrate())
    
    async def _async_manual_calibrate(self):
        if not self.device:
            return
        try:
            self._log("Sending calibrate_position command...")
            await self.device.calibrate_position(self.slave_id)
            self._log("Manual calibration command sent ✓")
        except Exception as e:
            self._log(f"Calibration failed: {e}")
    
    def _on_force_level_changed(self, index):
        """Force level changed (Revo1 only)"""
        if self._loading_settings or not self.device:
            return
        run_async(self._async_set_force_level(index))
    
    async def _async_set_force_level(self, index):
        if not self.device:
            return
        try:
            level_names = ['Small', 'Normal', 'Full']
            self._log(f"Setting force level: {level_names[index]}...")
            levels = [sdk.ForceLevel.Small, sdk.ForceLevel.Normal, sdk.ForceLevel.Full]
            await self.device.set_force_level(self.slave_id, levels[index])
            self._log(f"Force level set to {level_names[index]} ✓")
        except Exception as e:
            self._log(f"Failed to set force level: {e}")
    
    def _on_unit_mode_changed(self, index):
        """Unit mode changed (Revo2 only)"""
        if self._loading_settings or not self.device:
            return
        run_async(self._async_set_unit_mode(index))
    
    async def _async_set_unit_mode(self, index):
        if not self.device:
            return
        try:
            mode_names = ['Normalized', 'Physical']
            self._log(f"Setting unit mode: {mode_names[index]}...")
            modes = [sdk.FingerUnitMode.Normalized, sdk.FingerUnitMode.Physical]
            await self.device.set_finger_unit_mode(self.slave_id, modes[index])
            self._log(f"Unit mode set to {mode_names[index]} ✓")
        except Exception as e:
            self._log(f"Failed to set unit mode: {e}")
    
    def _on_led_changed(self, state):
        """LED checkbox changed"""
        if self._loading_settings or not self.device:
            return
        enabled = state == 2
        run_async(self._async_set_led(enabled))
    
    async def _async_set_led(self, enabled):
        if not self.device:
            return
        try:
            self._log(f"Setting LED: {'enabled' if enabled else 'disabled'}...")
            await self.device.set_led_enabled(self.slave_id, enabled)
            self._log(f"LED {'enabled' if enabled else 'disabled'} ✓")
        except Exception as e:
            self._log(f"Failed to set LED: {e}")
    
    def _on_buzzer_changed(self, state):
        """Buzzer checkbox changed"""
        if self._loading_settings or not self.device:
            return
        enabled = state == 2
        run_async(self._async_set_buzzer(enabled))
    
    async def _async_set_buzzer(self, enabled):
        if not self.device:
            return
        try:
            self._log(f"Setting buzzer: {'enabled' if enabled else 'disabled'}...")
            await self.device.set_buzzer_enabled(self.slave_id, enabled)
            self._log(f"Buzzer {'enabled' if enabled else 'disabled'} ✓")
        except Exception as e:
            self._log(f"Failed to set buzzer: {e}")
    
    def _on_vibration_changed(self, state):
        """Vibration checkbox changed"""
        if self._loading_settings or not self.device:
            return
        enabled = state == 2
        run_async(self._async_set_vibration(enabled))
    
    async def _async_set_vibration(self, enabled):
        if not self.device:
            return
        try:
            self._log(f"Setting vibration: {'enabled' if enabled else 'disabled'}...")
            await self.device.set_vibration_enabled(self.slave_id, enabled)
            self._log(f"Vibration {'enabled' if enabled else 'disabled'} ✓")
        except Exception as e:
            self._log(f"Failed to set vibration: {e}")

    # ========== Communication Settings Handlers ==========
    
    def _set_modbus_baudrate(self):
        """Set Modbus baudrate"""
        if not self.device:
            self._log("No device connected")
            return
        baud_text = self.modbus_baud_combo.currentText()
        reply = QMessageBox.question(
            self,
            "Confirm Baudrate Change",
            f"Set Modbus baudrate to {baud_text}?\n\nDevice will reboot after change.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self._log(f"Modbus baudrate change requested: {baud_text}")
            run_async(self._async_set_modbus_baudrate(baud_text))
    
    async def _async_set_modbus_baudrate(self, baud_text):
        if not self.device:
            return
        try:
            self._log(f"Setting Modbus baudrate: {baud_text}...")
            baud_map = {
                "115200": sdk.Baudrate.Baud115200,
                "460800": sdk.Baudrate.Baud460800,
                "1000000": sdk.Baudrate.Baud1Mbps,
                "2000000": sdk.Baudrate.Baud2Mbps,
                "5000000": sdk.Baudrate.Baud5Mbps,
            }
            baudrate = baud_map.get(baud_text)
            if baudrate:
                await self.device.set_serialport_baudrate(self.slave_id, baudrate)
                self._log(f"Modbus baudrate set to {baud_text} ✓")
        except Exception as e:
            self._log(f"Failed to set Modbus baudrate: {e}")
    
    def _set_canfd_baudrate(self):
        """Set CANFD baudrate"""
        if not self.device:
            self._log("No device connected")
            return
        baud_text = self.canfd_baud_combo.currentText()
        reply = QMessageBox.question(
            self,
            "Confirm Baudrate Change",
            f"Set CANFD data rate to {baud_text}?\n\nDevice will reboot after change.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self._log(f"CANFD baudrate change requested: {baud_text}")
            run_async(self._async_set_canfd_baudrate(baud_text))
    
    async def _async_set_canfd_baudrate(self, baud_text):
        if not self.device:
            return
        try:
            self._log(f"Setting CANFD data rate: {baud_text}...")
            baud_map = {
                "1 Mbps": sdk.BaudrateCAN.Baud1Mbps,
                "2 Mbps": sdk.BaudrateCAN.Baud2Mbps,
                "4 Mbps": sdk.BaudrateCAN.Baud4Mbps,
                "5 Mbps": sdk.BaudrateCAN.Baud5Mbps,
            }
            baudrate = baud_map.get(baud_text)
            if baudrate:
                await self.device.set_canfd_baudrate(self.slave_id, baudrate)
                self._log(f"CANFD data rate set to {baud_text} ✓")
        except Exception as e:
            self._log(f"Failed to set CANFD baudrate: {e}")
    
    # ========== Revo2 Finger Settings Handlers ==========
    
    def _load_finger_settings(self):
        """Load Revo2 finger settings from device"""
        if not self._supports_finger_settings():
            return
        self._loading_settings = True
        run_async(self._async_load_finger_settings())
        self._loading_settings = False
    
    async def _async_load_finger_settings(self):
        """Async load finger settings"""
        if not self.device:
            return
        try:
            # Load all finger settings
            all_settings = await self.device.get_all_finger_settings(self.slave_id)
            for i, settings in enumerate(all_settings):
                if i < len(self.finger_min_pos_spins):
                    self.finger_min_pos_spins[i].setValue(settings.min_position)
                    self.finger_max_pos_spins[i].setValue(settings.max_position)
                    self.finger_max_speed_spins[i].setValue(settings.max_speed)
                    self.finger_max_current_spins[i].setValue(settings.max_current)
            
            # Load protected currents
            protected_currents = await self.device.get_finger_protected_currents(self.slave_id)
            for i, current in enumerate(protected_currents):
                if i < len(self.protected_current_spins):
                    self.protected_current_spins[i].setValue(current)
            
            self._log("Finger settings loaded")
        except Exception as e:
            self._log(f"Failed to load finger settings: {e}")
    
    def _apply_all_finger_settings(self):
        """Apply all finger settings to device"""
        if not self._supports_finger_settings():
            return
        run_async(self._async_apply_all_finger_settings())
    
    async def _async_apply_all_finger_settings(self):
        """Async apply all finger settings"""
        if not self.device:
            return
        try:
            for i in range(6):
                finger_id = sdk.FingerId(i + 1)  # FingerId starts from 1
                settings = sdk.MotorSettings(
                    self.finger_min_pos_spins[i].value(),
                    self.finger_max_pos_spins[i].value(),
                    self.finger_max_speed_spins[i].value(),
                    self.finger_max_current_spins[i].value()
                )
                await self.device.set_finger_settings(self.slave_id, finger_id, settings)
            self._log("All finger settings applied")
        except Exception as e:
            self._log(f"Failed to apply finger settings: {e}")
    
    def _apply_protected_currents(self):
        """Apply protected currents to device"""
        if not self._supports_finger_settings():
            return
        run_async(self._async_apply_protected_currents())
    
    async def _async_apply_protected_currents(self):
        """Async apply protected currents"""
        if not self.device:
            return
        try:
            currents = [spin.value() for spin in self.protected_current_spins]
            await self.device.set_finger_protected_currents(self.slave_id, currents)
            self._log("Protected currents applied")
        except Exception as e:
            self._log(f"Failed to apply protected currents: {e}")
    
    # ========== Basic Settings Handlers ==========
    
    def _set_slave_id(self):
        """Set slave ID"""
        if not self.device:
            return
        
        new_id = self.new_slave_id_spin.value()
        self._log(tr("log_setting_slave_id").format(id=new_id))
        run_async(self._async_set_slave_id(new_id))
    
    async def _async_set_slave_id(self, new_id):
        if not self.device:
            return
        try:
            old_id = self.slave_id
            await self.device.set_slave_id(old_id, new_id)
            self._log(tr("log_slave_id_set").format(id=new_id))
            # Notify other components (this will update shared_data.slave_id)
            self.slave_id_changed.emit(new_id)
            self._log(f"Slave ID changed: {old_id} -> {new_id}")
        except Exception as e:
            self._log(tr("log_slave_id_failed").format(error=str(e)))
    
    def _reboot(self):
        """Reboot device"""
        if not self.device:
            return
        
        self._log(tr("log_rebooting"))
        run_async(self._async_reboot())
    
    async def _async_reboot(self):
        if not self.device:
            return
        try:
            await self.device.reboot(self.slave_id)
            self._log(tr("log_rebooted"))
        except Exception as e:
            self._log(tr("log_reboot_failed").format(error=str(e)))
    
    def _factory_reset(self):
        """Factory reset"""
        if not self.device:
            return
        
        reply = QMessageBox.question(
            self,
            tr("confirm"),
            tr("confirm_factory_reset"),
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self._log(tr("log_factory_resetting"))
            run_async(self._async_factory_reset())
    
    async def _async_factory_reset(self):
        if not self.device:
            return
        try:
            await self.device.reset_default_settings(self.slave_id)
            self._log(tr("log_factory_reset_done"))
        except Exception as e:
            self._log(tr("log_factory_reset_failed").format(error=str(e)))
    
    def _log(self, message):
        """Log message to both UI and console"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_msg = f"[{timestamp}] {message}"
        self.log_text.append(log_msg)
        print(f"[SystemConfig] {log_msg}")
