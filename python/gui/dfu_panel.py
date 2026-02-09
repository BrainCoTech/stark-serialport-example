"""DFU - Device Firmware Upgrade Panel"""

import asyncio
import sys
import os
from pathlib import Path

from typing import Optional, TYPE_CHECKING
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QLineEdit,
    QFileDialog, QProgressBar, QComboBox, QMessageBox
)
from PySide6.QtCore import Qt, Signal, QThread, QObject, QTimer

from .i18n import tr
from .styles import COLORS

# Add parent directory to path for SDK import
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import sdk, logger

if TYPE_CHECKING:
    from .shared_data import SharedDataManager

# DfuState enum from SDK
DfuState = sdk.DfuState

# Firmware paths configuration
SCRIPT_DIR = Path(__file__).resolve().parent
OTA_DIR = SCRIPT_DIR.parent / "ota_bin"

# Default firmware file paths
DEFAULT_FIRMWARE_PATHS = {
    "revo1_basic": OTA_DIR / "modbus" / "FW_MotorController_Release_SecureOTA_0.1.7.C.ota",
    "revo1_touch": OTA_DIR / "touch" / "FW_MotorController_Release_SecureOTA_V1.8.53.F.ota",
    "revo1_advanced": OTA_DIR / "stark2" / "Revo1.8_V1.0.3.C_2602031800.bin",
    "revo2_485_canfd": OTA_DIR / "stark2" / "Revo2_V1.0.20.U_2601091030.bin",
}

# DFU State names
DFU_STATE_NAMES = {
    0: "空闲",
    1: "启动中",
    2: "已启动",
    3: "传输中",
    4: "已完成",
    5: "已中止",
}

# Firmware type mapping
FIRMWARE_TYPES = {
    "Revo1Basic": {"name": "Revo1 Basic", "default_path": DEFAULT_FIRMWARE_PATHS.get("revo1_basic")},
    "Revo1Touch": {"name": "Revo1 Touch", "default_path": DEFAULT_FIRMWARE_PATHS.get("revo1_touch")},
    "Revo1Advanced": {"name": "Revo1 Advanced", "default_path": DEFAULT_FIRMWARE_PATHS.get("revo1_advanced")},
    "Revo1AdvancedTouch": {"name": "Revo1 Advanced Touch", "default_path": DEFAULT_FIRMWARE_PATHS.get("revo1_advanced")},
    "Revo2Basic": {"name": "Revo2 Basic", "default_path": DEFAULT_FIRMWARE_PATHS.get("revo2_485_canfd")},
    "Revo2Touch": {"name": "Revo2 Touch", "default_path": DEFAULT_FIRMWARE_PATHS.get("revo2_485_canfd")},
    "Revo2TouchPressure": {"name": "Revo2 Touch Pressure", "default_path": DEFAULT_FIRMWARE_PATHS.get("revo2_485_canfd")},
}


class DfuWorker(QObject):
    """DFU upgrade worker thread"""
    progress = Signal(int)  # percent 0-100
    state_changed = Signal(int)  # state enum value
    finished = Signal(bool, str)  # success, message

    def __init__(self, device, slave_id, firmware_path):
        super().__init__()
        self.device = device
        self.slave_id = slave_id
        self.firmware_path = firmware_path
        self.last_state = 0
        self.dfu_completed = False
        self.dfu_failed = False

    def run(self):
        """Execute DFU upgrade with event loop"""
        try:
            print(f"\n[DFU] Starting upgrade: {self.firmware_path}")

            # Create and set event loop for this thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            try:
                # start_dfu returns a coroutine, need to await it
                async def run_dfu_async():
                    # Call start_dfu and await the result
                    await self.device.start_dfu(
                        self.slave_id,
                        self.firmware_path,
                        180,  # wait_secs (3 minutes timeout)
                        self._on_dfu_state,
                        self._on_dfu_progress
                    )

                    # Wait for final state (Completed or Aborted)
                    # start_dfu may return before final state callback arrives
                    # Wait up to 60 seconds for completion
                    wait_count = 0
                    while not self.dfu_completed and not self.dfu_failed and wait_count < 120:
                        await asyncio.sleep(0.5)
                        wait_count += 1
                        if wait_count % 10 == 0:
                            print(f"[DFU] Waiting for completion... ({wait_count * 0.5}s)")

                loop.run_until_complete(run_dfu_async())
            finally:
                loop.close()

            print(f"\n[DFU] Upgrade finished, state: {DFU_STATE_NAMES.get(self.last_state, self.last_state)}")

            # Check final state
            if self.dfu_completed:
                self.finished.emit(True, "固件升级成功！设备将自动重启。")
            elif self.dfu_failed:
                self.finished.emit(False, "固件升级被中止")
            else:
                self.finished.emit(False, f"升级超时，状态: {DFU_STATE_NAMES.get(self.last_state, self.last_state)}")

        except Exception as e:
            import traceback
            traceback.print_exc()
            self.finished.emit(False, f"升级失败: {e}")

    def _on_dfu_state(self, slave_id, state):
        """DFU state callback - called from SDK thread"""
        try:
            # Convert to DfuState enum if it's an int
            if isinstance(state, int):
                dfu_state = DfuState(state)
                state_val = state
            else:
                dfu_state = state
                state_val = dfu_state.int_value

            self.last_state = state_val
            state_name = DFU_STATE_NAMES.get(state_val, str(dfu_state))
            print(f"[DFU] State: {state_name}")

            # Track completion/failure using enum
            if dfu_state == DfuState.Completed:
                self.dfu_completed = True
            elif dfu_state == DfuState.Aborted:
                self.dfu_failed = True

            # Emit signal (thread-safe via Qt queued connection)
            self.state_changed.emit(state_val)
        except Exception as e:
            print(f"[DFU] State callback error: {e}")

    def _on_dfu_progress(self, slave_id, progress):
        """DFU progress callback - called from SDK thread"""
        try:
            # Progress is a float 0.0-1.0, convert to percent
            if isinstance(progress, float) and progress <= 1.0:
                percent = int(progress * 100)
            else:
                percent = int(progress)

            # Print progress like hand_dfu.py
            if percent >= 100:
                print(f"\r[DFU] Slave {slave_id} progress: 100%")
            else:
                print(f"[DFU] Slave {slave_id} progress: {percent}%\r", end="", flush=True)

            # Emit signal (thread-safe via Qt queued connection)
            self.progress.emit(percent)
        except Exception as e:
            print(f"[DFU] Progress callback error: {e}")


class DfuPanel(QWidget):
    """DFU Firmware Upgrade Panel
    
    Uses SharedDataManager for device state.
    """

    dfu_started = Signal()
    dfu_finished = Signal(bool)  # success

    def __init__(self):
        super().__init__()
        self.shared_data: Optional['SharedDataManager'] = None
        self.worker: Optional[DfuWorker] = None
        self._thread: Optional[QThread] = None

        self._setup_ui()
    
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
        layout = QVBoxLayout(self)
        layout.setSpacing(16)
        layout.setContentsMargins(16, 16, 16, 16)

        # Warning banner
        warning_frame = QGroupBox()
        warning_frame.setStyleSheet("""
            QGroupBox {
                background-color: #fff3cd;
                border: 1px solid #ffc107;
                border-radius: 8px;
                padding: 12px;
            }
        """)
        warning_layout = QVBoxLayout(warning_frame)

        self.warning_title_label = QLabel(tr("dfu_warning_title"))
        self.warning_title_label.setStyleSheet("font-weight: bold; color: #856404;")
        warning_layout.addWidget(self.warning_title_label)

        self.warning_text_label = QLabel()
        self.warning_text_label.setStyleSheet("color: #856404;")
        self.warning_text_label.setWordWrap(True)
        warning_layout.addWidget(self.warning_text_label)

        layout.addWidget(warning_frame)

        # Device info
        self.device_group = QGroupBox(tr("device_info"))
        device_layout = QHBoxLayout(self.device_group)

        self.device_type_label = QLabel(tr("device_type") + ": --")
        device_layout.addWidget(self.device_type_label)

        self.firmware_version_label = QLabel(tr("current_firmware") + ": --")
        device_layout.addWidget(self.firmware_version_label)

        device_layout.addStretch()
        layout.addWidget(self.device_group)

        # Firmware selection
        self.firmware_group = QGroupBox(tr("firmware_file"))
        firmware_layout = QVBoxLayout(self.firmware_group)

        # Firmware type
        type_layout = QHBoxLayout()
        self.firmware_type_label = QLabel(tr("hardware_type") + ":")
        type_layout.addWidget(self.firmware_type_label)

        self.firmware_type_combo = QComboBox()
        for key, info in FIRMWARE_TYPES.items():
            self.firmware_type_combo.addItem(info["name"], key)
        self.firmware_type_combo.currentIndexChanged.connect(self._on_type_changed)
        type_layout.addWidget(self.firmware_type_combo, 1)

        firmware_layout.addLayout(type_layout)

        # Firmware file
        file_layout = QHBoxLayout()
        self.firmware_file_label = QLabel(tr("firmware_file") + ":")
        file_layout.addWidget(self.firmware_file_label)

        self.firmware_path_edit = QLineEdit()
        self.firmware_path_edit.setPlaceholderText(tr("dfu_select_file") + " (.bin)")
        file_layout.addWidget(self.firmware_path_edit, 1)

        self.browse_btn = QPushButton(tr("btn_browse"))
        self.browse_btn.clicked.connect(self._browse_firmware)
        file_layout.addWidget(self.browse_btn)

        firmware_layout.addLayout(file_layout)
        layout.addWidget(self.firmware_group)

        # Progress section
        self.progress_group = QGroupBox(tr("dfu_progress"))
        progress_layout = QVBoxLayout(self.progress_group)

        # Status label (large, prominent)
        self.status_label = QLabel(tr("dfu_status_waiting"))
        self.status_label.setStyleSheet(f"""
            font-size: 16px;
            font-weight: bold;
            color: {COLORS['text_primary']};
            padding: 8px;
        """)
        self.status_label.setAlignment(Qt.AlignCenter)
        progress_layout.addWidget(self.status_label)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setMinimumHeight(30)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #3498db;
                border-radius: 5px;
                text-align: center;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #3498db;
            }
        """)
        progress_layout.addWidget(self.progress_bar)

        # State label
        self.state_label = QLabel(tr("dfu_status_idle"))
        self.state_label.setStyleSheet(f"color: {COLORS['text_muted']};")
        self.state_label.setAlignment(Qt.AlignCenter)
        progress_layout.addWidget(self.state_label)

        layout.addWidget(self.progress_group)

        # Control buttons
        btn_layout = QHBoxLayout()
        
        self.start_btn = QPushButton(tr("btn_start_upgrade"))
        self.start_btn.setMinimumHeight(50)
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: #28a745;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #218838;
            }
            QPushButton:disabled {
                background-color: #6c757d;
            }
        """)
        self.start_btn.clicked.connect(self._start_upgrade)
        btn_layout.addWidget(self.start_btn, 3)
        
        # Reset button (for stuck DFU state)
        self.reset_btn = QPushButton(tr("btn_reset_state"))
        self.reset_btn.setMinimumHeight(50)
        self.reset_btn.setToolTip(tr("dfu_reset_tooltip"))
        self.reset_btn.setStyleSheet("""
            QPushButton {
                background-color: #6c757d;
                color: white;
                font-size: 14px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #5a6268;
            }
        """)
        self.reset_btn.clicked.connect(self._reset_dfu_state)
        btn_layout.addWidget(self.reset_btn, 1)
        
        layout.addLayout(btn_layout)

        layout.addStretch()

        # Initialize
        self._on_type_changed(0)
        self._update_warning_text()

    def _update_warning_text(self):
        """Update warning text based on current language"""
        self.warning_text_label.setText(
            f"• {tr('dfu_warning_1')}\n"
            f"• {tr('dfu_warning_2')}\n"
            f"• {tr('dfu_warning_3')}"
        )

    def update_texts(self):
        """Update texts for i18n"""
        self.warning_title_label.setText(tr("dfu_warning_title"))
        self._update_warning_text()
        self.device_group.setTitle(tr("device_info"))
        self.firmware_group.setTitle(tr("firmware_file"))
        self.firmware_type_label.setText(tr("hardware_type") + ":")
        self.firmware_file_label.setText(tr("firmware_file") + ":")
        self.browse_btn.setText(tr("btn_browse"))
        self.progress_group.setTitle(tr("dfu_progress"))
        self.start_btn.setText(tr("btn_start_upgrade"))
        self.reset_btn.setText(tr("btn_reset_state"))
        self.reset_btn.setToolTip(tr("dfu_reset_tooltip"))

    def set_device(self, device, slave_id, device_info, shared_data=None):
        """Set device"""
        self.shared_data = shared_data

        if device_info:
            hw_type = str(device_info.hardware_type).replace("StarkHardwareType.", "")
            self.device_type_label.setText(f"{tr('device_type')}: {hw_type}")
            self.firmware_version_label.setText(f"{tr('current_firmware')}: {device_info.firmware_version}")

            # Auto-select firmware type
            for i in range(self.firmware_type_combo.count()):
                key = self.firmware_type_combo.itemData(i)
                if key and hw_type in key:
                    self.firmware_type_combo.setCurrentIndex(i)
                    break
        else:
            self.device_type_label.setText("设备类型: --")
            self.firmware_version_label.setText("当前固件: --")

    def _on_type_changed(self, index):
        """Firmware type changed"""
        key = self.firmware_type_combo.currentData()
        if key and key in FIRMWARE_TYPES:
            info = FIRMWARE_TYPES[key]
            default_path = info.get("default_path")
            if default_path and default_path.exists():
                self.firmware_path_edit.setText(str(default_path))
            else:
                self.firmware_path_edit.clear()

    def _browse_firmware(self):
        """Browse for firmware file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "选择固件文件", "", "Binary Files (*.bin *.ota);;All Files (*)"
        )
        if filename:
            self.firmware_path_edit.setText(filename)

    def _start_upgrade(self):
        """Start firmware upgrade"""
        if not self.device:
            QMessageBox.warning(self, "错误", "请先连接设备")
            return

        firmware_path = self.firmware_path_edit.text()
        if not firmware_path or not os.path.exists(firmware_path):
            QMessageBox.warning(self, "错误", "请选择有效的固件文件")
            return

        # Confirm
        reply = QMessageBox.question(
            self, "确认升级",
            "确定要开始固件升级吗？\n\n升级过程中请勿断开连接或关闭电源。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply != QMessageBox.Yes:
            return

        # Disable UI
        self.start_btn.setEnabled(False)
        self.browse_btn.setEnabled(False)
        self.firmware_type_combo.setEnabled(False)

        # Reset progress
        self.progress_bar.setValue(0)
        self.status_label.setText("正在升级...")
        self.status_label.setStyleSheet(f"font-size: 16px; font-weight: bold; color: #3498db; padding: 8px;")
        self.state_label.setText("状态: 启动中")

        # Start worker
        self._thread = QThread()
        self.worker = DfuWorker(self.device, self.slave_id, firmware_path)
        self.worker.moveToThread(self._thread)

        self._thread.started.connect(self.worker.run)
        self.worker.progress.connect(self._on_progress)
        self.worker.state_changed.connect(self._on_state_changed)
        self.worker.finished.connect(self._on_finished)
        self.worker.finished.connect(self._thread.quit)

        self.dfu_started.emit()
        self._thread.start()

    def _on_progress(self, percent):
        """Progress update"""
        self.progress_bar.setValue(percent)
        self.status_label.setText(f"正在升级... {percent}%")

    def _on_state_changed(self, state):
        """DFU state changed"""
        state_name = DFU_STATE_NAMES.get(state, f"未知({state})")
        self.state_label.setText(f"状态: {state_name}")

    def _on_finished(self, success, message):
        """Upgrade finished"""
        # Re-enable UI
        self.start_btn.setEnabled(True)
        self.browse_btn.setEnabled(True)
        self.firmware_type_combo.setEnabled(True)

        self.dfu_finished.emit(success)

        if success:
            self.progress_bar.setValue(100)
            self.status_label.setText("✅ 升级成功，设备将自动重启")
            self.status_label.setStyleSheet(f"font-size: 16px; font-weight: bold; color: #28a745; padding: 8px;")
            self.state_label.setText("状态: 已完成")
            # No dialog - just show status in panel
        else:
            self.status_label.setText("❌ 升级失败")
            self.status_label.setStyleSheet(f"font-size: 16px; font-weight: bold; color: #dc3545; padding: 8px;")
            # Delay dialog to let UI update first
            QTimer.singleShot(100, lambda: QMessageBox.warning(self, "升级失败", message))

    def _reset_dfu_state(self):
        """Reset DFU state (for recovery from stuck states)"""
        if not self.device:
            QMessageBox.warning(self, "错误", "请先连接设备")
            return
        
        try:
            import asyncio
            device = self.device
            slave_id = self.slave_id
            
            async def do_reset():
                await device.reset_dfu_state(slave_id)
            
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(do_reset())
            finally:
                loop.close()
            
            # Reset UI
            self.progress_bar.setValue(0)
            self.status_label.setText("状态已重置")
            self.status_label.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {COLORS['text_primary']}; padding: 8px;")
            self.state_label.setText("状态: 空闲")
            
            QMessageBox.information(self, "成功", "DFU状态已重置，可以重新开始升级")
        except Exception as e:
            import traceback
            traceback.print_exc()
            QMessageBox.warning(self, "错误", f"重置失败: {e}")
