"""Data Collection Panel

Collects motor and touch data to CSV file using high-performance DataCollector.
"""

import sys
import time
from pathlib import Path
from datetime import datetime
from typing import Optional, TYPE_CHECKING

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QSpinBox, QLineEdit,
    QFileDialog, QTextEdit, QCheckBox, QProgressBar
)
from PySide6.QtCore import QTimer, QThread, QObject, Signal

from .i18n import tr
from .constants import MOTOR_COUNT, TOUCH_COUNT

# Add parent directory to path for SDK import
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import sdk

if TYPE_CHECKING:
    from .shared_data import SharedDataManager


class CollectorWorker(QObject):
    """Worker thread for data collection"""
    progress = Signal(int, int, float)  # count, total, elapsed
    log_message = Signal(str)
    finished = Signal(bool, str)  # success, message
    
    def __init__(self, device, slave_id, device_info, save_path, duration, freq, collect_motor, collect_touch):
        super().__init__()
        self.device = device
        self.slave_id = slave_id
        self.device_info = device_info
        self.save_path = save_path
        self.duration = duration
        self.freq = freq
        self.collect_motor = collect_motor
        self.collect_touch = collect_touch
        self.is_running = True
        
        self.motor_buffer = None
        self.touch_buffer = None
        self.data_collector = None
    
    def stop(self):
        self.is_running = False
    
    def run(self):
        """Run data collection"""
        try:
            # Create save directory
            save_dir = Path(self.save_path)
            save_dir.mkdir(parents=True, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = save_dir / f"data_{timestamp}.csv"
            
            self.log_message.emit(f"Saving to: {filename}")
            
            # Determine frequency based on platform
            import platform
            is_linux = platform.system() == "Linux"
            motor_freq = min(self.freq, 200 if is_linux else 60)
            touch_freq = min(10, motor_freq // 5) if self.collect_touch else 0
            
            self.log_message.emit(f"Collection frequency: Motor {motor_freq}Hz" + (f", Touch {touch_freq}Hz" if touch_freq else ""))
            
            # Create buffers
            buffer_size = self.duration * motor_freq * 2  # 2x for safety
            self.motor_buffer = sdk.MotorStatusBuffer(max_size=buffer_size)
            
            has_touch = self.collect_touch and self.device_info and self.device_info.is_touch()
            if has_touch:
                self.touch_buffer = sdk.TouchStatusBuffer(max_size=buffer_size)
                self.data_collector = sdk.DataCollector.new_capacitive(
                    self.device,
                    self.motor_buffer,
                    self.touch_buffer,
                    slave_id=self.slave_id,
                    motor_frequency=motor_freq,
                    touch_frequency=touch_freq,
                    enable_stats=False
                )
            else:
                self.data_collector = sdk.DataCollector.new_basic(
                    self.device,
                    self.motor_buffer,
                    slave_id=self.slave_id,
                    motor_frequency=motor_freq,
                    enable_stats=False
                )
            
            # Start collection
            self.data_collector.start()
            self.log_message.emit("DataCollector started")
            
            # Collect for duration
            start_time = time.time()
            total_samples = self.duration * motor_freq
            
            while self.is_running:
                elapsed = time.time() - start_time
                if elapsed >= self.duration:
                    break
                
                # Report progress
                current_count = self.motor_buffer.len()
                self.progress.emit(current_count, total_samples, elapsed)
                
                time.sleep(0.5)  # Update every 500ms
            
            # Stop collection
            self.data_collector.stop()
            self.data_collector.wait()
            self.log_message.emit("DataCollector stopped")
            
            # Write data to CSV
            self._write_csv(filename, has_touch)
            
            final_count = self.motor_buffer.len()
            self.finished.emit(True, f"Collection completed, {final_count} records")
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.finished.emit(False, f"Collection failed: {e}")
        finally:
            if self.data_collector:
                try:
                    self.data_collector.stop()
                except:
                    pass
    
    def _write_csv(self, filename, has_touch):
        """Write collected data to CSV"""
        self.log_message.emit("Writing CSV...")
        
        # Get all data from buffers
        if not self.motor_buffer:
            self.log_message.emit("No motor buffer available")
            return
        motor_data = self.motor_buffer.pop_all()
        touch_data = None
        if has_touch and self.touch_buffer:
            touch_data = self.touch_buffer.pop_all()
        
        with open(filename, 'w') as f:
            # Write header
            headers = ["index"]
            if self.collect_motor:
                for i in range(MOTOR_COUNT):
                    headers.extend([f"pos_{i}", f"speed_{i}", f"current_{i}"])
            if has_touch:
                for i in range(TOUCH_COUNT):
                    headers.extend([f"touch_normal_{i}", f"touch_tangential_{i}", f"touch_proximity_{i}"])
            f.write(",".join(headers) + "\n")
            
            # Write motor data
            for idx, motor in enumerate(motor_data):
                row = [str(idx)]
                
                if self.collect_motor:
                    positions = list(motor.positions) if motor.positions else [0] * MOTOR_COUNT
                    speeds = list(motor.speeds) if motor.speeds else [0] * MOTOR_COUNT
                    currents = list(motor.currents) if motor.currents else [0] * MOTOR_COUNT
                    for i in range(MOTOR_COUNT):
                        row.extend([
                            str(positions[i] if i < len(positions) else 0),
                            str(speeds[i] if i < len(speeds) else 0),
                            str(currents[i] if i < len(currents) else 0)
                        ])
                
                # Touch data (if available, match by index ratio)
                if has_touch and touch_data:
                    # Touch data is collected at lower frequency, interpolate
                    for finger_idx in range(TOUCH_COUNT):
                        finger_data = touch_data[finger_idx] if finger_idx < len(touch_data) else []
                        if finger_data:
                            # Get closest touch sample
                            touch_idx = min(idx * len(finger_data) // len(motor_data), len(finger_data) - 1)
                            t = finger_data[touch_idx]
                            row.extend([
                                str(t.normal_force1 if hasattr(t, 'normal_force1') else 0),
                                str(t.tangential_force1 if hasattr(t, 'tangential_force1') else 0),
                                str(t.self_proximity1 if hasattr(t, 'self_proximity1') else 0)
                            ])
                        else:
                            row.extend(["0", "0", "0"])
                
                f.write(",".join(row) + "\n")
        
        self.log_message.emit(f"CSV write completed: {len(motor_data)} rows")


class DataCollectorPanel(QWidget):
    """Data Collection Panel
    
    Uses SharedDataManager for device state.
    """
    
    def __init__(self):
        super().__init__()
        self.shared_data: Optional['SharedDataManager'] = None
        self.is_collecting = False
        self.worker: Optional[CollectorWorker] = None
        self._thread: Optional[QThread] = None
        
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
    
    def _setup_ui(self):
        """Setup UI"""
        layout = QVBoxLayout()
        layout.setSpacing(16)
        layout.setContentsMargins(16, 16, 16, 16)
        self.setLayout(layout)
        
        # Collection settings
        self.settings_group = QGroupBox()
        settings_layout = QVBoxLayout()
        settings_layout.setSpacing(12)
        
        # Duration
        duration_layout = QHBoxLayout()
        self.duration_label = QLabel()
        self.duration_label.setMinimumWidth(120)
        duration_layout.addWidget(self.duration_label)
        self.duration_spin = QSpinBox()
        self.duration_spin.setRange(1, 300)
        self.duration_spin.setValue(10)
        self.duration_spin.setSuffix(" s")
        self.duration_spin.setMinimumHeight(32)
        self.duration_spin.setMinimumWidth(100)
        duration_layout.addWidget(self.duration_spin)
        duration_layout.addStretch()
        settings_layout.addLayout(duration_layout)
        
        # Save path
        path_layout = QHBoxLayout()
        self.path_label = QLabel()
        self.path_label.setMinimumWidth(120)
        path_layout.addWidget(self.path_label)
        self.path_edit = QLineEdit()
        self.path_edit.setText(str(Path.home() / "stark_data"))
        self.path_edit.setMinimumWidth(300)
        self.path_edit.setMinimumHeight(32)
        path_layout.addWidget(self.path_edit, 1)
        self.browse_btn = QPushButton()
        self.browse_btn.setMinimumWidth(80)
        self.browse_btn.setMinimumHeight(32)
        self.browse_btn.clicked.connect(self._browse_path)
        path_layout.addWidget(self.browse_btn)
        settings_layout.addLayout(path_layout)
        
        self.settings_group.setLayout(settings_layout)
        layout.addWidget(self.settings_group)
        
        # Data type selection
        self.data_type_group = QGroupBox()
        data_type_layout = QHBoxLayout()
        
        self.motor_check = QCheckBox()
        self.motor_check.setChecked(True)
        data_type_layout.addWidget(self.motor_check)
        
        self.touch_check = QCheckBox()
        self.touch_check.setChecked(False)
        data_type_layout.addWidget(self.touch_check)
        
        data_type_layout.addStretch()
        self.data_type_group.setLayout(data_type_layout)
        layout.addWidget(self.data_type_group)
        
        # Progress
        self.progress_group = QGroupBox()
        progress_layout = QVBoxLayout()
        
        self.status_label = QLabel()
        progress_layout.addWidget(self.status_label)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        progress_layout.addWidget(self.progress_bar)
        
        self.progress_group.setLayout(progress_layout)
        layout.addWidget(self.progress_group)
        
        # Control buttons
        control_layout = QHBoxLayout()
        
        self.start_btn = QPushButton()
        self.start_btn.setMinimumHeight(40)
        self.start_btn.clicked.connect(self._start_collection)
        control_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton()
        self.stop_btn.setMinimumHeight(40)
        self.stop_btn.clicked.connect(self._stop_collection)
        self.stop_btn.setEnabled(False)
        control_layout.addWidget(self.stop_btn)
        
        control_layout.addStretch()
        layout.addLayout(control_layout)
        
        # Log
        self.log_group = QGroupBox()
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        log_layout.addWidget(self.log_text)
        
        self.log_group.setLayout(log_layout)
        layout.addWidget(self.log_group)
        
        layout.addStretch()
    
    def update_texts(self):
        """Update texts"""
        self.settings_group.setTitle(tr("collection_settings"))
        self.duration_label.setText(tr("duration_sec") + ":")
        self.path_label.setText(tr("save_path") + ":")
        self.browse_btn.setText(tr("btn_browse"))
        
        self.data_type_group.setTitle(tr("data_types"))
        self.motor_check.setText(tr("motor_status"))
        self.touch_check.setText(tr("touch_data_type"))
        
        self.progress_group.setTitle(tr("collection_status"))
        if not self.is_collecting:
            self.status_label.setText(tr("status_not_started"))
        
        self.start_btn.setText("▶ " + tr("btn_start_collection"))
        self.stop_btn.setText("⏹ " + tr("btn_stop_collection"))
        
        self.log_group.setTitle(tr("log"))
    
    def set_device(self, device, slave_id, device_info, shared_data=None):
        """Set device - uses SharedDataManager for device state"""
        self.shared_data = shared_data
        
        # Enable/disable touch based on device capability
        if device_info and device_info.is_touch():
            self.touch_check.setEnabled(True)
        else:
            self.touch_check.setEnabled(False)
            self.touch_check.setChecked(False)
        
        # Listen for slave_id changes
        if shared_data:
            shared_data.slave_id_updated.connect(self._on_slave_id_updated)
    
    def _on_slave_id_updated(self, new_id):
        """Handle slave_id change from shared_data"""
        # Worker uses properties, so it will get updated slave_id automatically
        pass
    
    def clear_device(self):
        """Clear device when disconnected"""
        # Disconnect signal
        if self.shared_data:
            try:
                self.shared_data.slave_id_updated.disconnect(self._on_slave_id_updated)
            except RuntimeError:
                pass
        
        self.shared_data = None
    
    def _browse_path(self):
        """Browse path"""
        path = QFileDialog.getExistingDirectory(self, tr("save_path"), self.path_edit.text())
        if path:
            self.path_edit.setText(path)
    
    def _start_collection(self):
        """Start collection"""
        if not self.device:
            self._log(tr("error_no_device"))
            return
        
        if not sdk:
            self._log("SDK not available")
            return
        
        self.is_collecting = True
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.progress_bar.setValue(0)
        
        self._log("Starting collection...")
        
        # Start worker thread
        self._thread = QThread()
        self.worker = CollectorWorker(
            self.device,
            self.slave_id,
            self.device_info,
            self.path_edit.text(),
            self.duration_spin.value(),
            50,  # Default frequency (will be adjusted by worker)
            self.motor_check.isChecked(),
            self.touch_check.isChecked()
        )
        self.worker.moveToThread(self._thread)
        
        self._thread.started.connect(self.worker.run)
        self.worker.progress.connect(self._on_progress)
        self.worker.log_message.connect(self._log)
        self.worker.finished.connect(self._on_finished)
        self.worker.finished.connect(self._thread.quit)
        
        self._thread.start()
    
    def _stop_collection(self):
        """Stop collection"""
        if self.worker:
            self.worker.stop()
        self._log("Stopping...")
    
    def _on_progress(self, count, total, elapsed):
        """Progress update"""
        if total > 0:
            percent = min(100, int(elapsed / self.duration_spin.value() * 100))
            self.progress_bar.setValue(percent)
        self.status_label.setText(f"Collecting: {count} records ({elapsed:.1f}s / {self.duration_spin.value()}s)")
    
    def _on_finished(self, success, message):
        """Collection finished"""
        self.is_collecting = False
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        
        if success:
            self.progress_bar.setValue(100)
            self.status_label.setText("✅ " + message)
        else:
            self.status_label.setText("❌ " + message)
        
        self._log(message)
    
    def _log(self, message):
        """Log message"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
