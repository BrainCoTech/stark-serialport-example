"""Timing Test Panel

Supports two test modes:
1. All Fingers: Test all fingers together (open/close)
2. Single Finger: Test one finger at a time
"""

import asyncio
import time
import sys
import os
from collections import deque
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QSpinBox, QTextEdit, QComboBox,
    QRadioButton, QButtonGroup
)
from PySide6.QtCore import QTimer, QThread, QObject, Signal

# Import from parent's common_imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_imports import sdk, logger, libstark, is_protobuf_device, supports_durations_api

from .i18n import tr
from .constants import MOTOR_COLORS, MOTOR_COUNT, MOTOR_NAMES_EN, FINGER_IDS


try:
    import pyqtgraph as pg
    HAS_PYQTGRAPH = True
except ImportError:
    HAS_PYQTGRAPH = False


# Chart config
MAX_PLOT_POINTS = 500

# Test modes
MODE_ALL_FINGERS = 0
MODE_SINGLE_FINGER = 1

# Single finger test targets (only 4 fingers: Index, Middle, Ring, Pinky)
SINGLE_FINGER_OPTIONS = [
    ('Index', 2),
    ('Middle', 3),
    ('Ring', 4),
    ('Pinky', 5),
]


class TimingTestWorker(QObject):
    """Worker thread for timing test
    
    Uses SharedDataManager for motor status instead of direct API calls.
    """
    log_message = Signal(str)
    data_point = Signal(list)  # positions
    finished = Signal()
    
    def __init__(self, device, slave_id, num_cycles, timeout, test_mode, finger_index, shared_data):
        super().__init__()
        self.device = device
        self.slave_id = slave_id
        self.num_cycles = num_cycles
        self.timeout = timeout
        self.test_mode = test_mode
        self.finger_index = finger_index  # For single finger mode
        self.shared_data = shared_data
        self.is_running = True
    
    @property
    def hw_type(self):
        """Get hardware type from shared_data"""
        return self.shared_data.hw_type if self.shared_data else None
    
    def stop(self):
        self.is_running = False
    
    async def _set_positions(self, positions, durations=None):
        """Set finger positions, using appropriate API based on hardware type
        
        Revo1 motor API (Protobuf/Basic/Touch) doesn't support durations parameter.
        Only Revo2 motor API supports durations.
        """
        if supports_durations_api(self.hw_type):
            if durations is None:
                durations = [1] * 6
            await self.device.set_finger_positions_and_durations(
                self.slave_id, positions, durations
            )
        else:
            await self.device.set_finger_positions(self.slave_id, positions)
    
    def _get_positions(self):
        """Get current positions from SharedDataManager"""
        if not self.shared_data:
            return [0] * MOTOR_COUNT
        motor = self.shared_data.get_latest_motor()
        if motor and hasattr(motor, 'positions') and motor.positions:
            return list(motor.positions)
        return [0] * MOTOR_COUNT
    
    def run(self):
        """Run the timing test"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            if self.test_mode == MODE_ALL_FINGERS:
                loop.run_until_complete(self._run_all_fingers_test())
            else:
                loop.run_until_complete(self._run_single_finger_test())
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.log_message.emit(f"Test error: {e}")
        finally:
            loop.close()
            self.finished.emit()
    
    async def _run_all_fingers_test(self):
        """Run all fingers timing test"""
        self.log_message.emit("=== All Fingers Timing Test ===")
        
        open_positions = [0, 0, 0, 0, 0, 0]
        close_positions = [500, 500, 1000, 1000, 1000, 1000]
        durations = [1] * 6
        
        # Move to initial position
        self.log_message.emit("Moving to initial position...")
        await self._set_positions(open_positions, durations)
        await asyncio.sleep(2.0)
        
        close_times = []
        open_times = []
        
        # Execute test cycles
        for cycle in range(self.num_cycles):
            if not self.is_running:
                break
            
            self.log_message.emit(f"\n--- Cycle {cycle + 1}/{self.num_cycles} ---")
            
            # Close test
            self.log_message.emit("CLOSE: 0% → 100%")
            close_time = await self._measure_all_fingers_movement(
                close_positions, durations
            )
            close_times.append(close_time)
            self.log_message.emit(f"  Close time: {close_time:.3f}s")
            
            # Open test
            self.log_message.emit("OPEN: 100% → 0%")
            open_time = await self._measure_all_fingers_movement(
                open_positions, durations
            )
            open_times.append(open_time)
            self.log_message.emit(f"  Open time: {open_time:.3f}s")
        
        # Show results
        self._show_results("All Fingers", close_times, open_times)
    
    async def _run_single_finger_test(self):
        """Run single finger timing test"""
        finger_name = MOTOR_NAMES_EN[self.finger_index]
        self.log_message.emit(f"=== Single Finger Test: {finger_name} ===")
        
        # All fingers start at 0
        open_positions = [0, 0, 0, 0, 0, 0]
        close_positions = [0, 0, 0, 0, 0, 0]
        close_positions[self.finger_index] = 1000  # Only test finger moves
        durations = [1] * 6
        
        # Move to initial position
        self.log_message.emit("Moving to initial position...")
        await self._set_positions(open_positions, durations)
        await asyncio.sleep(2.0)
        
        close_times = []
        open_times = []
        
        # Execute test cycles
        for cycle in range(self.num_cycles):
            if not self.is_running:
                break
            
            self.log_message.emit(f"\n--- Cycle {cycle + 1}/{self.num_cycles} ---")
            
            # Close test
            self.log_message.emit(f"CLOSE: {finger_name} 0% → 100%")
            close_time = await self._measure_single_finger_movement(
                close_positions, durations, self.finger_index
            )
            close_times.append(close_time)
            self.log_message.emit(f"  Close time: {close_time:.3f}s")
            
            # Open test
            self.log_message.emit(f"OPEN: {finger_name} 100% → 0%")
            open_time = await self._measure_single_finger_movement(
                open_positions, durations, self.finger_index
            )
            open_times.append(open_time)
            self.log_message.emit(f"  Open time: {open_time:.3f}s")
        
        # Show results
        self._show_results(finger_name, close_times, open_times)
    
    def _show_results(self, name, close_times, open_times):
        """Show test results"""
        if close_times and open_times:
            self.log_message.emit(f"\n{'='*50}")
            self.log_message.emit(f"{name} Timing Test Results")
            self.log_message.emit(f"{'='*50}")
            self.log_message.emit(f"Cycles: {len(close_times)}")
            self.log_message.emit(f"\nCLOSE (0% → 100%):")
            self.log_message.emit(f"  Average: {sum(close_times)/len(close_times):.3f}s")
            self.log_message.emit(f"  Min: {min(close_times):.3f}s, Max: {max(close_times):.3f}s")
            self.log_message.emit(f"\nOPEN (100% → 0%):")
            self.log_message.emit(f"  Average: {sum(open_times)/len(open_times):.3f}s")
            self.log_message.emit(f"  Min: {min(open_times):.3f}s, Max: {max(open_times):.3f}s")
            self.log_message.emit(f"{'='*50}")
    
    async def _measure_all_fingers_movement(self, target_positions, durations):
        """Measure all fingers movement time"""
        start_time = time.time()
        await self._set_positions(target_positions, durations)
        
        while self.is_running:
            await asyncio.sleep(0.02)
            elapsed = time.time() - start_time
            
            try:
                positions = self._get_positions()
                self.data_point.emit(positions)
                
                # Check if all fingers reached target
                all_reached = True
                for i in range(MOTOR_COUNT):
                    target = target_positions[i]
                    current = positions[i]
                    
                    if target == 0:
                        if current > 50:
                            all_reached = False
                            break
                    else:
                        threshold = target * 0.95
                        if current < threshold:
                            all_reached = False
                            break
                
                if all_reached:
                    return elapsed
            except Exception as e:
                pass
            
            if elapsed >= self.timeout:
                return elapsed
        
        return time.time() - start_time
    
    async def _measure_single_finger_movement(self, target_positions, durations, finger_idx):
        """Measure single finger movement time"""
        start_time = time.time()
        await self._set_positions(target_positions, durations)
        
        target = target_positions[finger_idx]
        
        while self.is_running:
            await asyncio.sleep(0.02)
            elapsed = time.time() - start_time
            
            try:
                positions = self._get_positions()
                self.data_point.emit(positions)
                
                current = positions[finger_idx]
                
                # Check if target reached (98% threshold)
                reached = False
                if target == 0:
                    reached = current <= 50  # 5% tolerance
                else:
                    threshold = target * 0.98
                    reached = current >= threshold
                
                if reached:
                    return elapsed
            except Exception as e:
                pass
            
            if elapsed >= self.timeout:
                return elapsed
        
        return time.time() - start_time


class TimingTestPanel(QWidget):
    """Timing Test Panel
    
    Uses SharedDataManager for device state.
    """
    
    def __init__(self):
        super().__init__()
        self.shared_data = None
        self.worker = None
        self.thread = None
        self.is_running = False
        
        # Data storage
        self.times = deque(maxlen=MAX_PLOT_POINTS)
        self.positions = [deque(maxlen=MAX_PLOT_POINTS) for _ in range(MOTOR_COUNT)]
        self.start_time = None
        
        # Statistics
        self.packet_count = 0
        self.error_count = 0
        self._last_packet_times = deque(maxlen=100)
        self._last_latencies = deque(maxlen=100)
        
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
        self.setLayout(layout)
        
        # Test mode selection
        self.mode_group = QGroupBox("Test Mode")
        mode_layout = QHBoxLayout()
        
        self.mode_btn_group = QButtonGroup(self)
        
        self.all_fingers_radio = QRadioButton("All Fingers")
        self.mode_btn_group.addButton(self.all_fingers_radio, MODE_ALL_FINGERS)
        mode_layout.addWidget(self.all_fingers_radio)
        
        self.single_finger_radio = QRadioButton("Single Finger")
        self.single_finger_radio.setChecked(True)
        self.mode_btn_group.addButton(self.single_finger_radio, MODE_SINGLE_FINGER)
        mode_layout.addWidget(self.single_finger_radio)
        
        # Finger selection (for single finger mode)
        self.finger_label = QLabel("Finger:")
        mode_layout.addWidget(self.finger_label)
        
        self.finger_combo = QComboBox()
        for name, idx in SINGLE_FINGER_OPTIONS:
            self.finger_combo.addItem(name, idx)
        self.finger_combo.setMinimumWidth(100)
        mode_layout.addWidget(self.finger_combo)
        
        mode_layout.addStretch()
        self.mode_group.setLayout(mode_layout)
        layout.addWidget(self.mode_group)
        
        # Connect mode change
        self.mode_btn_group.buttonClicked.connect(self._on_mode_changed)
        
        # Test config
        self.config_group = QGroupBox()
        config_layout = QHBoxLayout()
        
        self.cycles_label = QLabel()
        config_layout.addWidget(self.cycles_label)
        
        self.cycles_spin = QSpinBox()
        self.cycles_spin.setRange(1, 100)
        self.cycles_spin.setValue(5)
        config_layout.addWidget(self.cycles_spin)
        
        self.timeout_label = QLabel()
        config_layout.addWidget(self.timeout_label)
        
        self.timeout_spin = QSpinBox()
        self.timeout_spin.setRange(1, 10)
        self.timeout_spin.setValue(2)
        config_layout.addWidget(self.timeout_spin)
        
        config_layout.addSpacing(20)
        
        # Time window selection for chart
        self.time_window_label = QLabel("Time Window:")
        config_layout.addWidget(self.time_window_label)
        
        self.time_window_combo = QComboBox()
        for t in [5, 10, 30, 60]:
            self.time_window_combo.addItem(f"{t}s", t)
        self.time_window_combo.setCurrentIndex(1)  # Default 10s
        self.time_window_combo.currentIndexChanged.connect(self._on_time_window_changed)
        config_layout.addWidget(self.time_window_combo)
        
        config_layout.addStretch()
        self.config_group.setLayout(config_layout)
        layout.addWidget(self.config_group)
        
        # Control buttons
        control_layout = QHBoxLayout()
        
        self.start_btn = QPushButton()
        self.start_btn.clicked.connect(self._start_test)
        control_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton()
        self.stop_btn.clicked.connect(self._stop_test)
        self.stop_btn.setEnabled(False)
        control_layout.addWidget(self.stop_btn)
        
        self.clear_btn = QPushButton()
        self.clear_btn.clicked.connect(self._clear_data)
        control_layout.addWidget(self.clear_btn)
        
        control_layout.addStretch()
        
        # Statistics display
        self.stats_group = QGroupBox("Statistics")
        stats_layout = QHBoxLayout(self.stats_group)
        stats_layout.setSpacing(20)
        
        # Frequency
        freq_layout = QVBoxLayout()
        self.freq_title = QLabel("Frequency")
        self.freq_title.setStyleSheet("color: #888;")
        freq_layout.addWidget(self.freq_title)
        self.freq_value = QLabel("0.0 Hz")
        self.freq_value.setStyleSheet("font-weight: bold; font-size: 14px;")
        freq_layout.addWidget(self.freq_value)
        stats_layout.addLayout(freq_layout)
        
        # Latency
        latency_layout = QVBoxLayout()
        self.latency_title = QLabel("Avg Latency")
        self.latency_title.setStyleSheet("color: #888;")
        latency_layout.addWidget(self.latency_title)
        self.latency_value = QLabel("0.0 ms")
        self.latency_value.setStyleSheet("font-weight: bold; font-size: 14px;")
        latency_layout.addWidget(self.latency_value)
        stats_layout.addLayout(latency_layout)
        
        # Packets
        packets_layout = QVBoxLayout()
        self.packets_title = QLabel("Packets")
        self.packets_title.setStyleSheet("color: #888;")
        packets_layout.addWidget(self.packets_title)
        self.packets_value = QLabel("0")
        self.packets_value.setStyleSheet("font-weight: bold; font-size: 14px;")
        packets_layout.addWidget(self.packets_value)
        stats_layout.addLayout(packets_layout)
        
        # Errors
        errors_layout = QVBoxLayout()
        self.errors_title = QLabel("Errors")
        self.errors_title.setStyleSheet("color: #888;")
        errors_layout.addWidget(self.errors_title)
        self.errors_value = QLabel("0")
        self.errors_value.setStyleSheet("font-weight: bold; font-size: 14px; color: #dc3545;")
        errors_layout.addWidget(self.errors_value)
        stats_layout.addLayout(errors_layout)
        
        stats_layout.addStretch()
        control_layout.addWidget(self.stats_group)
        
        layout.addLayout(control_layout)
        
        # Chart
        if HAS_PYQTGRAPH:
            self.plot_widget = pg.PlotWidget()
            self.plot_widget.setBackground('#1a1a2e')
            self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
            self.plot_widget.setYRange(-50, 1050)
            
            # Add target lines
            self.plot_widget.addLine(y=500, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine, width=2))
            self.plot_widget.addLine(y=1000, pen=pg.mkPen('g', style=pg.QtCore.Qt.DashLine, width=2))
            self.plot_widget.addLine(y=0, pen=pg.mkPen('b', style=pg.QtCore.Qt.DashLine, width=2))
            
            # Create curves
            self.curves = []
            for i, color in enumerate(MOTOR_COLORS):
                pen = pg.mkPen(color=color, width=2)
                curve = self.plot_widget.plot([], [], pen=pen)
                self.curves.append(curve)
            
            layout.addWidget(self.plot_widget)
        else:
            self.plot_label = QLabel("pyqtgraph not installed - no chart available")
            layout.addWidget(self.plot_label)
            self.curves = []
        
        # Results display
        self.result_group = QGroupBox()
        result_layout = QVBoxLayout()
        
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMaximumHeight(200)
        result_layout.addWidget(self.result_text)
        
        self.result_group.setLayout(result_layout)
        layout.addWidget(self.result_group)
    
    def _on_mode_changed(self, button):
        """Test mode changed"""
        is_single = self.mode_btn_group.checkedId() == MODE_SINGLE_FINGER
        self.finger_label.setEnabled(is_single)
        self.finger_combo.setEnabled(is_single)
    
    def update_texts(self):
        """Update texts for i18n"""
        from .i18n import get_i18n
        lang = get_i18n().current_language
        
        self.mode_group.setTitle(tr("test_mode") if tr("test_mode") != "test_mode" else "Test Mode")
        self.config_group.setTitle(tr("test_config"))
        self.cycles_label.setText(tr("num_cycles") + ":")
        self.timeout_label.setText(tr("timeout_sec") + ":")
        
        self.start_btn.setText(tr("btn_start_test"))
        self.stop_btn.setText(tr("btn_stop_test"))
        self.clear_btn.setText(tr("btn_clear"))
        
        self.result_group.setTitle(tr("test_results"))
        
        # Update chart labels
        if HAS_PYQTGRAPH:
            self.plot_widget.setLabel('left', tr("position"), color='w')
            self.plot_widget.setLabel('bottom', tr("time_sec"), color='w')
            self.plot_widget.setTitle(tr("position_tracking"), color='w')
    
    def set_device(self, device, slave_id, device_info, shared_data=None):
        """Set device"""
        self.shared_data = shared_data
    
    def _clear_data(self):
        """Clear data"""
        self.times.clear()
        for pos in self.positions:
            pos.clear()
        self.start_time = None
        self.result_text.clear()
        
        # Clear statistics
        self.packet_count = 0
        self.error_count = 0
        self._last_packet_times.clear()
        self._last_latencies.clear()
        self._update_stats()
        
        self._update_plot()
    
    def _update_stats(self):
        """Update statistics display"""
        # Calculate frequency
        freq = 0.0
        if len(self._last_packet_times) >= 2:
            times = list(self._last_packet_times)
            duration = times[-1] - times[0]
            if duration > 0:
                freq = (len(times) - 1) / duration
        
        # Calculate average latency
        avg_latency = 0.0
        if self._last_latencies:
            avg_latency = sum(self._last_latencies) / len(self._last_latencies) * 1000
        
        # Update labels
        self.freq_value.setText(f"{freq:.1f} Hz")
        self.latency_value.setText(f"{avg_latency:.1f} ms")
        self.packets_value.setText(str(self.packet_count))
        self.errors_value.setText(str(self.error_count))
    
    def _record_packet(self, latency: float = 0):
        """Record a successful packet for statistics"""
        self.packet_count += 1
        self._last_packet_times.append(time.time())
        if latency > 0:
            self._last_latencies.append(latency)
    
    def _record_error(self):
        """Record an error for statistics"""
        self.error_count += 1
    
    def _update_plot(self):
        """Update chart"""
        if not self.curves:
            return
        
        times = list(self.times)
        if not times:
            for curve in self.curves:
                curve.setData([], [])
            return
        
        for i, curve in enumerate(self.curves):
            curve.setData(times, list(self.positions[i]))
        
        # Auto-scale X axis
        if len(times) > 1:
            # Use selected time window
            time_window = self.time_window_combo.currentData() or 10
            x_min = max(0, times[-1] - time_window)
            x_max = times[-1] + 0.5
            self.plot_widget.setXRange(x_min, x_max)
    
    def _on_time_window_changed(self, index):
        """Time window selection changed"""
        self._update_plot()
    
    def _add_data(self, positions):
        """Add data point"""
        if self.start_time is None:
            self.start_time = time.time()
        
        current_time = time.time() - self.start_time
        self.times.append(current_time)
        for i in range(min(MOTOR_COUNT, len(positions))):
            self.positions[i].append(positions[i])
        
        # Record packet for statistics
        self._record_packet()
        self._update_stats()
        
        # Update plot immediately
        self._update_plot()
    
    def _log(self, message):
        """Log message"""
        self.result_text.append(message)
    
    def _start_test(self):
        """Start test"""
        if not self.device:
            self._log(tr("error_no_device"))
            return
        
        self.is_running = True
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self._clear_data()
        
        # Get test mode and finger
        test_mode = self.mode_btn_group.checkedId()
        finger_index = self.finger_combo.currentData() if test_mode == MODE_SINGLE_FINGER else 0
        
        # Start worker thread
        self.thread = QThread()
        self.worker = TimingTestWorker(
            self.device, 
            self.slave_id,
            self.cycles_spin.value(),
            self.timeout_spin.value(),
            test_mode,
            finger_index,
            self.shared_data
        )
        self.worker.moveToThread(self.thread)
        
        self.thread.started.connect(self.worker.run)
        self.worker.log_message.connect(self._log)
        self.worker.data_point.connect(self._add_data)
        self.worker.finished.connect(self._on_test_finished)
        self.worker.finished.connect(self.thread.quit)
        
        self.thread.start()
    
    def _stop_test(self):
        """Stop test"""
        if self.worker:
            self.worker.stop()
        self.is_running = False
    
    def _on_test_finished(self):
        """Test finished"""
        self.is_running = False
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
