"""Real-time Monitor Panel

Uses DataCollector + MotorStatusBuffer for high-performance motor status polling.
This avoids GIL overhead from async callbacks and provides better real-time performance.
"""

import sys
import time
from collections import deque
from typing import List, Optional, TYPE_CHECKING, Sequence
from pathlib import Path

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QGridLayout,
    QSplitter, QFrame
)
from PySide6.QtCore import Qt, QTimer

if TYPE_CHECKING:
    from .shared_data import SharedDataManager

from .i18n import tr
from .styles import COLORS
from .hand_visualization import HandVisualization
from .constants import (
    MOTOR_COLORS, TOUCH_COLORS, MOTOR_COUNT, TOUCH_COUNT,
    MOTOR_BUFFER_SIZE, TOUCH_BUFFER_SIZE, TOUCH_NAMES_EN,
    TOUCH_SENSOR_CONFIG, get_touch_force_value
)


try:
    import pyqtgraph as pg
    HAS_PYQTGRAPH = True
except ImportError:
    pg = None  # type: ignore
    HAS_PYQTGRAPH = False

# Add parent directory to path for SDK import
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import sdk


TIME_WINDOWS = [5, 10, 30, 60]  # seconds
UPDATE_RATES = [10, 20, 50, 100]  # Hz (UI update rate)


class DataManager:
    """Manages data buffers and statistics for real-time visualization"""

    def __init__(self, max_points: int = 500):
        self.max_points = max_points

        # Time series data
        self.times: deque = deque(maxlen=max_points)

        # Motor data (6 DOF: Thumb, ThumbAux, Index, Middle, Ring, Pinky)
        self.motor_positions: List[deque] = [deque(maxlen=max_points) for _ in range(MOTOR_COUNT)]
        self.motor_speeds: List[deque] = [deque(maxlen=max_points) for _ in range(MOTOR_COUNT)]
        self.motor_currents: List[deque] = [deque(maxlen=max_points) for _ in range(MOTOR_COUNT)]

        # Touch data (5 fingers: Thumb, Index, Middle, Ring, Pinky - no ThumbAux)
        self.touch_normal: List[deque] = [deque(maxlen=max_points) for _ in range(TOUCH_COUNT)]
        self.touch_tangential: List[deque] = [deque(maxlen=max_points) for _ in range(TOUCH_COUNT)]
        self.touch_proximity: List[deque] = [deque(maxlen=max_points) for _ in range(TOUCH_COUNT)]

        # Statistics
        self.start_time: Optional[float] = None
        self.packet_count: int = 0
        self.error_count: int = 0
        self.last_latencies: deque = deque(maxlen=100)
        self._last_packet_times: deque = deque(maxlen=100)

    def add_motor_data(self, timestamp: float, positions: List[int],
                       speeds: List[int], currents: List[int], latency: float = 0):
        """Add motor data point (6 DOF)"""
        if self.start_time is None:
            self.start_time = timestamp

        rel_time = timestamp - self.start_time
        self.times.append(rel_time)

        for i in range(min(MOTOR_COUNT, len(positions))):
            self.motor_positions[i].append(positions[i])
        for i in range(min(MOTOR_COUNT, len(speeds))):
            self.motor_speeds[i].append(speeds[i])
        for i in range(min(MOTOR_COUNT, len(currents))):
            self.motor_currents[i].append(currents[i])

        self.packet_count += 1
        self._last_packet_times.append(timestamp)
        if latency > 0:
            self.last_latencies.append(latency)

    def add_touch_data(self, normal: List[int], tangential: List[int], proximity: List[int]):
        """Add touch data point (5 fingers)"""
        for i in range(min(TOUCH_COUNT, len(normal))):
            self.touch_normal[i].append(normal[i])
        for i in range(min(TOUCH_COUNT, len(tangential))):
            self.touch_tangential[i].append(tangential[i])
        for i in range(min(TOUCH_COUNT, len(proximity))):
            self.touch_proximity[i].append(proximity[i])

    def add_error(self):
        """Increment error count"""
        self.error_count += 1

    def get_frequency(self) -> float:
        """Calculate actual update frequency in Hz"""
        if len(self._last_packet_times) < 2:
            return 0.0
        times = list(self._last_packet_times)
        duration = times[-1] - times[0]
        if duration <= 0:
            return 0.0
        return (len(times) - 1) / duration

    def get_avg_latency(self) -> float:
        """Calculate average latency in ms"""
        if not self.last_latencies:
            return 0.0
        return sum(self.last_latencies) / len(self.last_latencies) * 1000

    def clear(self):
        """Reset all buffers and statistics"""
        self.times.clear()
        # Clear motor data (6 DOF)
        for i in range(MOTOR_COUNT):
            self.motor_positions[i].clear()
            self.motor_speeds[i].clear()
            self.motor_currents[i].clear()
        # Clear touch data (5 fingers)
        for i in range(TOUCH_COUNT):
            self.touch_normal[i].clear()
            self.touch_tangential[i].clear()
            self.touch_proximity[i].clear()

        self.start_time = None
        self.packet_count = 0
        self.error_count = 0
        self.last_latencies.clear()
        self._last_packet_times.clear()

    def set_max_points(self, max_points: int):
        """Adjust buffer size"""
        self.max_points = max_points
        # Recreate deques with new maxlen
        old_times = list(self.times)
        self.times = deque(old_times[-max_points:] if len(old_times) > max_points else old_times, maxlen=max_points)

        # Motor data (6 DOF)
        for i in range(MOTOR_COUNT):
            for data_list in [self.motor_positions, self.motor_speeds, self.motor_currents]:
                old_data = list(data_list[i])
                data_list[i] = deque(old_data[-max_points:] if len(old_data) > max_points else old_data, maxlen=max_points)
        
        # Touch data (5 fingers)
        for i in range(TOUCH_COUNT):
            for data_list in [self.touch_normal, self.touch_tangential, self.touch_proximity]:
                old_data = list(data_list[i])
                data_list[i] = deque(old_data[-max_points:] if len(old_data) > max_points else old_data, maxlen=max_points)



class WaveformChart(QWidget):
    """Single waveform chart with multiple finger curves"""

    def __init__(self, title: str, y_label: str, y_range: tuple = (0, 1000), 
                 num_curves: int = MOTOR_COUNT, colors: Optional[list] = None, scale: float = 1.0):
        super().__init__()
        self.title = title
        self.y_label = y_label
        self.y_range = y_range
        self.time_window = 10.0
        self.curves = []
        self.plot_widget = None
        self.num_curves = num_curves
        self.colors = colors if colors is not None else MOTOR_COLORS[:num_curves]
        self.scale = scale  # Scale factor for data (e.g., 0.01 to convert 0-2500 to 0-25N)
        
        self._setup_ui()

    def _setup_ui(self):
        """Setup chart UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        if HAS_PYQTGRAPH and pg is not None:
            self.plot_widget = pg.PlotWidget()
            self.plot_widget.setBackground('#1a1a2e')
            self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
            self.plot_widget.setYRange(self.y_range[0], self.y_range[1])
            self.plot_widget.setTitle(self.title, color='w', size='10pt')
            self.plot_widget.setLabel('left', self.y_label, color='w')
            self.plot_widget.setLabel('bottom', 's', color='w')

            # Create curves
            for i, color in enumerate(self.colors):
                pen = pg.mkPen(color=color, width=2)
                curve = self.plot_widget.plot([], [], pen=pen)
                self.curves.append(curve)

            layout.addWidget(self.plot_widget)
        else:
            label = QLabel(f"{self.title}\n(pyqtgraph not installed)")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("color: #888; font-style: italic;")
            layout.addWidget(label)

    def update_data(self, times: list, data: Sequence[Sequence]):
        """Update all curves with new data"""
        if not HAS_PYQTGRAPH or not self.curves:
            return

        for i, curve in enumerate(self.curves):
            if i < len(data):
                # Apply scale factor to data
                if self.scale != 1.0:
                    scaled_data = [v * self.scale for v in data[i]]
                    curve.setData(times, scaled_data)
                else:
                    curve.setData(times, list(data[i]))

        # Auto-scroll X axis
        if times and self.plot_widget:
            x_max = times[-1]
            x_min = max(0, x_max - self.time_window)
            self.plot_widget.setXRange(x_min, x_max + 0.5)

    def set_time_window(self, seconds: float):
        """Adjust visible time window"""
        self.time_window = seconds

    def clear_data(self):
        """Clear all curves"""
        if not HAS_PYQTGRAPH:
            return
        for curve in self.curves:
            curve.setData([], [])

    def update_title(self, title: str):
        """Update chart title"""
        self.title = title
        if HAS_PYQTGRAPH and self.plot_widget:
            self.plot_widget.setTitle(title, color='w', size='10pt')

    def update_legend_names(self, names: List[str]):
        """Update legend with localized names"""
        pass


class RealtimeMonitorPanel(QWidget):
    """Real-time data visualization panel
    
    Uses SharedDataManager for data collection (no separate DataCollector).
    """

    def __init__(self):
        super().__init__()

        # State
        self.device = None
        self.slave_id = 1
        self.device_info = None
        self.shared_data: Optional['SharedDataManager'] = None
        self.is_running = False
        self.is_paused = False
        
        # Touch display settings
        self.touch_data_type = "normal"  # normal, tangential, proximity
        self.touch_sensor_index = 0  # 0, 1, 2

        # Data management (for UI visualization)
        self.data_manager = DataManager()

        # Timer for reading from shared data
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._on_timer_tick)

        # UI setup
        self._setup_ui()
        self.update_texts()

    def _setup_ui(self):
        """Setup the panel UI"""
        layout = QVBoxLayout(self)
        layout.setSpacing(12)

        # Top bar: Config + Control + Stats
        top_bar = QHBoxLayout()
        top_bar.setSpacing(16)

        # Config group
        self.config_group = QGroupBox()
        config_layout = QHBoxLayout(self.config_group)
        config_layout.setSpacing(8)

        self.time_window_label = QLabel()
        config_layout.addWidget(self.time_window_label)

        self.time_window_combo = QComboBox()
        for t in TIME_WINDOWS:
            self.time_window_combo.addItem(f"{t}s", t)
        self.time_window_combo.setCurrentIndex(1)  # Default 10s
        self.time_window_combo.currentIndexChanged.connect(self._on_time_window_changed)
        config_layout.addWidget(self.time_window_combo)

        config_layout.addSpacing(16)

        self.update_rate_label = QLabel()
        config_layout.addWidget(self.update_rate_label)

        self.update_rate_combo = QComboBox()
        for r in UPDATE_RATES:
            self.update_rate_combo.addItem(f"{r} Hz", r)
        self.update_rate_combo.setCurrentIndex(2)  # Default 50Hz
        self.update_rate_combo.currentIndexChanged.connect(self._on_update_rate_changed)
        config_layout.addWidget(self.update_rate_combo)

        top_bar.addWidget(self.config_group)

        # Control group
        self.control_group = QGroupBox()
        control_layout = QHBoxLayout(self.control_group)
        control_layout.setSpacing(8)

        self.start_btn = QPushButton("▶")
        self.start_btn.setFixedWidth(60)
        self.start_btn.clicked.connect(self._on_start_clicked)
        control_layout.addWidget(self.start_btn)

        self.pause_btn = QPushButton("⏸")
        self.pause_btn.setFixedWidth(60)
        self.pause_btn.setEnabled(False)
        self.pause_btn.clicked.connect(self._on_pause_clicked)
        control_layout.addWidget(self.pause_btn)

        self.clear_btn = QPushButton("🗑")
        self.clear_btn.setFixedWidth(60)
        self.clear_btn.clicked.connect(self._on_clear_clicked)
        control_layout.addWidget(self.clear_btn)

        top_bar.addWidget(self.control_group)

        # Stats group
        self.stats_group = QGroupBox()
        stats_layout = QGridLayout(self.stats_group)
        stats_layout.setSpacing(4)

        self.freq_title = QLabel()
        self.freq_value = QLabel("0.0 Hz")
        self.freq_value.setStyleSheet("font-weight: bold;")
        stats_layout.addWidget(self.freq_title, 0, 0)
        stats_layout.addWidget(self.freq_value, 0, 1)

        self.latency_title = QLabel()
        self.latency_value = QLabel("0.0 ms")
        self.latency_value.setStyleSheet("font-weight: bold;")
        stats_layout.addWidget(self.latency_title, 0, 2)
        stats_layout.addWidget(self.latency_value, 0, 3)

        self.packets_title = QLabel()
        self.packets_value = QLabel("0")
        self.packets_value.setStyleSheet("font-weight: bold;")
        stats_layout.addWidget(self.packets_title, 1, 0)
        stats_layout.addWidget(self.packets_value, 1, 1)

        self.errors_title = QLabel()
        self.errors_value = QLabel("0")
        self.errors_value.setStyleSheet("font-weight: bold; color: red;")
        stats_layout.addWidget(self.errors_title, 1, 2)
        stats_layout.addWidget(self.errors_value, 1, 3)

        top_bar.addWidget(self.stats_group)
        top_bar.addStretch()

        layout.addLayout(top_bar)

        # Main content: Charts + Hand visualization
        content_splitter = QSplitter(Qt.Horizontal)

        # Left: Charts
        charts_widget = QWidget()
        charts_layout = QGridLayout(charts_widget)
        charts_layout.setSpacing(8)

        # Motor charts use 6 curves (6 DOF)
        self.position_chart = WaveformChart("Position", "pos", (0, 1000), num_curves=MOTOR_COUNT, colors=MOTOR_COLORS)
        charts_layout.addWidget(self.position_chart, 0, 0)

        self.speed_chart = WaveformChart("Speed", "speed", (-500, 500), num_curves=MOTOR_COUNT, colors=MOTOR_COLORS)
        charts_layout.addWidget(self.speed_chart, 0, 1)

        self.current_chart = WaveformChart("Current", "mA", (0, 1000), num_curves=MOTOR_COUNT, colors=MOTOR_COLORS)
        charts_layout.addWidget(self.current_chart, 1, 0)

        # Touch chart container with controls
        touch_container = QWidget()
        touch_layout = QVBoxLayout(touch_container)
        touch_layout.setContentsMargins(0, 0, 0, 0)
        touch_layout.setSpacing(4)
        
        # Touch data type selection
        touch_control_layout = QHBoxLayout()
        touch_control_layout.setSpacing(8)
        
        self.touch_type_label = QLabel("Type:")
        self.touch_type_label.setStyleSheet("color: white; font-size: 10px;")
        touch_control_layout.addWidget(self.touch_type_label)
        
        self.touch_type_combo = QComboBox()
        self.touch_type_combo.addItem("Normal Force", "normal")
        self.touch_type_combo.addItem("Tangential Force", "tangential")
        self.touch_type_combo.addItem("Proximity", "proximity")
        self.touch_type_combo.setFixedWidth(120)
        self.touch_type_combo.currentIndexChanged.connect(self._on_touch_type_changed)
        touch_control_layout.addWidget(self.touch_type_combo)
        
        self.touch_sensor_label = QLabel("Sensor:")
        self.touch_sensor_label.setStyleSheet("color: white; font-size: 10px;")
        touch_control_layout.addWidget(self.touch_sensor_label)
        
        self.touch_sensor_combo = QComboBox()
        self.touch_sensor_combo.addItem("Sensor 1", 0)
        self.touch_sensor_combo.addItem("Sensor 2", 1)
        self.touch_sensor_combo.addItem("Sensor 3", 2)
        self.touch_sensor_combo.setFixedWidth(80)
        self.touch_sensor_combo.currentIndexChanged.connect(self._on_touch_sensor_changed)
        touch_control_layout.addWidget(self.touch_sensor_combo)
        
        touch_control_layout.addStretch()
        touch_layout.addLayout(touch_control_layout)
        
        # Touch chart uses 5 curves (5 fingers, no ThumbAux)
        # Raw value 0-2500, scale 0.01 to display 0-25N
        self.touch_chart = WaveformChart(
            "Normal Force", "N", (0, 25), 
            num_curves=TOUCH_COUNT, colors=TOUCH_COLORS, scale=0.01
        )
        touch_layout.addWidget(self.touch_chart, 1)
        
        charts_layout.addWidget(touch_container, 1, 1)

        content_splitter.addWidget(charts_widget)

        # Right: Hand visualization
        hand_container = QFrame()
        hand_container.setStyleSheet(f"""
            QFrame {{
                background-color: {COLORS.get('bg_card', '#ffffff')};
                border: 1px solid {COLORS.get('border', '#e0e0e0')};
                border-radius: 8px;
            }}
        """)
        hand_container.setMinimumWidth(220)
        hand_layout = QVBoxLayout(hand_container)
        hand_layout.setContentsMargins(8, 8, 8, 8)

        self.hand_label = QLabel("Hand Visualization")
        self.hand_label.setAlignment(Qt.AlignCenter)
        self.hand_label.setStyleSheet("font-weight: bold; margin-bottom: 4px;")
        hand_layout.addWidget(self.hand_label)

        self.hand_widget = HandVisualization()
        hand_layout.addWidget(self.hand_widget, 1)

        content_splitter.addWidget(hand_container)
        content_splitter.setSizes([800, 200])

        layout.addWidget(content_splitter, 1)


    def update_texts(self):
        """Update UI texts for i18n"""
        self.config_group.setTitle(tr("config"))
        self.time_window_label.setText(tr("time_window") + ":")
        self.update_rate_label.setText(tr("update_rate") + ":")

        self.control_group.setTitle(tr("control"))

        self.stats_group.setTitle(tr("statistics"))
        self.freq_title.setText(tr("frequency") + ":")
        self.latency_title.setText(tr("latency") + ":")
        self.packets_title.setText(tr("packets") + ":")
        self.errors_title.setText(tr("errors") + ":")

        # Update chart titles
        self.position_chart.update_title(tr("position"))
        self.speed_chart.update_title(tr("speed"))
        self.current_chart.update_title(tr("current"))
        self.touch_chart.update_title(tr("touch_force"))

        self.hand_label.setText(tr("hand_visualization"))

    def set_device(self, device, slave_id, device_info, shared_data=None):
        """Called when device connects"""
        self.device = device
        self.slave_id = slave_id
        self.device_info = device_info
        self.shared_data = shared_data
        self.start_btn.setEnabled(True)
    
    def clear_device(self):
        """Called when device disconnects"""
        self._stop_monitoring()
        self.device = None
        self.slave_id = 1
        self.device_info = None
        self.shared_data = None
        self.start_btn.setEnabled(False)

    def _on_time_window_changed(self, index):
        """Time window selection changed"""
        time_window = self.time_window_combo.currentData()
        update_rate = self.update_rate_combo.currentData()
        max_points = time_window * update_rate

        self.data_manager.set_max_points(max_points)

        for chart in [self.position_chart, self.speed_chart, self.current_chart, self.touch_chart]:
            chart.set_time_window(time_window)

    def _on_update_rate_changed(self, index):
        """Update rate selection changed"""
        rate = self.update_rate_combo.currentData()
        interval_ms = int(1000 / rate)

        if self.is_running:
            self.update_timer.setInterval(interval_ms)

        # Adjust buffer size
        time_window = self.time_window_combo.currentData()
        max_points = time_window * rate
        self.data_manager.set_max_points(max_points)

    def _on_start_clicked(self):
        """Start/Stop button clicked"""
        if self.is_running:
            self._stop_monitoring()
        else:
            self._start_monitoring()

    def _on_pause_clicked(self):
        """Pause/Resume button clicked"""
        if self.is_paused:
            self.is_paused = False
            self.pause_btn.setText("⏸")
        else:
            self.is_paused = True
            self.pause_btn.setText("▶")

    def _on_clear_clicked(self):
        """Clear button clicked"""
        self.data_manager.clear()
        self._update_charts()
        self._update_stats()

    def _on_touch_type_changed(self, index):
        """Touch data type changed"""
        self.touch_data_type = self.touch_type_combo.currentData()
        
        # Update chart title and Y-axis based on type
        if self.touch_data_type == "normal":
            self.touch_chart.update_title("Normal Force")
            if HAS_PYQTGRAPH and self.touch_chart.plot_widget:
                self.touch_chart.plot_widget.setYRange(0, 25)
                self.touch_chart.plot_widget.setLabel('left', 'N', color='w')
                self.touch_chart.plot_widget.enableAutoRange(axis='y', enable=False)
            self.touch_chart.scale = 0.01
        elif self.touch_data_type == "tangential":
            self.touch_chart.update_title("Tangential Force")
            if HAS_PYQTGRAPH and self.touch_chart.plot_widget:
                self.touch_chart.plot_widget.setYRange(0, 25)
                self.touch_chart.plot_widget.setLabel('left', 'N', color='w')
                self.touch_chart.plot_widget.enableAutoRange(axis='y', enable=False)
            self.touch_chart.scale = 0.01
        elif self.touch_data_type == "proximity":
            self.touch_chart.update_title("Proximity")
            if HAS_PYQTGRAPH and self.touch_chart.plot_widget:
                # Auto-scale Y axis for proximity (values can be millions)
                self.touch_chart.plot_widget.enableAutoRange(axis='y', enable=True)
                self.touch_chart.plot_widget.setLabel('left', 'raw', color='w')
            self.touch_chart.scale = 1.0
    
    def _on_touch_sensor_changed(self, index):
        """Touch sensor index changed"""
        self.touch_sensor_index = self.touch_sensor_combo.currentData()

    def _start_monitoring(self):
        """Start monitoring (uses SharedDataManager)"""
        if not self.shared_data:
            return

        self.is_running = True
        self.is_paused = False
        self.start_btn.setText("⏹")
        self.pause_btn.setEnabled(True)

        # Start UI update timer (20Hz is enough for UI)
        self.update_timer.start(50)

    def _stop_monitoring(self):
        """Stop monitoring"""
        self.is_running = False
        self.is_paused = False
        self.update_timer.stop()

        self.start_btn.setText("▶")
        self.pause_btn.setEnabled(False)
        self.pause_btn.setText("⏸")

    def _on_timer_tick(self):
        """Read data from shared data manager and update UI"""
        if self.is_paused or not self.shared_data:
            return

        try:
            self._read_from_shared_data()
            self._update_charts()
            self._update_stats()
        except Exception as e:
            self.data_manager.add_error()
            self._update_stats()
            print(f"Read shared data error: {e}")
    
    def _read_from_shared_data(self):
        """Read data from SharedDataManager into DataManager"""
        if not self.shared_data:
            return
        timestamp = time.time()
        
        # Read motor data from shared data manager
        motor_data_list = self.shared_data.get_all_motor()
        for status in motor_data_list:
            positions = list(status.positions) if status.positions else [0] * MOTOR_COUNT
            speeds = list(status.speeds) if status.speeds else [0] * MOTOR_COUNT
            currents = list(status.currents) if status.currents else [0] * MOTOR_COUNT
            self.data_manager.add_motor_data(timestamp, positions, speeds, currents, 0)
        
        # Read touch data if available
        if self.shared_data.has_touch():
            try:
                touch_data_list = self.shared_data.get_all_touch()
                sensor_idx = self.touch_sensor_index
                
                for touch_data in touch_data_list:
                    if touch_data and len(touch_data) >= TOUCH_COUNT:
                        # Extract data based on selected sensor index
                        normal = []
                        tangential = []
                        proximity = []
                        
                        for t in touch_data[:TOUCH_COUNT]:
                            # Get force values based on sensor index (1, 2, or 3)
                            if sensor_idx == 0:
                                normal.append(getattr(t, 'normal_force1', 0))
                                tangential.append(getattr(t, 'tangential_force1', 0))
                                proximity.append(getattr(t, 'self_proximity1', 0))
                            elif sensor_idx == 1:
                                normal.append(getattr(t, 'normal_force2', 0))
                                tangential.append(getattr(t, 'tangential_force2', 0))
                                proximity.append(getattr(t, 'self_proximity2', 0))
                            else:  # sensor_idx == 2
                                normal.append(getattr(t, 'normal_force3', 0))
                                tangential.append(getattr(t, 'tangential_force3', 0))
                                proximity.append(getattr(t, 'mutual_proximity', 0))
                        
                        self.data_manager.add_touch_data(normal, tangential, proximity)
            except Exception as e:
                print(f"Touch data read error: {e}")

    def _update_charts(self):
        """Update all chart displays"""
        times = list(self.data_manager.times)

        self.position_chart.update_data(times, self.data_manager.motor_positions)
        self.speed_chart.update_data(times, self.data_manager.motor_speeds)
        self.current_chart.update_data(times, self.data_manager.motor_currents)
        
        # Update touch chart based on selected data type
        if self.touch_data_type == "normal":
            self.touch_chart.update_data(times, self.data_manager.touch_normal)
        elif self.touch_data_type == "tangential":
            self.touch_chart.update_data(times, self.data_manager.touch_tangential)
        elif self.touch_data_type == "proximity":
            self.touch_chart.update_data(times, self.data_manager.touch_proximity)

        # Update hand visualization with latest data
        if self.data_manager.motor_positions[0]:
            positions = [list(p)[-1] if p else 0 for p in self.data_manager.motor_positions]
            self.hand_widget.set_finger_positions(positions)

        if self.data_manager.touch_normal[0]:
            intensities = [list(t)[-1] if t else 0 for t in self.data_manager.touch_normal]
            self.hand_widget.set_touch_intensities(intensities)

    def _update_stats(self):
        """Update statistics display"""
        freq = self.data_manager.get_frequency()
        latency = self.data_manager.get_avg_latency()

        self.freq_value.setText(f"{freq:.1f} Hz")
        self.latency_value.setText(f"{latency:.1f} ms")
        self.packets_value.setText(str(self.data_manager.packet_count))
        self.errors_value.setText(str(self.data_manager.error_count))
