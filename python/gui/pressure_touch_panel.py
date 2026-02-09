"""Pressure Touch Sensor Panel - For Revo2 Touch Pressure (Modulus) devices

This panel displays pressure touch sensor data with:
- 6 sensors: 5 fingers + 1 palm
- Summary view: Total force per sensor (mN)
- Detailed view: Individual sampling points per sensor

Data structure (from SDK):
- Summary: 6 values (one per sensor) - total force in mN
- Detailed: 6 PressureDetailedItem objects
  - Fingers: 9 sensor points each
  - Palm: 46 sensor points
"""

import asyncio
from typing import Optional, TYPE_CHECKING
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QCheckBox, QGridLayout, QTabWidget,
    QFrame, QProgressBar, QComboBox
)
from PySide6.QtCore import Qt, QTimer

if TYPE_CHECKING:
    from .shared_data import SharedDataManager

from .i18n import tr
from .styles import COLORS


def run_async(coro):
    """Run async coroutine in a new event loop (for Qt callbacks)"""
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


try:
    import pyqtgraph as pg
    HAS_PYQTGRAPH = True
except ImportError:
    pg = None  # type: ignore
    HAS_PYQTGRAPH = False


# Pressure sensor configuration
# 6 sensors: 5 fingers + palm
PRESSURE_SENSOR_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"]
PRESSURE_SENSOR_COLORS = [
    (255, 100, 100),   # Thumb - Red
    (100, 255, 100),   # Index - Green
    (100, 100, 255),   # Middle - Blue
    (255, 255, 100),   # Ring - Yellow
    (255, 100, 255),   # Pinky - Magenta
    (100, 255, 255),   # Palm - Cyan
]

# Sampling points per sensor (based on SDK PressureDetailedItem)
# Fingers: 9 sensor points each
# Palm: 46 sensor points
PRESSURE_SENSOR_POINTS = {
    "Thumb": 9,
    "Index": 9,
    "Middle": 9,
    "Ring": 9,
    "Pinky": 9,
    "Palm": 46,
}


class PressureSummaryChart(QWidget):
    """Chart showing pressure summary for all 6 sensors"""
    
    def __init__(self, title: str = "Pressure Summary", y_range: tuple = (0, 5000)):
        super().__init__()
        self.title = title
        self.y_range = y_range
        self.curves = []
        self.data = [[] for _ in range(6)]  # 6 sensors
        self.max_points = 200
        
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        if HAS_PYQTGRAPH and pg is not None:
            self.plot = pg.PlotWidget()
            self.plot.setBackground('#1a1a2e')
            self.plot.showGrid(x=True, y=True, alpha=0.3)
            self.plot.setYRange(self.y_range[0], self.y_range[1])
            self.plot.setXRange(0, self.max_points)
            self.plot.setTitle(self.title, color='w', size='10pt')
            self.plot.setLabel('bottom', 'samples', color='w')
            self.plot.setLabel('left', 'mN', color='w')
            
            # Create curves for each sensor
            for i, (name, color) in enumerate(zip(PRESSURE_SENSOR_NAMES, PRESSURE_SENSOR_COLORS)):
                pen = pg.mkPen(color=color, width=2)
                curve = self.plot.plot([], [], pen=pen, name=name)
                self.curves.append(curve)
            
            self.plot.addLegend()
            layout.addWidget(self.plot)
        else:
            layout.addWidget(QLabel("pyqtgraph not installed"))
    
    def add_data(self, values: list):
        """Add data for all 6 sensors"""
        for i, val in enumerate(values[:6]):
            self.data[i].append(val)
            if len(self.data[i]) > self.max_points:
                self.data[i].pop(0)
        self._update_curves()
    
    def _update_curves(self):
        if not HAS_PYQTGRAPH:
            return
        for i, curve in enumerate(self.curves):
            if self.data[i]:
                curve.setData(list(range(len(self.data[i]))), self.data[i])
    
    def clear(self):
        self.data = [[] for _ in range(6)]
        self._update_curves()
    
    def set_sensor_visible(self, sensor_idx: int, visible: bool):
        """Show/hide a sensor curve"""
        if HAS_PYQTGRAPH and sensor_idx < len(self.curves):
            self.curves[sensor_idx].setVisible(visible)


class PressureDetailChart(QWidget):
    """Chart showing detailed pressure points for a single sensor with values display"""
    
    def __init__(self, sensor_name: str, point_count: int, color: tuple):
        super().__init__()
        self.sensor_name = sensor_name
        self.point_count = point_count
        self.color = color
        self.max_points = 200
        self.data = [[] for _ in range(point_count)]
        self.current_values = [0] * point_count
        self.total_force = 0
        self.point_colors = []  # Will be populated in _setup_ui
        
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(12)
        
        # Left: Chart
        chart_widget = QWidget()
        chart_layout = QVBoxLayout(chart_widget)
        chart_layout.setContentsMargins(0, 0, 0, 0)
        
        if HAS_PYQTGRAPH and pg is not None:
            self.plot = pg.PlotWidget()
            self.plot.setBackground('#1a1a2e')
            self.plot.showGrid(x=True, y=True, alpha=0.3)
            self.plot.enableAutoRange(axis='y', enable=True)
            self.plot.setXRange(0, self.max_points)
            self.plot.setTitle(f"{self.sensor_name} ({self.point_count} points)", color='w', size='10pt')
            self.plot.setLabel('bottom', 'samples', color='w')
            self.plot.setLabel('left', 'mN', color='w')
            
            # Create curves for each sampling point
            self.curves = []
            base_r, base_g, base_b = self.color
            
            # Generate distinct colors for each point using HSV color wheel
            import colorsys
            for i in range(self.point_count):
                # Use HSV to generate distinct colors
                # Hue varies across the spectrum, saturation and value stay high
                hue = (i / max(1, self.point_count)) * 0.8  # 0 to 0.8 (red to purple)
                sat = 0.8
                val = 0.9
                r, g, b = colorsys.hsv_to_rgb(hue, sat, val)
                r, g, b = int(r * 255), int(g * 255), int(b * 255)
                pen = pg.mkPen(color=(r, g, b), width=1)
                curve = self.plot.plot([], [], pen=pen)
                self.curves.append(curve)
            
            chart_layout.addWidget(self.plot)
        else:
            chart_layout.addWidget(QLabel("pyqtgraph not installed"))
        
        layout.addWidget(chart_widget, 2)  # Chart takes 2/3 width
        
        # Right: Values panel
        values_widget = QFrame()
        values_widget.setStyleSheet(f"""
            QFrame {{
                background-color: {COLORS['bg_card']};
                border-radius: 8px;
                padding: 8px;
            }}
        """)
        values_layout = QVBoxLayout(values_widget)
        values_layout.setSpacing(6)
        
        # Title with sensor name
        r, g, b = self.color
        title = QLabel(f"🔴 {self.sensor_name}")
        title.setStyleSheet(f"font-size: 14px; font-weight: bold; color: rgb({r}, {g}, {b});")
        values_layout.addWidget(title)
        
        # Total force
        total_layout = QHBoxLayout()
        total_label = QLabel("Total Force:")
        total_label.setStyleSheet("font-weight: bold;")
        total_layout.addWidget(total_label)
        self.total_label = QLabel("0 mN")
        self.total_label.setStyleSheet(f"font-weight: bold; color: rgb({r}, {g}, {b}); font-size: 16px;")
        total_layout.addWidget(self.total_label)
        total_layout.addStretch()
        values_layout.addLayout(total_layout)
        
        # Status
        status_layout = QHBoxLayout()
        status_label = QLabel("Status:")
        status_layout.addWidget(status_label)
        self.status_label = QLabel("--")
        self.status_label.setStyleSheet(f"font-weight: bold; color: {COLORS['success']};")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        values_layout.addLayout(status_layout)
        
        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("background-color: #444;")
        sep.setFixedHeight(1)
        values_layout.addWidget(sep)
        
        # Point values (scrollable for Palm with 46 points)
        from PySide6.QtWidgets import QScrollArea
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        
        points_widget = QWidget()
        points_layout = QGridLayout(points_widget)
        points_layout.setSpacing(4)
        points_layout.setContentsMargins(0, 0, 0, 0)
        
        self.point_labels = []
        self.point_colors = []  # Store colors for each point
        cols = 3 if self.point_count > 12 else 2
        
        # Generate colors matching the curves
        import colorsys
        for i in range(self.point_count):
            row = i // cols
            col = i % cols
            
            # Generate same color as curve
            hue = (i / max(1, self.point_count)) * 0.8
            sat = 0.8
            val = 0.9
            r, g, b = colorsys.hsv_to_rgb(hue, sat, val)
            r, g, b = int(r * 255), int(g * 255), int(b * 255)
            self.point_colors.append((r, g, b))
            
            point_frame = QFrame()
            point_frame.setStyleSheet(f"background-color: #2a2a3e; border-radius: 4px; padding: 2px; border-left: 3px solid rgb({r}, {g}, {b});")
            point_layout = QHBoxLayout(point_frame)
            point_layout.setContentsMargins(4, 2, 4, 2)
            point_layout.setSpacing(4)
            
            idx_label = QLabel(f"P{i+1}:")
            idx_label.setFixedWidth(28)
            idx_label.setStyleSheet(f"font-size: 10px; color: rgb({r}, {g}, {b});")
            point_layout.addWidget(idx_label)
            
            val_label = QLabel("0")
            val_label.setStyleSheet("font-size: 10px; font-weight: bold;")
            val_label.setAlignment(Qt.AlignRight)
            point_layout.addWidget(val_label)
            
            self.point_labels.append(val_label)
            points_layout.addWidget(point_frame, row, col)
        
        scroll.setWidget(points_widget)
        values_layout.addWidget(scroll, 1)
        
        layout.addWidget(values_widget, 1)  # Values panel takes 1/3 width
    
    def add_data(self, values: list):
        """Add data for all sampling points"""
        self.current_values = list(values[:self.point_count])
        self.total_force = sum(self.current_values)
        
        for i, val in enumerate(values[:self.point_count]):
            self.data[i].append(val)
            if len(self.data[i]) > self.max_points:
                self.data[i].pop(0)
        
        self._update_curves()
        self._update_values()
    
    def _update_curves(self):
        if not HAS_PYQTGRAPH:
            return
        for i, curve in enumerate(self.curves):
            if self.data[i]:
                curve.setData(list(range(len(self.data[i]))), self.data[i])
    
    def _update_values(self):
        """Update value labels"""
        # Total force
        self.total_label.setText(f"{self.total_force} mN")
        
        # Status based on total force
        if self.total_force > 100:
            self.status_label.setText("Contact")
            self.status_label.setStyleSheet(f"font-weight: bold; color: {COLORS['accent']};")
        else:
            self.status_label.setText("Idle")
            self.status_label.setStyleSheet(f"font-weight: bold; color: {COLORS['text_muted']};")
        
        # Individual point values
        for i, val in enumerate(self.current_values):
            if i < len(self.point_labels):
                self.point_labels[i].setText(str(val))
                # Highlight active points with their own color
                if val > 50:
                    r, g, b = self.point_colors[i] if i < len(self.point_colors) else (255, 165, 0)
                    self.point_labels[i].setStyleSheet(f"font-size: 10px; font-weight: bold; color: rgb({r}, {g}, {b});")
                else:
                    self.point_labels[i].setStyleSheet("font-size: 10px; font-weight: bold; color: #888;")
    
    def clear(self):
        self.data = [[] for _ in range(self.point_count)]
        self.current_values = [0] * self.point_count
        self.total_force = 0
        self._update_curves()
        self._update_values()


class PressureTouchPanel(QWidget):
    """Pressure Touch Sensor Panel for Revo2 Touch Pressure (Modulus) devices
    
    Uses SharedDataManager for device state.
    """
    
    def __init__(self):
        super().__init__()
        self.shared_data: Optional['SharedDataManager'] = None
        
        self._setup_ui()
        self.update_texts()
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_data_from_shared)
        self.update_timer.setInterval(50)
    
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
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(8, 8, 8, 8)
        
        # Control buttons
        control_layout = QHBoxLayout()
        control_layout.setSpacing(8)
        
        self.enable_btn = QPushButton("Enable")
        self.enable_btn.clicked.connect(self._enable_touch)
        control_layout.addWidget(self.enable_btn)
        
        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.clicked.connect(self._calibrate)
        control_layout.addWidget(self.calibrate_btn)
        
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self._clear_charts)
        control_layout.addWidget(self.clear_btn)
        
        # Data type selector
        control_layout.addWidget(QLabel("Data Type:"))
        self.data_type_combo = QComboBox()
        self.data_type_combo.addItems(["Summary", "Detailed"])
        self.data_type_combo.currentIndexChanged.connect(self._on_data_type_changed)
        control_layout.addWidget(self.data_type_combo)
        
        # Sensor checkboxes
        control_layout.addWidget(QLabel("|"))
        self.sensor_checks = []
        for i, name in enumerate(PRESSURE_SENSOR_NAMES):
            check = QCheckBox(name)
            check.setChecked(True)
            check.stateChanged.connect(lambda state, idx=i: self._on_sensor_check_changed(idx, state))
            self.sensor_checks.append(check)
            control_layout.addWidget(check)
        
        control_layout.addStretch()
        layout.addLayout(control_layout)
        
        # Tab widget
        self.tabs = QTabWidget()
        
        # Overview tab
        overview_widget = QWidget()
        overview_layout = QGridLayout(overview_widget)
        overview_layout.setSpacing(8)
        
        # Summary chart (left)
        self.summary_chart = PressureSummaryChart("Pressure Summary (mN)", (0, 5000))
        overview_layout.addWidget(self.summary_chart, 0, 0, 2, 1)
        
        # Sensor status cards (right)
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        status_layout.setSpacing(4)
        
        title = QLabel("📊 Sensor Status")
        title.setStyleSheet("font-weight: bold; font-size: 12px;")
        status_layout.addWidget(title)
        
        self.sensor_cards = []
        self.sensor_bars = []
        self.sensor_labels = []
        
        for i, (name, color) in enumerate(zip(PRESSURE_SENSOR_NAMES, PRESSURE_SENSOR_COLORS)):
            card = QFrame()
            card.setStyleSheet(f"""
                QFrame {{
                    background-color: {COLORS['bg_card']};
                    border-left: 4px solid rgb{color};
                    border-radius: 4px;
                    padding: 4px;
                }}
            """)
            card_layout = QHBoxLayout(card)
            card_layout.setContentsMargins(8, 4, 8, 4)
            card_layout.setSpacing(8)
            
            # Sensor name
            name_label = QLabel(name)
            name_label.setFixedWidth(50)
            name_label.setStyleSheet(f"color: rgb{color}; font-weight: bold;")
            card_layout.addWidget(name_label)
            
            # Progress bar
            bar = QProgressBar()
            bar.setRange(0, 5000)
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setFixedHeight(16)
            r, g, b = color
            bar.setStyleSheet(f"""
                QProgressBar {{
                    border: 1px solid #444;
                    border-radius: 4px;
                    background-color: #2a2a3e;
                }}
                QProgressBar::chunk {{
                    background-color: rgb({r}, {g}, {b});
                    border-radius: 3px;
                }}
            """)
            card_layout.addWidget(bar, 1)
            self.sensor_bars.append(bar)
            
            # Value label
            val_label = QLabel("0 mN")
            val_label.setFixedWidth(70)
            val_label.setAlignment(Qt.AlignRight)
            val_label.setStyleSheet("font-family: 'Menlo', 'Monaco', 'Courier New', monospace;")
            card_layout.addWidget(val_label)
            self.sensor_labels.append(val_label)
            
            status_layout.addWidget(card)
            self.sensor_cards.append(card)
        
        status_layout.addStretch()
        overview_layout.addWidget(status_widget, 0, 1, 2, 1)
        
        self.tabs.addTab(overview_widget, "📊 Overview")
        
        # Detail tabs for each sensor
        self.detail_charts = []
        for i, (name, color) in enumerate(zip(PRESSURE_SENSOR_NAMES, PRESSURE_SENSOR_COLORS)):
            point_count = PRESSURE_SENSOR_POINTS[name]
            chart = PressureDetailChart(name, point_count, color)
            self.detail_charts.append(chart)
            self.tabs.addTab(chart, name)
        
        layout.addWidget(self.tabs, 1)
    
    def update_texts(self):
        """Update texts for i18n"""
        self.enable_btn.setText(tr("btn_enable_touch"))
        self.calibrate_btn.setText(tr("btn_calibrate"))
        self.clear_btn.setText(tr("btn_clear"))
    
    def set_device(self, device, slave_id, device_info, shared_data=None):
        """Set device for pressure touch panel"""
        if self.shared_data:
            try:
                self.shared_data.pressure_touch_updated.disconnect(self._on_pressure_touch_updated)
            except:
                pass
        
        self.shared_data = shared_data
        
        if not device:
            self.update_timer.stop()
            return
        
        # Check if device has pressure touch using helper function
        from common_imports import uses_pressure_touch_api
        if device_info and hasattr(device_info, 'hardware_type'):
            hw_type = device_info.hardware_type
            if not uses_pressure_touch_api(hw_type):
                self.setEnabled(False)
                return
        
        self.setEnabled(True)
        
        # Connect to pressure_touch_updated signal
        if shared_data and hasattr(shared_data, 'pressure_touch_updated'):
            shared_data.pressure_touch_updated.connect(self._on_pressure_touch_updated)
        
        self.update_timer.start()
    
    def clear_device(self):
        """Clear device when disconnected"""
        self.update_timer.stop()
        if self.shared_data:
            try:
                self.shared_data.pressure_touch_updated.disconnect(self._on_pressure_touch_updated)
            except:
                pass
        self.shared_data = None
    
    def _on_pressure_touch_updated(self, summary_data, detailed_data):
        """Handle pressure touch data from SharedDataManager signal"""
        self._process_pressure_data(summary_data, detailed_data)
    
    def _update_data_from_shared(self):
        """Timer callback - data is received via signal"""
        pass
    
    def _process_pressure_data(self, summary_data, detailed_data):
        """Process pressure touch data
        
        Args:
            summary_data: List of 6 int values (one per sensor: 5 fingers + palm)
            detailed_data: List of 6 PressureDetailedItem (one per sensor)
        """
        if summary_data:
            # Filter out None values and convert to list
            valid_summary = [v if v is not None else 0 for v in summary_data[:6]]
            
            # Update summary chart
            self.summary_chart.add_data(valid_summary)
            
            # Update sensor status cards
            for i, val in enumerate(valid_summary):
                if i < len(self.sensor_bars):
                    self.sensor_bars[i].setValue(min(val, 5000))
                    self.sensor_labels[i].setText(f"{val} mN")
        
        if detailed_data:
            # detailed_data is a list of PressureDetailedItem (one per sensor)
            for i, item in enumerate(detailed_data[:6]):
                if item is None:
                    continue
                
                # Get sensor data from PressureDetailedItem
                sensor_values = item.sensors if hasattr(item, 'sensors') else []
                
                if i < len(self.detail_charts) and sensor_values:
                    self.detail_charts[i].add_data(sensor_values)
    
    def _clear_charts(self):
        """Clear all charts"""
        self.summary_chart.clear()
        for chart in self.detail_charts:
            chart.clear()
    
    def _on_sensor_check_changed(self, sensor_idx: int, state):
        """Sensor checkbox changed"""
        is_checked = state == Qt.Checked.value if hasattr(Qt.Checked, 'value') else state == 2
        
        # Show/hide sensor card
        if sensor_idx < len(self.sensor_cards):
            self.sensor_cards[sensor_idx].setVisible(is_checked)
        
        # Show/hide curve in summary chart
        self.summary_chart.set_sensor_visible(sensor_idx, is_checked)
    
    def _on_data_type_changed(self, index):
        """Data type combo changed"""
        # TODO: Update data collection mode via SDK
        pass
    
    def _get_selected_sensors_bits(self):
        """Get selected sensors bitmask (6 bits for 5 fingers + palm)"""
        bits = 0
        for i, check in enumerate(self.sensor_checks):
            if check.isChecked():
                bits |= (1 << i)
        return bits
    
    def _enable_touch(self):
        """Enable pressure touch sensors"""
        if self.device:
            bits = self._get_selected_sensors_bits()
            run_async(self._async_enable_touch(bits))
    
    async def _async_enable_touch(self, bits):
        if not self.device:
            return
        try:
            await self.device.touch_sensor_setup(self.slave_id, bits)
        except Exception as e:
            print(f"Enable touch failed: {e}")
    
    def _calibrate(self):
        """Calibrate pressure touch sensors"""
        if self.device:
            bits = self._get_selected_sensors_bits()
            run_async(self._async_calibrate(bits))
    
    async def _async_calibrate(self, bits):
        if not self.device:
            return
        try:
            await self.device.touch_sensor_calibrate(self.slave_id, bits)
        except Exception as e:
            print(f"Calibrate failed: {e}")
