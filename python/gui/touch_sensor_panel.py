"""Touch Sensor Panel with Charts - Supports Revo1 multi-sensor per finger"""

import asyncio
from typing import Optional, TYPE_CHECKING
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QCheckBox, QGridLayout, QTabWidget,
    QScrollArea, QFrame, QSplitter, QMessageBox, QComboBox
)
from PySide6.QtCore import Qt, QTimer

if TYPE_CHECKING:
    from .shared_data import SharedDataManager

from .i18n import tr
from .styles import COLORS
from .constants import TOUCH_COLORS, TOUCH_NAMES_EN, TOUCH_COUNT, TOUCH_SENSOR_CONFIG


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

# Use constants from constants.py
FINGER_COLORS = TOUCH_COLORS
FINGER_NAMES = TOUCH_NAMES_EN

# Revo1 Touch sensor counts per finger (force groups) from TOUCH_SENSOR_CONFIG
REVO1_SENSOR_COUNTS = [TOUCH_SENSOR_CONFIG[name][0] for name in FINGER_NAMES]


class MultiSensorChart(QWidget):
    """Chart showing multiple sensors per finger"""
    
    def __init__(self, title: str, y_range: tuple = (0, 1000), max_sensors: int = 3):
        super().__init__()
        self.title = title
        self.y_range = y_range
        self.max_sensors = max_sensors
        self.curves = []  # [finger][sensor] -> curve
        self.data = [[[] for _ in range(max_sensors)] for _ in range(5)]
        self.max_points = 200
        self.scale = 1.0  # Scale factor for data
        self.auto_scale_y = False  # Auto scale Y axis
        
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        if HAS_PYQTGRAPH and pg is not None:
            self.plot = pg.PlotWidget()
            self.plot.setBackground('#1a1a2e')
            self.plot.showGrid(x=True, y=True, alpha=0.3)
            self.plot.setYRange(self.y_range[0], self.y_range[1])
            self.plot.setXRange(0, self.max_points)  # Fixed X axis
            self.plot.setTitle(self.title, color='w', size='10pt')
            self.plot.setLabel('bottom', 'samples', color='w')
            
            # Create curves for each finger and sensor
            line_styles = [Qt.SolidLine, Qt.DashLine, Qt.DotLine]
            for i, color in enumerate(FINGER_COLORS):
                finger_curves = []
                for s in range(self.max_sensors):
                    style = line_styles[s % len(line_styles)]
                    pen = pg.mkPen(color=color, width=2, style=style)
                    name = f"{FINGER_NAMES[i]}_{s+1}" if self.max_sensors > 1 else FINGER_NAMES[i]
                    curve = self.plot.plot([], [], pen=pen, name=name)
                    finger_curves.append(curve)
                self.curves.append(finger_curves)
            
            layout.addWidget(self.plot)
        else:
            layout.addWidget(QLabel("pyqtgraph not installed"))
    
    def add_data(self, finger_idx: int, sensor_values: list):
        """Add data for one finger's sensors"""
        for s, val in enumerate(sensor_values[:self.max_sensors]):
            # Apply scale factor
            scaled_val = val * self.scale
            self.data[finger_idx][s].append(scaled_val)
            if len(self.data[finger_idx][s]) > self.max_points:
                self.data[finger_idx][s].pop(0)
        self._update_curves()
    
    def _update_curves(self):
        if not HAS_PYQTGRAPH:
            return
        
        # Enable auto scale Y if needed
        if self.auto_scale_y and hasattr(self, 'plot'):
            self.plot.enableAutoRange(axis='y', enable=True)
        
        for i, finger_curves in enumerate(self.curves):
            for s, curve in enumerate(finger_curves):
                if self.data[i][s]:
                    curve.setData(list(range(len(self.data[i][s]))), self.data[i][s])
    
    def clear(self):
        self.data = [[[] for _ in range(self.max_sensors)] for _ in range(5)]
        self._update_curves()


class FingerDetailWidget(QWidget):
    """Detailed view for a single finger's sensors with charts"""
    
    def __init__(self, finger_idx: int):
        super().__init__()
        self.finger_idx = finger_idx
        self.finger_name = FINGER_NAMES[finger_idx]
        self.sensor_count = REVO1_SENSOR_COUNTS[finger_idx]
        self.max_points = 200
        
        # Data storage for charts
        self.normal_data = [[], [], []]  # 3 sensors
        self.tangential_data = [[], [], []]
        self.proximity_data = [[], [], []]
        
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # Title
        title = QLabel(f"🖐️ {self.finger_name} ({self.sensor_count} force groups)")
        title.setStyleSheet(f"font-size: 14px; font-weight: bold; color: rgb{FINGER_COLORS[self.finger_idx]};")
        layout.addWidget(title)
        
        if HAS_PYQTGRAPH and pg is not None:
            # Charts for this finger
            charts_layout = QGridLayout()
            charts_layout.setSpacing(8)
            
            # Normal force chart (3 sensors) - auto scale Y
            self.normal_plot = pg.PlotWidget()
            self.normal_plot.setBackground('#1a1a2e')
            self.normal_plot.showGrid(x=True, y=True, alpha=0.3)
            self.normal_plot.enableAutoRange(axis='y', enable=True)
            self.normal_plot.setTitle("Normal Force (N)", color='w', size='10pt')
            self.normal_plot.setLabel('left', 'N', color='w')
            
            self.normal_curves = []
            colors = [(255, 100, 100), (100, 255, 100), (100, 100, 255)]
            for i in range(3):
                pen = pg.mkPen(color=colors[i], width=2)
                curve = self.normal_plot.plot([], [], pen=pen, name=f"Sensor {i+1}")
                self.normal_curves.append(curve)
            self.normal_plot.addLegend()
            charts_layout.addWidget(self.normal_plot, 0, 0)
            
            # Tangential force chart (3 sensors) - auto scale Y
            self.tangential_plot = pg.PlotWidget()
            self.tangential_plot.setBackground('#1a1a2e')
            self.tangential_plot.showGrid(x=True, y=True, alpha=0.3)
            self.tangential_plot.enableAutoRange(axis='y', enable=True)
            self.tangential_plot.setTitle("Tangential Force (N)", color='w', size='10pt')
            self.tangential_plot.setLabel('left', 'N', color='w')
            
            self.tangential_curves = []
            for i in range(3):
                pen = pg.mkPen(color=colors[i], width=2)
                curve = self.tangential_plot.plot([], [], pen=pen, name=f"Sensor {i+1}")
                self.tangential_curves.append(curve)
            self.tangential_plot.addLegend()
            charts_layout.addWidget(self.tangential_plot, 0, 1)
            
            # Proximity chart (self1, self2, mutual)
            self.proximity_plot = pg.PlotWidget()
            self.proximity_plot.setBackground('#1a1a2e')
            self.proximity_plot.showGrid(x=True, y=True, alpha=0.3)
            self.proximity_plot.enableAutoRange(axis='y', enable=True)
            self.proximity_plot.setTitle("Proximity", color='w', size='10pt')
            self.proximity_plot.setLabel('left', 'raw', color='w')
            
            self.proximity_curves = []
            prox_names = ['Self 1', 'Self 2', 'Mutual']
            for i in range(3):
                pen = pg.mkPen(color=colors[i], width=2)
                curve = self.proximity_plot.plot([], [], pen=pen, name=prox_names[i])
                self.proximity_curves.append(curve)
            self.proximity_plot.addLegend()
            charts_layout.addWidget(self.proximity_plot, 1, 0)
            
            # Status and current values
            status_widget = QFrame()
            status_widget.setStyleSheet(f"background-color: {COLORS['bg_card']}; border-radius: 8px;")
            status_layout = QVBoxLayout(status_widget)
            status_layout.setSpacing(4)
            
            status_title = QLabel("Current Values")
            status_title.setStyleSheet("font-weight: bold;")
            status_layout.addWidget(status_title)
            
            # Sensor colors matching chart curves
            sensor_colors = ['rgb(255, 100, 100)', 'rgb(100, 255, 100)', 'rgb(100, 100, 255)']
            
            # Value labels - dynamically based on sensor_count
            self.value_labels = {}
            value_fields = []
            
            # Add Normal force labels based on sensor_count
            for i in range(self.sensor_count):
                value_fields.append((f'Normal {i+1}', sensor_colors[i]))
            
            # Add Tangential force labels based on sensor_count
            for i in range(self.sensor_count):
                value_fields.append((f'Tang {i+1}', sensor_colors[i]))
            
            # Proximity labels (always show available ones)
            value_fields.append(('Self Prox 1', sensor_colors[0]))
            if self.sensor_count >= 2:
                value_fields.append(('Self Prox 2', sensor_colors[1]))
            if self.sensor_count >= 2:  # Mutual proximity available for most fingers
                value_fields.append(('Mutual Prox', sensor_colors[2] if self.sensor_count >= 3 else sensor_colors[1]))
            
            value_fields.append(('Status', None))
            
            for field, color in value_fields:
                row = QHBoxLayout()
                name_label = QLabel(f"{field}:")
                name_label.setFixedWidth(80)
                if color:
                    name_label.setStyleSheet(f"color: {color};")
                else:
                    name_label.setStyleSheet(f"color: {COLORS['text_muted']};")
                row.addWidget(name_label)
                
                val_label = QLabel("--")
                if color:
                    val_label.setStyleSheet(f"font-weight: bold; color: {color};")
                row.addWidget(val_label)
                row.addStretch()
                
                self.value_labels[field] = val_label
                status_layout.addLayout(row)
            
            status_layout.addStretch()
            charts_layout.addWidget(status_widget, 1, 1)
            
            layout.addLayout(charts_layout)
        else:
            layout.addWidget(QLabel("pyqtgraph not installed"))
        
        layout.addStretch()
    
    def update_data(self, items: list):
        """Update with TouchFingerItem list"""
        if not items:
            return
        
        # Use first item for data (Revo1AdvancedTouch returns one item per finger)
        item = items[0]
        
        # Get values
        n1 = getattr(item, 'normal_force1', 0)
        n2 = getattr(item, 'normal_force2', 0)
        n3 = getattr(item, 'normal_force3', 0)
        t1 = getattr(item, 'tangential_force1', 0)
        t2 = getattr(item, 'tangential_force2', 0)
        t3 = getattr(item, 'tangential_force3', 0)
        p1 = getattr(item, 'self_proximity1', 0)
        p2 = getattr(item, 'self_proximity2', 0)
        pm = getattr(item, 'mutual_proximity', 0)
        status = getattr(item, 'status', 0)
        
        # Update data buffers (scale force to N: raw/100)
        self.normal_data[0].append(n1 * 0.01)
        self.normal_data[1].append(n2 * 0.01)
        self.normal_data[2].append(n3 * 0.01)
        
        self.tangential_data[0].append(t1 * 0.01)
        self.tangential_data[1].append(t2 * 0.01)
        self.tangential_data[2].append(t3 * 0.01)
        
        self.proximity_data[0].append(p1)
        self.proximity_data[1].append(p2)
        self.proximity_data[2].append(pm)
        
        # Trim to max points
        for data_list in [self.normal_data, self.tangential_data, self.proximity_data]:
            for i in range(3):
                if len(data_list[i]) > self.max_points:
                    data_list[i].pop(0)
        
        # Update charts
        if HAS_PYQTGRAPH:
            for i in range(3):
                x = list(range(len(self.normal_data[i])))
                self.normal_curves[i].setData(x, self.normal_data[i])
                self.tangential_curves[i].setData(x, self.tangential_data[i])
                self.proximity_curves[i].setData(x, self.proximity_data[i])
        
        # Update value labels - only update labels that exist
        if hasattr(self, 'value_labels'):
            # Normal force labels
            normal_values = [n1, n2, n3]
            for i in range(self.sensor_count):
                key = f'Normal {i+1}'
                if key in self.value_labels:
                    val = normal_values[i]
                    self.value_labels[key].setText(f"{val} ({val*0.01:.2f}N)")
            
            # Tangential force labels
            tang_values = [t1, t2, t3]
            for i in range(self.sensor_count):
                key = f'Tang {i+1}'
                if key in self.value_labels:
                    val = tang_values[i]
                    self.value_labels[key].setText(f"{val} ({val*0.01:.2f}N)")
            
            # Proximity labels
            if 'Self Prox 1' in self.value_labels:
                self.value_labels['Self Prox 1'].setText(str(p1))
            if 'Self Prox 2' in self.value_labels:
                self.value_labels['Self Prox 2'].setText(str(p2))
            if 'Mutual Prox' in self.value_labels:
                self.value_labels['Mutual Prox'].setText(str(pm))
            
            # Status
            if 'Status' in self.value_labels:
                self.value_labels['Status'].setText(f"{status} ({'OK' if status == 0 else 'ERR'})")
                self.value_labels['Status'].setStyleSheet(
                    f"font-weight: bold; color: {COLORS['success']};" if status == 0 
                    else f"font-weight: bold; color: {COLORS['error']};"
                )
    
    def clear(self):
        """Clear chart data"""
        self.normal_data = [[], [], []]
        self.tangential_data = [[], [], []]
        self.proximity_data = [[], [], []]
        if HAS_PYQTGRAPH:
            for curves in [self.normal_curves, self.tangential_curves, self.proximity_curves]:
                for curve in curves:
                    curve.setData([], [])


class TouchSensorPanel(QWidget):
    """Touch Sensor Panel with multi-sensor support
    
    Uses SharedDataManager for device state.
    """
    
    def __init__(self):
        super().__init__()
        self.shared_data: Optional['SharedDataManager'] = None
        self.touch_data = None
        
        self._setup_ui()
        self.update_texts()
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_data_from_shared)
        self.update_timer.setInterval(50)
    
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
        """Setup UI with tabs for overview and finger details"""
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(8, 8, 8, 8)
        
        # Control buttons (compact)
        control_layout = QHBoxLayout()
        control_layout.setSpacing(8)
        
        self.enable_btn = QPushButton("Enable")
        self.enable_btn.clicked.connect(self._enable_touch)
        control_layout.addWidget(self.enable_btn)
        
        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.clicked.connect(self._calibrate)
        control_layout.addWidget(self.calibrate_btn)
        
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.clicked.connect(self._reset)
        control_layout.addWidget(self.reset_btn)
        
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self._clear_charts)
        control_layout.addWidget(self.clear_btn)
        
        # Finger checkboxes
        control_layout.addWidget(QLabel("|"))
        self.finger_checks = []
        for i in range(5):
            check = QCheckBox(FINGER_NAMES[i])  # Full name: Thumb, Index, Middle, Ring, Pinky
            check.setChecked(True)
            check.stateChanged.connect(lambda state, idx=i: self._on_finger_check_changed(idx, state))
            self.finger_checks.append(check)
            control_layout.addWidget(check)
        
        control_layout.addStretch()
        layout.addLayout(control_layout)
        
        # Tab widget
        self.tabs = QTabWidget()
        
        # Overview tab - charts
        overview_widget = QWidget()
        overview_main_layout = QVBoxLayout(overview_widget)
        overview_main_layout.setSpacing(8)
        overview_main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Charts grid (no control bar needed - all 3 data types shown)
        overview_layout = QGridLayout()
        overview_layout.setSpacing(8)
        overview_main_layout.addLayout(overview_layout, 1)
        
        # Left side: Force chart (all 5 fingers) - shows SUM of all sensors
        # Auto scale Y axis for better visualization
        self.normal_chart = MultiSensorChart("Normal Force (Total)", (0, 10), max_sensors=1)
        self.normal_chart.scale = 0.01
        self.normal_chart.auto_scale_y = True
        if HAS_PYQTGRAPH and hasattr(self.normal_chart, 'plot'):
            self.normal_chart.plot.setLabel('left', 'N', color='w')
        overview_layout.addWidget(self.normal_chart, 0, 0)
        
        # Right side: Finger status cards
        finger_status_widget = QWidget()
        finger_status_layout = QVBoxLayout(finger_status_widget)
        finger_status_layout.setSpacing(4)
        finger_status_layout.setContentsMargins(4, 4, 4, 4)
        
        finger_title = QLabel("📊 Finger Status")
        finger_title.setStyleSheet("font-weight: bold; font-size: 12px;")
        finger_status_layout.addWidget(finger_title)
        
        # Create finger status cards with progress bars
        self.finger_cards = []
        self.finger_force_bars = []
        self.finger_force_labels = []
        
        for i in range(5):
            card = QFrame()
            card.setStyleSheet(f"""
                QFrame {{
                    background-color: {COLORS['bg_card']};
                    border-left: 4px solid rgb{FINGER_COLORS[i]};
                    border-radius: 4px;
                    padding: 4px;
                }}
            """)
            card_layout = QHBoxLayout(card)
            card_layout.setContentsMargins(8, 4, 8, 4)
            card_layout.setSpacing(8)
            
            # Finger name
            name_label = QLabel(FINGER_NAMES[i])
            name_label.setFixedWidth(50)
            name_label.setStyleSheet(f"color: rgb{FINGER_COLORS[i]}; font-weight: bold;")
            card_layout.addWidget(name_label)
            
            # Force progress bar
            from PySide6.QtWidgets import QProgressBar
            force_bar = QProgressBar()
            force_bar.setRange(0, 2500)  # Raw value range
            force_bar.setValue(0)
            force_bar.setTextVisible(False)
            force_bar.setFixedHeight(16)
            r, g, b = FINGER_COLORS[i]
            force_bar.setStyleSheet(f"""
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
            card_layout.addWidget(force_bar, 1)
            self.finger_force_bars.append(force_bar)
            
            # Force value label
            force_label = QLabel("0.00 N")
            force_label.setFixedWidth(60)
            force_label.setAlignment(Qt.AlignRight)
            force_label.setStyleSheet("font-family: 'Menlo', 'Monaco', 'Courier New', monospace;")
            card_layout.addWidget(force_label)
            self.finger_force_labels.append(force_label)
            
            finger_status_layout.addWidget(card)
            self.finger_cards.append(card)
        
        finger_status_layout.addStretch()
        overview_layout.addWidget(finger_status_widget, 0, 1)
        
        # Bottom left: Tangential force chart - shows SUM of all sensors
        # Auto scale Y axis for better visualization
        self.tangential_chart = MultiSensorChart("Tangential Force (Total)", (0, 10), max_sensors=1)
        self.tangential_chart.scale = 0.01
        self.tangential_chart.auto_scale_y = True
        if HAS_PYQTGRAPH and hasattr(self.tangential_chart, 'plot'):
            self.tangential_chart.plot.setLabel('left', 'N', color='w')
        overview_layout.addWidget(self.tangential_chart, 1, 0)
        
        # Bottom right: Proximity chart - shows MAX of all sensors
        self.proximity_chart = MultiSensorChart("Proximity (Max)", (0, 1000), max_sensors=1)
        self.proximity_chart.auto_scale_y = True
        if HAS_PYQTGRAPH and hasattr(self.proximity_chart, 'plot'):
            self.proximity_chart.plot.setLabel('left', 'raw', color='w')
        overview_layout.addWidget(self.proximity_chart, 1, 1)
        
        # Remove old status labels (replaced by finger cards)
        self.status_labels = self.finger_force_labels
        
        self.tabs.addTab(overview_widget, "📊 Overview")
        
        # Finger detail tabs
        self.finger_details = []
        for i in range(5):
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            detail = FingerDetailWidget(i)
            scroll.setWidget(detail)
            self.finger_details.append(detail)
            self.tabs.addTab(scroll, FINGER_NAMES[i])  # Full name
        
        layout.addWidget(self.tabs, 1)

    def update_texts(self):
        """Update texts for i18n"""
        self.enable_btn.setText(tr("btn_enable_touch"))
        self.calibrate_btn.setText(tr("btn_calibrate"))
        self.reset_btn.setText(tr("btn_reset"))
        self.clear_btn.setText(tr("btn_clear"))
    
    def set_device(self, device, slave_id, device_info, shared_data=None):
        """Set device for touch sensor panel"""
        # Disconnect old signal if any
        if self.shared_data:
            try:
                self.shared_data.touch_updated.disconnect(self._on_touch_updated)
            except:
                pass
        
        self.shared_data = shared_data
        
        if not device:
            self.update_timer.stop()
            return
        
        # Check if device has capacitive touch (not pressure touch)
        from common_imports import uses_revo1_touch_api, uses_revo2_touch_api
        if device_info and hasattr(device_info, 'hardware_type'):
            hw_type = device_info.hardware_type
            # Capacitive touch: Revo1 touch API or Revo2 touch API (not pressure)
            is_capacitive_touch = uses_revo1_touch_api(hw_type) or uses_revo2_touch_api(hw_type)
            if not is_capacitive_touch:
                self.setEnabled(False)
                return
        
        self.setEnabled(True)
        
        # Connect to touch_updated signal instead of polling
        if shared_data:
            shared_data.touch_updated.connect(self._on_touch_updated)
        
        # Keep timer for UI refresh (chart updates, etc.)
        self.update_timer.start()
    
    def clear_device(self):
        """Clear device when disconnected"""
        self.update_timer.stop()
        if self.shared_data:
            try:
                self.shared_data.touch_updated.disconnect(self._on_touch_updated)
            except:
                pass
        self.shared_data = None
    
    def _on_touch_updated(self, touch_data):
        """Handle touch data from SharedDataManager signal"""
        if not touch_data:
            return
        self._process_touch_data(touch_data)
    
    def _update_data_from_shared(self):
        """Timer callback for UI refresh (chart updates, etc.)
        
        Note: Touch data is now received via signal, not polled here.
        """
        pass  # Charts are updated in _process_touch_data
    
    def _process_touch_data(self, touch_data):
        """Process touch data received from signal
        
        Uses touch data collected by DataCollector instead of direct API calls.
        """
        if not touch_data:
            return
        
        try:
            self.touch_data = touch_data
            
            for i in range(min(5, len(touch_data))):
                finger_data = touch_data[i]
                
                # Skip None data
                if finger_data is None:
                    continue
                
                # Get items list - could be TouchFingerData with items, or direct TouchFingerItem
                if hasattr(finger_data, 'items'):
                    items = finger_data.items
                else:
                    items = [finger_data]  # Single item
                
                if not items:
                    continue
                
                # Check if this finger is enabled (checkbox checked)
                finger_enabled = self.finger_checks[i].isChecked() if i < len(self.finger_checks) else True
                
                # Update overview charts based on selected sensor index
                first_item = items[0]
                
                # Get all force values for this finger
                n1 = getattr(first_item, 'normal_force1', 0)
                n2 = getattr(first_item, 'normal_force2', 0)
                n3 = getattr(first_item, 'normal_force3', 0)
                t1 = getattr(first_item, 'tangential_force1', 0)
                t2 = getattr(first_item, 'tangential_force2', 0)
                t3 = getattr(first_item, 'tangential_force3', 0)
                p1 = getattr(first_item, 'self_proximity1', 0)
                p2 = getattr(first_item, 'self_proximity2', 0)
                pm = getattr(first_item, 'mutual_proximity', 0)
                
                # Calculate total force (sum of all sensors) for overview
                # This gives a better representation of total finger force
                normal_total = n1 + n2 + n3
                tangential_total = t1 + t2 + t3
                
                # For proximity, use max value (most sensitive detection)
                proximity_max = max(p1, p2, pm)
                
                # Only update charts and status if finger is enabled
                if finger_enabled:
                    # Charts show total force (sum)
                    self.normal_chart.add_data(i, [normal_total])
                    self.tangential_chart.add_data(i, [tangential_total])
                    self.proximity_chart.add_data(i, [proximity_max])
                    
                    # Update finger status cards with total force
                    if hasattr(self, 'finger_force_bars') and i < len(self.finger_force_bars):
                        # Progress bar max based on sensor count (2500 per sensor)
                        sensor_count = REVO1_SENSOR_COUNTS[i]
                        max_force = 2500 * sensor_count
                        self.finger_force_bars[i].setRange(0, max_force)
                        self.finger_force_bars[i].setValue(min(normal_total, max_force))
                        force_n = normal_total * 0.01
                        self.finger_force_labels[i].setText(f"{force_n:.1f} N")
                        # Show card
                        if hasattr(self, 'finger_cards') and i < len(self.finger_cards):
                            self.finger_cards[i].setVisible(True)
                else:
                    # Hide card when finger is disabled
                    if hasattr(self, 'finger_cards') and i < len(self.finger_cards):
                        self.finger_cards[i].setVisible(False)
                
                # Always update finger detail view (it has its own tab)
                if i < len(self.finger_details):
                    self.finger_details[i].update_data(items)
                    
        except Exception as e:
            print(f"Process touch data failed: {e}")
    
    def _clear_charts(self):
        """Clear charts"""
        self.normal_chart.clear()
        self.tangential_chart.clear()
        self.proximity_chart.clear()
    
    def _on_finger_check_changed(self, finger_idx, state):
        """Finger checkbox changed - update UI visibility"""
        is_checked = state == Qt.Checked.value if hasattr(Qt.Checked, 'value') else state == 2
        
        # Show/hide finger card in Overview
        if hasattr(self, 'finger_cards') and finger_idx < len(self.finger_cards):
            self.finger_cards[finger_idx].setVisible(is_checked)
        
        # Show/hide curves in charts
        if HAS_PYQTGRAPH:
            for chart in [self.normal_chart, self.tangential_chart, self.proximity_chart]:
                if hasattr(chart, 'curves') and finger_idx < len(chart.curves):
                    for curve in chart.curves[finger_idx]:
                        curve.setVisible(is_checked)
    
    def _get_selected_fingers_bits(self):
        """Get selected fingers bitmask"""
        bits = 0
        for i, check in enumerate(self.finger_checks):
            if check.isChecked():
                bits |= (1 << i)
        return bits
    
    def _enable_touch(self):
        """Enable touch sensors"""
        if self.device:
            bits = self._get_selected_fingers_bits()
            run_async(self._async_enable_touch(bits))
    
    async def _async_enable_touch(self, bits):
        if not self.device:
            return
        try:
            await self.device.touch_sensor_setup(self.slave_id, bits)
        except Exception as e:
            print(f"Enable touch failed: {e}")
    
    def _calibrate(self):
        """Calibrate touch sensors with confirmation dialog"""
        if not self.device:
            return
        
        # Confirmation dialog
        reply = QMessageBox.question(
            self,
            tr("dialog_calibrate_title"),
            tr("dialog_calibrate_message"),
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            bits = self._get_selected_fingers_bits()
            run_async(self._async_calibrate(bits))
    
    async def _async_calibrate(self, bits):
        if not self.device:
            return
        try:
            await self.device.touch_sensor_calibrate(self.slave_id, bits)
        except Exception as e:
            print(f"Calibrate failed: {e}")
    
    def _reset(self):
        """Reset touch sensors with confirmation dialog"""
        if not self.device:
            return
        
        # Confirmation dialog
        reply = QMessageBox.question(
            self,
            tr("dialog_reset_title"),
            tr("dialog_reset_message"),
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            bits = self._get_selected_fingers_bits()
            run_async(self._async_reset(bits))
    
    async def _async_reset(self, bits):
        if not self.device:
            return
        try:
            await self.device.touch_sensor_reset(self.slave_id, bits)
        except Exception as e:
            print(f"Reset failed: {e}")
