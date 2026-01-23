"""
Revo2 Single Finger Timing Test with Real-time GUI (PySide6)

This script tests individual finger timing (Index, Middle, Ring, Pinky):
- Tests each finger separately: 0% to 100%
- Performs multiple cycles per finger
- Displays GUI with position curves

Usage:
    python revo2_timing_test_single_finger.py
"""

import asyncio
import sys
import time
from collections import deque
from PySide6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                                QWidget, QLabel, QPushButton, QComboBox, QSpinBox)
from PySide6.QtCore import QTimer, Signal, QObject
from PySide6.QtGui import QFont
import pyqtgraph as pg
from revo2_utils import *

# Test configuration
NUM_CYCLES = 5  # Number of open/close cycles per finger
MAX_INTERVAL = 1.0  # Hard timeout per movement (seconds)

# Finger configuration
FINGER_MAP = {
    'Index': 2,
    'Middle': 3,
    'Ring': 4,
    'Pinky': 5
}

FINGER_COLORS = {
    'Index': (69, 183, 209),   # Blue
    'Middle': (255, 160, 122),  # Orange
    'Ring': (152, 216, 200),    # Green
    'Pinky': (247, 220, 111)    # Yellow
}

MAX_PLOT_POINTS = 500


class PositionTracker(QObject):
    """Track finger positions over time for plotting"""
    data_updated = Signal()

    def __init__(self):
        super().__init__()
        self.times = deque(maxlen=MAX_PLOT_POINTS)
        self.positions = [deque(maxlen=MAX_PLOT_POINTS) for _ in range(6)]
        self.start_time = time.time()

    def add_data(self, positions):
        """Add new position data"""
        current_time = time.time() - self.start_time
        self.times.append(current_time)
        for i in range(6):
            self.positions[i].append(positions[i])
        self.data_updated.emit()

    def get_data(self):
        """Get current data for plotting"""
        return (
            list(self.times),
            [list(pos) for pos in self.positions]
        )

    def reset(self):
        """Reset tracking data"""
        self.times.clear()
        for pos in self.positions:
            pos.clear()
        self.start_time = time.time()


class SingleFingerTestWindow(QMainWindow):
    """Main window for single finger timing test"""

    def __init__(self, tracker):
        super().__init__()
        self.tracker = tracker
        self.setWindowTitle("Revo2 Single Finger Timing Test")
        self.setGeometry(100, 100, 1200, 800)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Control panel
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        
        # Finger selection
        control_layout.addWidget(QLabel("Finger:"))
        self.finger_combo = QComboBox()
        self.finger_combo.addItems(list(FINGER_MAP.keys()))
        control_layout.addWidget(self.finger_combo)
        
        # Cycles selection
        control_layout.addWidget(QLabel("Cycles:"))
        self.cycles_spin = QSpinBox()
        self.cycles_spin.setRange(1, 20)
        self.cycles_spin.setValue(NUM_CYCLES)
        control_layout.addWidget(self.cycles_spin)
        
        # Start button
        self.start_button = QPushButton("Start Test")
        self.start_button.setFont(QFont("Arial", 12, QFont.Bold))
        self.start_button.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                padding: 10px 20px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #229954;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
            }
        """)
        control_layout.addWidget(self.start_button)
        
        control_layout.addStretch()
        main_layout.addWidget(control_panel)

        # Status label
        self.status_label = QLabel("Ready")
        self.status_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.status_label.setStyleSheet("""
            padding: 12px;
            background-color: #2c3e50;
            color: #ecf0f1;
            border-radius: 5px;
        """)
        main_layout.addWidget(self.status_label)

        # Create plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        
        label_style = {'color': '#000', 'font-size': '16pt', 'font-weight': 'bold'}
        self.plot_widget.setLabel('left', 'Position', units='0-1000', **label_style)
        self.plot_widget.setLabel('bottom', 'Time', units='s', **label_style)
        self.plot_widget.setTitle('Single Finger Position Tracking', size='20pt', color='#000', bold=True)
        
        legend = self.plot_widget.addLegend()
        legend.setLabelTextSize('14pt')
        
        axis_font = QFont("Arial", 12, QFont.Bold)
        self.plot_widget.getAxis('left').setTickFont(axis_font)
        self.plot_widget.getAxis('bottom').setTickFont(axis_font)
        
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setYRange(-50, 1050)

        # Add target lines
        self.plot_widget.addLine(y=1000, pen=pg.mkPen('g', style=pg.QtCore.Qt.DashLine, width=2),
                                label='Target (100%)')
        self.plot_widget.addLine(y=0, pen=pg.mkPen('b', style=pg.QtCore.Qt.DashLine, width=2),
                                label='Open (0%)')

        main_layout.addWidget(self.plot_widget)

        # Results display
        self.results_label = QLabel("")
        self.results_label.setFont(QFont("Courier", 10))
        self.results_label.setStyleSheet("""
            padding: 10px;
            background-color: #ecf0f1;
            border-radius: 5px;
        """)
        main_layout.addWidget(self.results_label)

        # Create plot curves for all fingers
        self.curves = {}
        finger_names = ['Thumb Flex', 'Thumb Aux', 'Index', 'Middle', 'Ring', 'Pinky']
        for i, name in enumerate(finger_names):
            color = (200, 200, 200) if i < 2 else FINGER_COLORS.get(name, (128, 128, 128))
            pen = pg.mkPen(color=color, width=3 if i >= 2 else 1)
            curve = self.plot_widget.plot([], [], name=name, pen=pen)
            self.curves[i] = curve

        # Connect tracker signal
        self.tracker.data_updated.connect(self.update_plot)

        # Timer for auto-scaling
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(50)

    def update_plot(self):
        """Update plot with new data"""
        times, positions = self.tracker.get_data()

        if len(times) > 0:
            for i, curve in self.curves.items():
                curve.setData(times, positions[i])

            # Auto-scale x-axis
            if len(times) > 1:
                x_min = max(0, times[-1] - 10)
                x_max = times[-1] + 0.5
                self.plot_widget.setXRange(x_min, x_max)

    def update_status(self, text):
        """Update status label"""
        self.status_label.setText(text)

    def update_results(self, text):
        """Update results display"""
        self.results_label.setText(text)

    def set_testing(self, is_testing):
        """Enable/disable controls during test"""
        self.start_button.setEnabled(not is_testing)
        self.finger_combo.setEnabled(not is_testing)
        self.cycles_spin.setEnabled(not is_testing)


async def main():
    """Main function"""
    collector = None
    client = None

    # Create Qt application
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    tracker = PositionTracker()
    window = SingleFingerTestWindow(tracker)
    window.show()

    # Test state
    test_running = False

    async def run_test():
        nonlocal test_running, collector, client
        
        if test_running:
            return
        
        test_running = True
        window.set_testing(True)
        tracker.reset()
        
        try:
            selected_finger = window.finger_combo.currentText()
            num_cycles = window.cycles_spin.value()
            
            logger.info(f"Starting test for {selected_finger} ({num_cycles} cycles)")
            window.update_status(f"Testing {selected_finger}...")
            
            await run_single_finger_test(
                client, slave_id, motor_buffer, tracker, window,
                selected_finger, num_cycles
            )
            
        except Exception as e:
            logger.error(f"Test error: {e}", exc_info=True)
            window.update_status(f"Error: {e}")
        finally:
            test_running = False
            window.set_testing(False)

    # Connect button
    window.start_button.clicked.connect(lambda: asyncio.create_task(run_test()))

    try:
        # Connect to device
        logger.info("Connecting to device...")
        window.update_status("Connecting to device...")

        (protocol, detected_port_name, baudrate, detected_slave_id) = (
            await libstark.auto_detect_modbus_revo2(None, True)
        )
        logger.info(f"Detected: port={detected_port_name}, baudrate={baudrate}, slave_id=0x{detected_slave_id:X}")

        client = await libstark.modbus_open(detected_port_name, baudrate)
        slave_id = detected_slave_id

        device_info = await client.get_device_info(slave_id)
        logger.info(f"Device: {device_info.description}")

        # Set normalized mode
        await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized)

        # Create data collector
        logger.info("Creating data collector...")
        motor_buffer = libstark.MotorStatusBuffer(max_size=1000)

        import platform
        is_linux = platform.system() == 'Linux'
        motor_frequency = 100 if is_linux else 20

        collector = libstark.DataCollector.new_basic(
            ctx=client,
            motor_buffer=motor_buffer,
            slave_id=slave_id,
            motor_frequency=motor_frequency,
            enable_stats=True,
        )

        collector.start()
        await asyncio.sleep(1.0)

        # Verify collector
        test_data = motor_buffer.peek_all()
        if test_data:
            logger.info(f"✓ Collector working - {len(test_data)} samples")
            motor_buffer.clear()
        else:
            raise Exception("Collector not receiving data")

        window.update_status("Ready - Select finger and click Start Test")

        # Keep GUI running
        while window.isVisible():
            QApplication.processEvents()
            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        logger.info("User interrupted")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        window.update_status(f"Error: {e}")
    finally:
        if collector:
            collector.stop()
            await asyncio.sleep(0.5)
        if client:
            libstark.modbus_close(client)


async def run_single_finger_test(client, slave_id, motor_buffer, tracker, window, finger_name, num_cycles):
    """Run timing test for a single finger"""
    finger_idx = FINGER_MAP[finger_name]
    
    logger.info("=" * 60)
    logger.info(f"Testing {finger_name} (Index {finger_idx})")
    logger.info("=" * 60)

    # Keep other fingers at 0
    open_positions = [0, 0, 0, 0, 0, 0]
    close_positions = [0, 0, 0, 0, 0, 0]
    close_positions[finger_idx] = 1000  # Only move test finger to 100%
    durations = [1] * 6

    # Move to open position
    logger.info("Moving to initial position...")
    await client.set_finger_positions_and_durations(slave_id, open_positions, durations)
    await asyncio.sleep(2.0)

    close_times = []
    open_times = []

    # Run cycles
    for cycle in range(num_cycles):
        try:
            logger.info(f"\n{'='*60}")
            logger.info(f"CYCLE {cycle + 1}/{num_cycles}")
            logger.info(f"{'='*60}")

            # Close
            logger.info(f"[Cycle {cycle + 1}] CLOSE: {finger_name} 0% → 100%")
            window.update_status(f"{finger_name} - Cycle {cycle + 1}/{num_cycles} - CLOSING...")
            close_time = await measure_single_finger_movement(
                client, slave_id, motor_buffer, tracker,
                close_positions, durations, finger_idx, "CLOSE"
            )
            close_times.append(close_time)

            # Open
            logger.info(f"[Cycle {cycle + 1}] OPEN: {finger_name} → 0%")
            window.update_status(f"{finger_name} - Cycle {cycle + 1}/{num_cycles} - OPENING...")
            open_time = await measure_single_finger_movement(
                client, slave_id, motor_buffer, tracker,
                open_positions, durations, finger_idx, "OPEN"
            )
            open_times.append(open_time)

            QApplication.processEvents()

        except Exception as e:
            logger.error(f"Error in cycle {cycle + 1}: {e}")
            break

    # Calculate statistics
    if close_times and open_times:
        avg_close = sum(close_times) / len(close_times)
        avg_open = sum(open_times) / len(open_times)
        min_close = min(close_times)
        max_close = max(close_times)
        min_open = min(open_times)
        max_open = max(open_times)

        # Display results
        results_text = f"""
{finger_name} Timing Test Results ({len(close_times)} cycles)

CLOSE (0% → 100%):
  Average: {avg_close:.3f}s
  Min: {min_close:.3f}s, Max: {max_close:.3f}s
  Times: {', '.join(f'{t:.3f}s' for t in close_times)}

OPEN (100% → 0%):
  Average: {avg_open:.3f}s
  Min: {min_open:.3f}s, Max: {max_open:.3f}s
  Times: {', '.join(f'{t:.3f}s' for t in open_times)}
"""
        
        logger.info("\n" + "=" * 60)
        logger.info(f"{finger_name} TIMING TEST SUMMARY")
        logger.info("=" * 60)
        logger.info(results_text)
        logger.info("=" * 60)

        window.update_results(results_text)
        window.update_status(f"✓ {finger_name} Complete | Avg Close: {avg_close:.3f}s | Avg Open: {avg_open:.3f}s")


async def measure_single_finger_movement(client, slave_id, motor_buffer, tracker, 
                                        target_positions, durations, finger_idx, movement_name):
    """Measure time for single finger movement"""
    motor_buffer.clear()

    start_time = time.time()
    await client.set_finger_positions_and_durations(slave_id, target_positions, durations)

    movement_time = None
    last_log_time = start_time
    last_positions = None

    while True:
        await asyncio.sleep(0.02)
        elapsed = time.time() - start_time

        QApplication.processEvents()

        latest = motor_buffer.peek_latest()
        if not latest:
            if elapsed >= MAX_INTERVAL:
                logger.warning(f"  ⚠ Timeout at {elapsed:.3f}s (no data)")
                movement_time = elapsed
                break
            continue

        last_positions = list(latest.positions)
        tracker.add_data(latest.positions)

        # Log progress
        if elapsed - (last_log_time - start_time) > 0.2:
            logger.debug(f"  Progress at {elapsed:.3f}s: Finger[{finger_idx}]={last_positions[finger_idx]}")
            last_log_time = time.time()

        # Check if target reached (98% threshold)
        target = target_positions[finger_idx]
        current = latest.positions[finger_idx]

        reached = False
        if target == 0:
            reached = current <= 50  # 5% tolerance
        else:
            threshold = target * 0.98
            reached = current >= threshold

        if reached:
            movement_time = elapsed
            logger.info(f"  ✓ Reached target in {movement_time:.3f}s")
            logger.info(f"    Position: {current} (Target: {target})")
            break

        if elapsed >= MAX_INTERVAL:
            logger.warning(f"  ⚠ Timeout at {elapsed:.3f}s")
            logger.info(f"    Current: {current}, Target: {target}")
            completion = (current / target * 100) if target > 0 else 100
            logger.info(f"    Completion: {completion:.1f}%")
            movement_time = elapsed
            break

    return movement_time


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
