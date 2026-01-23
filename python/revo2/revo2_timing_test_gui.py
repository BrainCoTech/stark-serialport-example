"""
Revo2 Open/Close Timing Test with Real-time GUI (PySide6)

This script tests the open/close timing of the Revo2 hand:
- Thumb: 0% to 50%
- Other fingers: 0% to 100%
- Performs multiple cycles with real-time position tracking
- Displays GUI with position curves for all fingers

Uses high-performance data collector for real-time position monitoring.
"""

import asyncio
import sys
import time
from collections import deque
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel
from PySide6.QtCore import QTimer, Signal, QObject
from PySide6.QtGui import QFont
import pyqtgraph as pg
from revo2_utils import *

# Test configuration
NUM_CYCLES = 5  # Number of open/close cycles to perform
MAX_INTERVAL = 1.0  # Hard timeout per movement (seconds)

# GUI configuration
FINGER_NAMES = ['Thumb Flex', 'Thumb Aux', 'Index', 'Middle', 'Ring', 'Pinky']
FINGER_COLORS = [
    (255, 107, 107),  # Red
    (78, 205, 196),   # Teal
    (69, 183, 209),   # Blue
    (255, 160, 122),  # Orange
    (152, 216, 200),  # Green
    (247, 220, 111)   # Yellow
]
MAX_PLOT_POINTS = 500  # Maximum points to display on plot


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


class TimingTestWindow(QMainWindow):
    """Main window for timing test GUI"""

    def __init__(self, tracker):
        super().__init__()
        self.tracker = tracker
        self.setWindowTitle("Revo2 Timing Test - Position Tracking")
        self.setGeometry(100, 100, 1200, 700)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Status label - larger font and better contrast
        self.status_label = QLabel("Initializing...")
        self.status_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.status_label.setStyleSheet("""
            padding: 15px; 
            background-color: #2c3e50; 
            color: #ecf0f1;
            border-radius: 5px;
        """)
        layout.addWidget(self.status_label)

        # Create plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        
        # Increase axis label font sizes with better contrast
        label_style = {'color': '#000', 'font-size': '16pt', 'font-weight': 'bold'}
        self.plot_widget.setLabel('left', 'Position', units='0-1000', **label_style)
        self.plot_widget.setLabel('bottom', 'Time', units='s', **label_style)
        
        # Larger title
        self.plot_widget.setTitle('Finger Position Tracking', size='20pt', color='#000', bold=True)
        
        # Increase legend font size and make it more visible
        legend = self.plot_widget.addLegend()
        legend.setLabelTextSize('14pt')
        
        # Make axis tick labels larger and bolder
        axis_font = QFont("Arial", 12, QFont.Bold)
        self.plot_widget.getAxis('left').setTickFont(axis_font)
        self.plot_widget.getAxis('bottom').setTickFont(axis_font)
        self.plot_widget.getAxis('left').setStyle(tickTextOffset=10)
        self.plot_widget.getAxis('bottom').setStyle(tickTextOffset=10)
        
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setYRange(-50, 1050)

        # Add target lines with thicker width
        self.plot_widget.addLine(y=500, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine, width=2),
                                label='Thumb Target (50%)')
        self.plot_widget.addLine(y=1000, pen=pg.mkPen('g', style=pg.QtCore.Qt.DashLine, width=2),
                                label='Others Target (100%)')
        self.plot_widget.addLine(y=0, pen=pg.mkPen('b', style=pg.QtCore.Qt.DashLine, width=2),
                                label='Open (0%)')

        layout.addWidget(self.plot_widget)

        # Create plot curves
        self.curves = []
        for i, (name, color) in enumerate(zip(FINGER_NAMES, FINGER_COLORS)):
            pen = pg.mkPen(color=color, width=2)
            curve = self.plot_widget.plot([], [], name=name, pen=pen)
            self.curves.append(curve)

        # Connect tracker signal
        self.tracker.data_updated.connect(self.update_plot)

        # Timer for auto-scaling
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(50)  # Update every 50ms

    def update_plot(self):
        """Update plot with new data"""
        times, positions = self.tracker.get_data()

        if len(times) > 0:
            for i, curve in enumerate(self.curves):
                curve.setData(times, positions[i])

            # Auto-scale x-axis
            if len(times) > 1:
                x_min = max(0, times[-1] - 10)
                x_max = times[-1] + 0.5
                self.plot_widget.setXRange(x_min, x_max)

    def update_status(self, text):
        """Update status label"""
        self.status_label.setText(text)


async def main():
    """Main function: Initialize Revo2 and run timing test"""
    collector = None
    client = None

    # Create Qt application
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    tracker = PositionTracker()
    window = TimingTestWindow(tracker)
    window.show()

    try:
        # Connect to Revo2 device
        logger.info("Connecting to device...")
        window.update_status("Connecting to device...")

        (protocol, detected_port_name, baudrate, detected_slave_id) = (
            await libstark.auto_detect_modbus_revo2(None, True)
        )
        logger.info(f"Detected: protocol={protocol}, port={detected_port_name}, baudrate={baudrate}, slave_id=0x{detected_slave_id:X}")

        # Open connection
        client = await libstark.modbus_open(detected_port_name, baudrate)
        slave_id = detected_slave_id

        device_info = await client.get_device_info(slave_id)
        logger.info(f"Device info: {device_info.description}")
        logger.info(f"Using slave_id: 0x{slave_id:X} ({slave_id})")

        # Set normalized mode
        await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized)
        logger.info("Finger unit mode set to Normalized")

        # Create shared buffer for motor status
        logger.info("Creating shared buffer...")
        motor_buffer = libstark.MotorStatusBuffer(max_size=1000)

        # Create data collector
        import platform
        is_linux = platform.system() == 'Linux'
        motor_frequency = 100 if is_linux else 20

        logger.info(f"Creating data collector ({motor_frequency}Hz)...")
        logger.info(f"Platform: {platform.system()}")
        window.update_status(f"Creating data collector ({motor_frequency}Hz)...")

        collector = libstark.DataCollector.new_basic(
            ctx=client,
            motor_buffer=motor_buffer,
            slave_id=slave_id,
            motor_frequency=motor_frequency,
            enable_stats=True,
        )

        # Start collector
        logger.info("Starting data collector...")
        window.update_status("Starting data collector...")
        collector.start()

        # Wait for collector to initialize and start collecting
        logger.info("Waiting for collector to start collecting data...")
        await asyncio.sleep(1.0)  # Give it more time to start

        # Check if collector is working
        test_data = motor_buffer.peek_all()
        logger.info(f"Buffer check: {len(test_data)} samples in buffer")

        if test_data:
            logger.info(f"✓ Collector working - received {len(test_data)} samples")
            logger.info(f"  Sample positions: {list(test_data[-1].positions)}")
            motor_buffer.clear()
        else:
            logger.warning("⚠ No data from collector yet, waiting longer...")
            await asyncio.sleep(1.0)
            test_data = motor_buffer.peek_all()
            logger.info(f"Buffer check after wait: {len(test_data)} samples in buffer")

            if test_data:
                logger.info(f"✓ Collector working - received {len(test_data)} samples")
                logger.info(f"  Sample positions: {list(test_data[-1].positions)}")
                motor_buffer.clear()
            else:
                logger.error("✗ Collector not receiving data!")
                collector.stop()
                await asyncio.sleep(0.5)  # Wait for stop
                collector = None
                window.update_status("Error: Collector not receiving data")
                return

        # Run timing test
        await run_timing_test(client, slave_id, motor_buffer, tracker, window, collector)

    except KeyboardInterrupt:
        logger.info("User interrupted")
        window.update_status("User interrupted")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        window.update_status(f"Error: {e}")
    finally:
        # Stop collector
        if collector:
            logger.info("Stopping data collector...")
            try:
                collector.stop()
                # Wait for collector thread to actually stop
                await asyncio.sleep(0.5)
                logger.info("Data collector stopped")
            except Exception as e:
                logger.error(f"Error stopping collector: {e}")

        # Clean up
        if client:
            try:
                libstark.modbus_close(client)
                logger.info("Modbus client closed")
            except Exception as e:
                logger.error(f"Error closing client: {e}")

        logger.info("Test completed - Close window to exit")
        window.update_status("Test completed - Close window to exit")
        
        # Keep GUI open - wait for user to close window
        logger.info("GUI will remain open. Close the window when done.")
        
        # Run Qt event loop to keep window responsive
        while window.isVisible():
            QApplication.processEvents()
            await asyncio.sleep(0.1)


async def run_timing_test(client, slave_id, motor_buffer, tracker, window, collector):
    """Run open/close timing test with GUI updates"""
    logger.info("=" * 60)
    logger.info(f"Starting Open/Close Timing Test ({NUM_CYCLES} cycles)")
    logger.info("=" * 60)

    # Test parameters
    open_positions = [0, 0, 0, 0, 0, 0]
    close_positions = [500, 1000, 1000, 1000, 1000, 1000]
    durations = [1] * 6

    # Move to open position first
    logger.info("Moving to initial open position (0%)...")
    window.update_status("Moving to initial position...")
    await client.set_finger_positions_and_durations(slave_id, open_positions, durations)
    await asyncio.sleep(2.0)

    # Get initial status
    motor_buffer.clear()
    await asyncio.sleep(0.1)
    latest = motor_buffer.peek_latest()
    if latest:
        logger.info(f"Initial positions: {list(latest.positions)}")
        tracker.add_data(latest.positions)
    else:
        logger.warning("No initial position data available")

    # Store timing results
    close_times = []
    open_times = []

    # Perform multiple cycles
    for cycle in range(NUM_CYCLES):
        try:
            logger.info(f"\n{'='*60}")
            logger.info(f"CYCLE {cycle + 1}/{NUM_CYCLES}")
            logger.info(f"{'='*60}")

            # Test closing
            logger.info(f"\n[Cycle {cycle + 1}] CLOSE: 0% → Thumb:50%, Others:100%")
            window.update_status(f"Cycle {cycle + 1}/{NUM_CYCLES} - CLOSING...")
            close_time = await measure_movement(
                client, slave_id, motor_buffer, tracker,
                close_positions, durations, "CLOSE"
            )
            close_times.append(close_time)

            # Test opening
            logger.info(f"\n[Cycle {cycle + 1}] OPEN: → All:0%")
            window.update_status(f"Cycle {cycle + 1}/{NUM_CYCLES} - OPENING...")
            open_time = await measure_movement(
                client, slave_id, motor_buffer, tracker,
                open_positions, durations, "OPEN"
            )
            open_times.append(open_time)

            # Process Qt events
            QApplication.processEvents()

        except KeyboardInterrupt:
            logger.info("User interrupted during cycle")
            break
        except Exception as e:
            logger.error(f"Error in cycle {cycle + 1}: {e}")
            if len(close_times) <= cycle:
                close_times.append(MAX_INTERVAL)
            if len(open_times) <= cycle:
                open_times.append(MAX_INTERVAL)
            break

    # Calculate statistics
    if not close_times or not open_times:
        logger.warning("No timing data collected")
        window.update_status("Test completed - No data collected")
        return

    avg_close = sum(close_times) / len(close_times)
    avg_open = sum(open_times) / len(open_times)
    min_close = min(close_times)
    max_close = max(close_times)
    min_open = min(open_times)
    max_open = max(open_times)

    # Summary
    logger.info("\n" + "=" * 60)
    logger.info("TIMING TEST SUMMARY")
    logger.info("=" * 60)
    logger.info(f"Number of cycles: {len(close_times)}")
    logger.info(f"Hard timeout per movement: {MAX_INTERVAL}s")
    logger.info(f"Control mode: Position with duration=1ms (fastest speed)")
    logger.info("")
    logger.info("CLOSE times (0% → Thumb:50%, Others:100%):")
    for i, t in enumerate(close_times, 1):
        logger.info(f"  Cycle {i}: {t:.3f}s")
    logger.info(f"  Average: {avg_close:.3f}s")
    logger.info(f"  Min: {min_close:.3f}s, Max: {max_close:.3f}s")
    logger.info("")
    logger.info("OPEN times (→ All:0%):")
    for i, t in enumerate(open_times, 1):
        logger.info(f"  Cycle {i}: {t:.3f}s")
    logger.info(f"  Average: {avg_open:.3f}s")
    logger.info(f"  Min: {min_open:.3f}s, Max: {max_open:.3f}s")
    logger.info("=" * 60)

    # Update GUI status
    status_text = f"✓ Complete | Avg Close: {avg_close:.3f}s | Avg Open: {avg_open:.3f}s | Cycles: {len(close_times)}"
    window.update_status(status_text)


async def measure_movement(client, slave_id, motor_buffer, tracker, target_positions, durations, movement_name):
    """Measure time for a single movement with GUI updates

    IMPORTANT: set_finger_positions_and_durations() is NOT blocking - it just sends the command.
    The actual movement takes time, which we monitor via the data collector buffer.
    """
    # Clear buffer before sending command
    motor_buffer.clear()

    # Send command - this returns immediately, does NOT wait for movement to complete
    start_time = time.time()
    await client.set_finger_positions_and_durations(slave_id, target_positions, durations)
    command_time = time.time() - start_time
    logger.debug(f"  Command sent in {command_time*1000:.1f}ms (non-blocking)")

    # Monitor until all fingers reach target
    movement_time = None
    # Relaxed tolerance: 98% completion is good enough
    # For target 1000: accept >= 980
    # For target 500: accept >= 490
    # For target 0: accept <= 20
    last_log_time = start_time
    no_data_count = 0
    last_positions = None

    while True:
        await asyncio.sleep(0.02)
        elapsed = time.time() - start_time

        # Process Qt events
        QApplication.processEvents()

        # Get latest data from buffer (non-destructive read)
        latest = motor_buffer.peek_latest()
        if not latest:
            no_data_count += 1
            if no_data_count > 5 and elapsed - (last_log_time - start_time) > 0.2:
                logger.debug(f"  No data from buffer at {elapsed:.3f}s (count: {no_data_count})")
                last_log_time = time.time()

            if elapsed >= MAX_INTERVAL:
                logger.warning(f"  ⚠ Hard timeout at {elapsed:.3f}s (no data)")
                if last_positions:
                    logger.info(f"    Last known positions: {last_positions}")
                movement_time = elapsed
                break
            continue

        no_data_count = 0
        last_positions = list(latest.positions)

        # Update tracker for GUI
        tracker.add_data(latest.positions)

        # Log progress
        if elapsed - (last_log_time - start_time) > 0.2:
            logger.debug(f"  Progress at {elapsed:.3f}s: {last_positions}")
            last_log_time = time.time()

        # Check if reached target (Thumb: 90%, Others: 98%)
        # Revo2 finger indices: 0=Thumb Aux, 1=Thumb Flex, 2=Index, 3=Middle, 4=Ring, 5=Pinky
        all_reached = True
        for i in range(6):
            target = target_positions[i]
            current = latest.positions[i]

            if target == 0:
                # Opening: accept <= 5% (50 out of 1000)
                if current > 50:
                    all_reached = False
                    break
            else:
                # Closing: Thumb (0,1) 90%, Others (2-5) 98%
                threshold_ratio = 0.90 if i <= 1 else 0.98
                threshold = target * threshold_ratio
                if current < threshold:
                    all_reached = False
                    break

        if all_reached:
            movement_time = elapsed
            logger.info(f"  ✓ Reached target in {movement_time:.3f}s (Thumb: 90%, Others: 98%)")
            logger.info(f"    Positions: {last_positions}")
            logger.info(f"    Targets: {target_positions}")
            break

        if elapsed >= MAX_INTERVAL:
            logger.warning(f"  ⚠ Hard timeout at {elapsed:.3f}s")
            logger.info(f"    Current: {last_positions}")
            logger.info(f"    Target: {target_positions}")
            # Calculate completion percentage
            completions = []
            for i in range(6):
                if target_positions[i] == 0:
                    comp = 100 if latest.positions[i] <= 20 else (1000 - latest.positions[i]) / 10
                else:
                    comp = (latest.positions[i] / target_positions[i]) * 100
                completions.append(comp)
            logger.info(f"    Completion: {[f'{c:.1f}%' for c in completions]}")
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
