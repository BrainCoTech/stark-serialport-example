"""
Revo2 Modulus (Pressure) - High-Performance Data Collector Example with Trajectory Control

This example demonstrates high-performance data collection for pressure touch sensors
while simultaneously running trajectory control.
Supports three modes:
- Summary mode: Fast pressure monitoring (100Hz) + Trajectory control (100Hz)
- Detailed mode: Detailed sensor point analysis (10Hz) + Trajectory control (100Hz)
- Dual mode: Summary + Detailed simultaneously (100Hz + 10Hz) + Trajectory control (100Hz)

Hardware Support:
- ✅ Revo2 Touch (Pressure Touch Sensor)

Usage:
    # Summary mode (default)
    python revo2_touch_pressure_collector.py

    # Detailed mode
    python revo2_touch_pressure_collector.py --mode detailed

    # Dual mode (Summary + Detailed)
    python revo2_touch_pressure_collector.py --mode dual
"""

import asyncio
import sys
import argparse
from pathlib import Path
from revo2_utils import *
from utils import setup_shutdown_event

# Trajectory control parameters
TRAJ_LEN = 20           # Number of trajectory points
CTRL_INTERVAL = 0.01    # Control interval: 10ms (100Hz)
ENABLE_CONTROL = True   # Enable/disable trajectory control


async def main():
    """Main function: High-performance pressure sensor data collection"""

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Revo2 Modulus Pressure Sensor Data Collection')
    parser.add_argument('--mode', type=str, default='summary',
                       choices=['summary', 'detailed', 'dual'],
                       help='Collection mode: summary (fast), detailed (precise), dual (both)')
    args = parser.parse_args()

    mode = args.mode
    logger.info(f"Collection mode: {mode}")

    # Setup shutdown event listener
    shutdown_event = setup_shutdown_event(logger)

    # Open Modbus connection
    logger.info("Connecting to device...")
    (client, detected_slave_id) = await open_modbus_revo2()
    slave_id = detected_slave_id

    # Get device info
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    # Verify device has touch sensor
    if not device_info.is_touch():
        logger.error("This device does not support touch sensor!")
        sys.exit(1)

    # Check touch sensor type
    if not client.is_touch_pressure(slave_id):
        logger.error("This example is for pressure touch sensors (Modulus)!")
        logger.error("Current device has capacitive touch sensor")
        logger.error("Please use revo2_touch_buffer.py instead")
        sys.exit(1)

    logger.info("Detected pressure touch sensor (Modulus)")

    # ========================================================================
    # Create shared buffers based on mode
    # ========================================================================

    logger.info(f"Creating shared buffers for {mode} mode...")
    motor_buffer = libstark.MotorStatusBuffer(max_size=1000)

    if mode == 'summary':
        pressure_summary_buffer = libstark.PressureSummaryBuffer(max_size=1000)
        pressure_detailed_buffer = None
    elif mode == 'detailed':
        pressure_summary_buffer = None
        pressure_detailed_buffer = libstark.PressureDetailedBuffer(max_size=1000)
    else:  # dual
        pressure_summary_buffer = libstark.PressureSummaryBuffer(max_size=1000)
        pressure_detailed_buffer = libstark.PressureDetailedBuffer(max_size=1000)

    logger.info("Shared buffers created")

    # ========================================================================
    # Enable touch sensor
    # ========================================================================

    logger.info("Enabling pressure sensor...")
    bits = 0x3F  # Enable all 5 fingers + palm
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)

    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Pressure sensor enabled status: {(bits & 0x3F):06b}")

    # ========================================================================
    # Create and start data collector based on mode
    # ========================================================================

    logger.info(f"Creating data collector ({mode} mode)...")

    # Define frequencies (platform-dependent)
    # Linux: 100Hz motor, 100Hz summary, 10Hz detailed (high-performance)
    # Windows/macOS: 20Hz motor, 20Hz summary, 10Hz detailed (conservative)
    import platform
    is_linux = platform.system() == 'Linux'
    motor_frequency = 100 if is_linux else 20
    summary_frequency = 100 if is_linux else 20
    detailed_frequency = 10  # Same for all platforms (detailed data is large)
    
    logger.info(f"Platform: {platform.system()}, Motor: {motor_frequency}Hz, Summary: {summary_frequency}Hz, Detailed: {detailed_frequency}Hz")

    if mode == 'summary':
        assert pressure_summary_buffer is not None
        collector = libstark.DataCollector.new_pressure_summary(
            ctx=client,
            motor_buffer=motor_buffer,
            pressure_summary_buffer=pressure_summary_buffer,
            slave_id=slave_id,
            motor_frequency=motor_frequency,
            touch_frequency=summary_frequency,
            enable_stats=True,
        )
    elif mode == 'detailed':
        assert pressure_detailed_buffer is not None
        collector = libstark.DataCollector.new_pressure_detailed(
            ctx=client,
            motor_buffer=motor_buffer,
            pressure_detailed_buffer=pressure_detailed_buffer,
            slave_id=slave_id,
            motor_frequency=motor_frequency,
            touch_frequency=detailed_frequency,
            enable_stats=True,
        )
    else:  # dual
        assert pressure_summary_buffer is not None
        assert pressure_detailed_buffer is not None
        collector = libstark.DataCollector.new_pressure_hybrid(
            ctx=client,
            motor_buffer=motor_buffer,
            pressure_summary_buffer=pressure_summary_buffer,
            pressure_detailed_buffer=pressure_detailed_buffer,
            slave_id=slave_id,
            motor_frequency=motor_frequency,
            summary_frequency=summary_frequency,
            detailed_frequency=detailed_frequency,
            enable_stats=True,
        )

    logger.info("Data collector created")

    # ========================================================================
    # Start data collection
    # ========================================================================

    logger.info("Starting data collector...")
    collector.start()
    logger.info("Data collector started (background thread)")

    if mode == 'dual':
        logger.info("Dual mode: Summary 100Hz + Detailed 10Hz")

    logger.info("Please touch the sensor to see pressure data...")

    # ========================================================================
    # Start trajectory control task (optional)
    # ========================================================================

    stop_flag = {'value': False}
    index_ref = {'value': 0}
    control_task = None

    if ENABLE_CONTROL:
        logger.info("Initializing trajectory control...")
        # Generate trajectory (0-100% range)
        # Note: Thumb trajectory will be auto-clamped to max 50% if needed
        trajectory = init_cosine_trajectory(traj_len=TRAJ_LEN)
        finger_id = libstark.FingerId.Ring  # Control ring finger

        control_task = asyncio.create_task(
            trajectory_control_task(
                client, slave_id, finger_id, trajectory,
                ctrl_interval=CTRL_INTERVAL,
                stop_flag_ref=stop_flag,
                index_ref=index_ref
            )
        )
        logger.info("Trajectory control task started")

    # ========================================================================
    # Start data processing tasks
    # ========================================================================

    # Start data processing tasks with intervals matching collection frequencies
    asyncio.create_task(process_motor_data(motor_buffer, motor_frequency, index_ref if ENABLE_CONTROL else None))

    if pressure_summary_buffer:
        asyncio.create_task(process_pressure_summary_data(pressure_summary_buffer, summary_frequency, mode))

    if pressure_detailed_buffer:
        asyncio.create_task(process_pressure_detailed_data(pressure_detailed_buffer, detailed_frequency, mode))

    logger.info("Data processing tasks started")
    logger.info("Press Ctrl+C to stop...")

    # Wait for shutdown event
    await shutdown_event.wait()

    # ========================================================================
    # Stop and cleanup
    # ========================================================================

    logger.info("Stopping trajectory control...")
    if ENABLE_CONTROL and control_task:
        stop_flag['value'] = True
        await control_task
        logger.info("Trajectory control stopped")

    logger.info("Stopping data collector...")
    collector.stop()
    # Give a small delay to ensure logs are flushed
    await asyncio.sleep(0.1)
    logger.info("Data collector stopped")

    # Cleanup resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def process_motor_data(motor_buffer, frequency, index_ref=None):
    """Process motor data
    
    Args:
        motor_buffer: Motor data buffer
        frequency: Collection frequency in Hz (determines processing interval)
        index_ref: Optional trajectory index reference for correlation
    """
    # Calculate processing interval based on frequency
    # Use 2x frequency for processing to avoid buffer overflow
    process_interval = 1.0 / (frequency * 2)  # seconds
    
    logger.info(f"Motor data processing task started (interval: {process_interval*1000:.1f}ms)")
    process_count = 0

    while True:
        try:
            if motor_buffer.is_empty():
                await asyncio.sleep(process_interval)
                continue

            # Batch get data
            data_batch = motor_buffer.pop_all()

            if len(data_batch) > 0:
                if process_count % 10 == 0:  # Print every 10 times
                    traj_info = f" [Traj Index: {index_ref['value']}]" if index_ref else ""
                    logger.debug(f"[Motor-{process_count}]{traj_info} Processed {len(data_batch)} motor data")

                    # Print latest data
                    latest = data_batch[-1]
                    logger.debug(f"Positions: {latest.positions[:3]}...")

                process_count += 1

            await asyncio.sleep(process_interval)

        except Exception as e:
            logger.error(f"Motor data processing error: {e}")
            await asyncio.sleep(1)


async def process_pressure_summary_data(pressure_summary_buffer, frequency, mode):
    """Process pressure sensor Summary data
    
    Args:
        pressure_summary_buffer: Pressure summary data buffer
        frequency: Collection frequency in Hz (determines processing interval)
        mode: Collection mode ('summary', 'detailed', or 'dual')
    """
    # Calculate processing interval based on frequency
    # Use 2x frequency for processing to avoid buffer overflow
    process_interval = 1.0 / (frequency * 2)  # seconds
    
    mode_label = "Summary" if mode == 'summary' else "Summary-Dual"
    logger.info(f"Pressure sensor {mode_label} data processing task started (interval: {process_interval*1000:.1f}ms)")
    process_count = 0

    # Part names (5 fingers + 1 palm)
    part_names = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"]

    while True:
        try:
            # Check buffer data count
            data_counts = pressure_summary_buffer.len_all()
            total_count = sum(data_counts)

            if total_count == 0:
                await asyncio.sleep(0.01)
                continue

            # Get all parts pressure data (5 fingers + 1 palm)
            all_data = pressure_summary_buffer.pop_all()

            print_interval = 10 if mode == 'dual' else 1
            if process_count % print_interval == 0:
                logger.info(
                    f"[{mode_label}-{process_count}] Pressure data: "
                    f"Thumb={len(all_data[0])}, "
                    f"Index={len(all_data[1])}, "
                    f"Middle={len(all_data[2])}, "
                    f"Ring={len(all_data[3])}, "
                    f"Pinky={len(all_data[4])}, "
                    f"Palm={len(all_data[5])}"
                )

                # Print each part's latest pressure value
                for part_idx in range(6):
                    if len(all_data[part_idx]) > 0:
                        # Get latest pressure value
                        latest_pressure = all_data[part_idx][-1]

                        # Check if there is contact
                        is_contact = latest_pressure > 50  # Threshold: 50mN

                        if is_contact:
                            print(
                                f"  {part_names[part_idx]}: {latest_pressure} mN [CONTACT]"
                            )

            process_count += 1
            await asyncio.sleep(process_interval)

        except Exception as e:
            logger.error(f"Summary data processing error: {e}")
            await asyncio.sleep(1)


async def process_pressure_detailed_data(pressure_detailed_buffer, frequency, mode):
    """Process pressure sensor Detailed data
    
    Args:
        pressure_detailed_buffer: Pressure detailed data buffer
        frequency: Collection frequency in Hz (determines processing interval)
        mode: Collection mode ('summary', 'detailed', or 'dual')
    """
    # Calculate processing interval based on frequency
    # Use 2x frequency for processing to avoid buffer overflow
    process_interval = 1.0 / (frequency * 2)  # seconds
    
    mode_label = "Detailed" if mode == 'detailed' else "Detailed-Dual"
    logger.info(f"Pressure sensor {mode_label} data processing task started (interval: {process_interval*1000:.1f}ms)")
    process_count = 0

    # Part names (5 fingers + 1 palm)
    part_names = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"]

    while True:
        try:
            # Check buffer data count
            data_counts = pressure_detailed_buffer.len_all()
            total_count = sum(data_counts)

            if total_count == 0:
                await asyncio.sleep(process_interval)
                continue

            # Get all parts detailed pressure data (5 fingers + 1 palm)
            all_data = pressure_detailed_buffer.pop_all()

            if process_count % 10 == 0:  # Print every 10 times
                logger.info(f"[{mode_label}-{process_count}] Processed {len(all_data[0])} detailed pressure data")

            # Print each part's detailed data
            for part_idx in range(6):
                if len(all_data[part_idx]) > 0:
                    # Get latest data
                    latest_item = all_data[part_idx][-1]

                    # Get valid sensor data
                    sensors = latest_item.get_sensors()
                    sensor_count = latest_item.sensor_count

                    # Calculate statistics
                    total_pressure = sum(sensors)
                    max_pressure = max(sensors)
                    avg_pressure = total_pressure / sensor_count if sensor_count > 0 else 0

                    # Check if there is contact
                    is_contact = max_pressure > 50  # Threshold: 50mN

                    if is_contact:
                        print(
                            f"  {part_names[part_idx]}: "
                            f"sensors={sensor_count}, "
                            f"total={total_pressure}mN, "
                            f"max={max_pressure}mN, "
                            f"avg={avg_pressure:.1f}mN "
                            f"[CONTACT]"
                        )

                        # Find max pressure sensor point
                        max_idx = sensors.index(max_pressure)
                        print(f"    Max pressure sensor: #{max_idx}")

            process_count += 1
            await asyncio.sleep(process_interval)

        except Exception as e:
            logger.error(f"Detailed data processing error: {e}")
            await asyncio.sleep(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
