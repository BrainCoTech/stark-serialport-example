"""
Revo1 Touch (Capacitive) - High-Performance Data Collector Example with Trajectory Control

This example demonstrates high-performance data collection for capacitive touch sensors
while simultaneously running trajectory control:
- Motor status data collection at 100Hz
- Capacitive touch sensor data collection at 100Hz
- Trajectory control at 100Hz
- Efficient data collection without GIL overhead
- Observe motor response and touch feedback in real-time

Hardware Support:
- ✅ Revo1 Touch

Usage:
    python revo1_touch_collector.py
"""

import asyncio
import sys
import time
from revo1_utils import *
from utils import setup_shutdown_event

# Trajectory control parameters
TRAJ_LEN = 20           # Number of trajectory points
CTRL_INTERVAL = 0.01    # Control interval: 10ms (100Hz)
ENABLE_CONTROL = True   # Enable/disable trajectory control


async def main():
    """Main function: High-performance capacitive touch data collection"""

    # Setup shutdown event listener
    shutdown_event = setup_shutdown_event(logger)

    # Open Modbus connection
    logger.info("Connecting to device...")
    (client, detected_slave_id) = await open_modbus_revo1()
    slave_id = detected_slave_id

    # Get device info
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    # Verify device has touch sensor
    if not device_info.is_revo1_touch():
        logger.error("This device does not support touch sensor!")
        sys.exit(1)

    logger.info("Detected capacitive touch sensor")

    # ========================================================================
    # Create shared buffers
    # ========================================================================

    logger.info("Creating shared buffers...")
    motor_buffer = libstark.MotorStatusBuffer(max_size=1000)
    touch_buffer = libstark.TouchStatusBuffer(max_size=1000)
    logger.info("Shared buffers created")

    # ========================================================================
    # Enable touch sensor
    # ========================================================================

    logger.info("Enabling touch sensor...")
    bits = 0x1F  # Enable all 5 fingers
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)

    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch sensor enabled status: {(bits & 0x1F):05b}")

    # ========================================================================
    # Create and start data collector - Capacitive mode
    # ========================================================================

    logger.info("Creating data collector (Capacitive mode)...")

    # Collection frequencies (platform-dependent)
    # Linux: 50Hz motor, 10Hz touch (Revo1 is slower than Revo2)
    # Windows/macOS: 20Hz motor, 10Hz touch (conservative)
    import platform
    is_linux = platform.system() == 'Linux'
    motor_frequency = 50 if is_linux else 20  # Revo1 motor is slower
    touch_frequency = 10  # Touch: 10Hz (same for all platforms)
    
    logger.info(f"Platform: {platform.system()}, Motor: {motor_frequency}Hz, Touch: {touch_frequency}Hz")

    collector = libstark.DataCollector.new_capacitive(
        ctx=client,
        motor_buffer=motor_buffer,
        touch_buffer=touch_buffer,
        slave_id=slave_id,
        motor_frequency=motor_frequency,
        touch_frequency=touch_frequency,
        enable_stats=True,
    )

    logger.info("Data collector created")

    # ========================================================================
    # Start data collection
    # ========================================================================

    logger.info("Starting data collector...")
    collector.start()
    logger.info("Data collector started (background thread)")
    logger.info("Please touch the sensor to see touch data...")

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
    asyncio.create_task(process_motor_data(motor_buffer, index_ref if ENABLE_CONTROL else None))
    asyncio.create_task(process_touch_data(touch_buffer, touch_frequency))

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


async def process_motor_data(motor_buffer, index_ref=None):
    """
    Process motor data

    Args:
        motor_buffer: Motor status buffer
        index_ref: Optional trajectory index reference for correlation
    """
    logger.info("Motor data processing task started")
    process_count = 0

    while True:
        try:
            if motor_buffer.is_empty():
                await asyncio.sleep(0.01)
                continue

            # Batch get data
            data_batch = motor_buffer.pop_all()

            if len(data_batch) > 0:
                if process_count % 2 == 0:  # Print every 2 times
                    traj_info = f" [Traj Index: {index_ref['value']}]" if index_ref else ""
                    logger.debug(f"[{process_count}]{traj_info} Processed {len(data_batch)} motor data")

                    # Print latest data
                    latest = data_batch[-1]
                    # logger.info(f"Positions: {latest.positions[:6]}")
                    # logger.info(f"Speeds: {latest.speeds[:6]}")
                    # logger.info(f"Currents: {latest.currents[:6]}")
                    logger.info(f"Finger Detail: {latest.description}")

                process_count += 1

            await asyncio.sleep(0.01)  # 10ms processing interval

        except Exception as e:
            logger.error(f"Motor data processing error: {e}")
            await asyncio.sleep(1)


async def process_touch_data(touch_buffer, frequency):
    """Process capacitive touch data
    
    Args:
        touch_buffer: Touch data buffer
        frequency: Collection frequency in Hz (determines processing interval)
    """
    # Calculate processing interval based on frequency
    # Use 2x frequency for processing to avoid buffer overflow
    process_interval = 1.0 / (frequency * 2)  # seconds
    
    logger.info(f"Capacitive touch data processing task started (interval: {process_interval*1000:.1f}ms)")
    process_count = 0

    # Finger names
    finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

    while True:
        try:
            # Check buffer data count
            data_counts = touch_buffer.len_all()
            total_count = sum(data_counts)

            if total_count == 0:
                await asyncio.sleep(process_interval)
                continue

            # Get all finger touch data
            all_data = touch_buffer.pop_all()

            if process_count % 10 == 0:  # Print every 10 times
                logger.info(f"[{process_count}] Processed {len(all_data[0])} touch data")

            # Print each finger's latest data
            for finger_idx in range(5):
                if len(all_data[finger_idx]) > 0:
                    # Get latest data
                    latest_item: libstark.TouchFingerItem = all_data[finger_idx][-1]

                    # Check if there is contact
                    is_contact = latest_item.normal_force1 > 50  # Threshold: 50

                    if is_contact:
                        print(
                            f"  {finger_names[finger_idx]}: "
                            f"normal_force={latest_item.normal_force1}, "
                            f"tangential_force={latest_item.tangential_force1}, "
                            f"proximity={latest_item.self_proximity1} "
                            f"[CONTACT]"
                        )
                    else:
                        logger.debug(
                            f"  {finger_names[finger_idx]}: "
                            f"normal_force={latest_item.normal_force1}"
                        )

            process_count += 1
            await asyncio.sleep(process_interval)

        except Exception as e:
            logger.error(f"Touch data processing error: {e}")
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
