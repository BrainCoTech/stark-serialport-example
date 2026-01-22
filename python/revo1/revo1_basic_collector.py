"""
Revo1 Basic Version - High-Performance Data Collector Example with Trajectory Control

This example demonstrates high-performance motor data collection using shared buffers
while simultaneously running trajectory control:
- Motor status data collection at 100Hz
- Trajectory control at 100Hz
- Efficient data collection without GIL overhead
- Observe motor response to trajectory commands in real-time

Hardware Support:
- ✅ Revo1 Basic
- ✅ Revo1 Touch

Usage:
    python revo1_basic_collector.py
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
    """Main function: High-performance motor data collection"""

    # Setup shutdown event listener
    shutdown_event = setup_shutdown_event(logger)

    # Open Modbus connection
    logger.info("Connecting to device...")
    (client, detected_slave_id) = await open_modbus_revo1()
    slave_id = detected_slave_id

    # Get device info
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    # ========================================================================
    # Create shared buffer
    # ========================================================================

    logger.info("Creating shared buffer...")
    motor_buffer = libstark.MotorStatusBuffer(max_size=1000)
    logger.info("Shared buffer created")

    # ========================================================================
    # Create and start data collector - Basic mode
    # ========================================================================

    logger.info("Creating data collector (Basic mode)...")

    # Collection frequency (platform-dependent)
    # Linux: 100Hz (high-performance serial port)
    # Windows/macOS: 20Hz (conservative for compatibility)
    import platform
    is_linux = platform.system() == 'Linux'
    motor_frequency = 100 if is_linux else 20
    
    logger.info(f"Platform: {platform.system()}, Motor: {motor_frequency}Hz")

    collector = libstark.DataCollector.new_basic(
        ctx=client,
        motor_buffer=motor_buffer,
        slave_id=slave_id,
        motor_frequency=motor_frequency,
        enable_stats=True,
    )

    logger.info("Data collector created")

    # ========================================================================
    # Start data collection
    # ========================================================================

    logger.info("Starting data collector...")
    collector.start()
    logger.info("Data collector started (background thread)")

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
    # Start data processing task
    # ========================================================================

    # Start data processing task
    asyncio.create_task(process_motor_data(motor_buffer, index_ref if ENABLE_CONTROL else None))

    logger.info("Data processing task started")
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


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
