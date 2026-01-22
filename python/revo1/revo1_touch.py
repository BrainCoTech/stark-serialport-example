"""
Revo1 Touch Version Dexterous Hand Control Example

This example demonstrates how to control Revo1 touch hand dexterous hand device, including:
- Configuration and enabling of tactile sensors
- Acquisition and processing of tactile sensor data
- Motor status monitoring and automatic control
- Finger position control and current control
"""

import asyncio
import sys
import time
from revo1_utils import *
from utils import setup_shutdown_event

async def main():
    """
    Main function: Initialize touch hand dexterous hand connection and control tasks
    """
    # Set up shutdown event listener
    shutdown_event = setup_shutdown_event(logger)

    # Detect baud rate and device ID of dexterous hand, initialize client
    (client, slave_id) = await open_modbus_revo1()

    # Verify device type is touch hand
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")
    if not device_info.is_revo1_touch():
        logger.error("This example is only for Revo1 Touch hardware")
        sys.exit(1)

    # Configure tactile sensors
    await setup_touch_sensor(client, slave_id)

    # Start motor status monitoring task
    asyncio.create_task(get_motor_status_periodically(client, slave_id))
    logger.info("Status task started")

    # Wait for shutdown event
    await shutdown_event.wait()

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)

async def setup_touch_sensor(client, slave_id):
    """
    Configure and enable tactile sensors

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Enable tactile sensor function (disabled by default on boot)
    bits = 0x1F  # 0x1f: Enable all tactile sensors on 5 fingers
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)  # Wait for tactile sensors to be ready

    # Verify sensor enabled status
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x1F):05b}")

    # Get tactile sensor firmware version (can only be obtained after setup)
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    # tactile sensor maintenance commands (optional)
    # Reset: Send sensor acquisition channel reset command, try to avoid applying force to finger sensors during execution
    # await client.touch_sensor_reset(slave_id, 0x1f)

    # Calibration: Use when 3D force values are not zero in idle state, execution takes longer
    # Recommended to ignore data within ten seconds after calibration, finger sensors must not be under force during execution
    # await client.touch_sensor_calibrate(slave_id, 0x1f)

async def get_touch_status(client, slave_id):
    """
    Get tactile sensor status and data

    tactile sensor data description:
    - Normal force: Force perpendicular to the contact surface (pressure)
    - Tangential force: Force along the contact surface tangent direction (friction force, inertia force, etc.)
    - Self-proximity: Self-capacitance change value
    - Mutual-proximity: Mutual-capacitance change value
    - Sensor status: 0=normal, 1=data abnormal, 2=communication abnormal

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Get the tactile sensor status of all fingers
    touch_status: list[libstark.TouchFingerItem] = await client.get_touch_sensor_status(slave_id)
    thumb = touch_status[0]   # Thumb
    index = touch_status[1]   # Index
    middle = touch_status[2]  # Middle
    ring = touch_status[3]    # Ring
    pinky = touch_status[4]   # Pinky

    # Record index sensor detailed data
    logger.debug(f"Index Sensor Desc: {index.description}")
    logger.info(f"Index Sensor status: {index.status}")  # Touch sensor status
    logger.info(f"Index normal_force1: {index.normal_force1}")  # 法向力1
    logger.info(f"Index normal_force2: {index.normal_force2}")  # Normal force 2
    logger.info(f"Index normal_force3: {index.normal_force3}")  # Normal force 3
    logger.info(f"Index tangential_force1: {index.tangential_force1}")  # Tangential force 1
    logger.info(f"Index tangential_force2: {index.tangential_force2}")  # Tangential force 2
    logger.info(f"Index tangential_force3: {index.tangential_force3}")  # Tangential force 3
    logger.info(f"Index tangential_direction1: {index.tangential_direction1}")  # Tangential direction 1
    logger.info(f"Index tangential_direction2: {index.tangential_direction2}")  # Tangential direction 2
    logger.info(f"Index tangential_direction3: {index.tangential_direction3}")  # Tangential direction 3
    logger.info(f"Index self_proximity1: {index.self_proximity1}")  # Self proximity 1
    logger.info(f"Index self_proximity2: {index.self_proximity2}")  # Self proximity 2
    logger.info(f"Index mutual_proximity: {index.mutual_proximity}")  # Mutual proximity

    # Get single finger touch sensor status (another way)
    thumb = await client.get_single_touch_sensor_status(slave_id, 0)
    index = await client.get_single_touch_sensor_status(slave_id, 1)
    middle = await client.get_single_touch_sensor_status(slave_id, 2)
    ring = await client.get_single_touch_sensor_status(slave_id, 3)
    pinky = await client.get_single_touch_sensor_status(slave_id, 4)

    # Record thumb sensor data
    logger.debug(f"Thumb Sensor Desc: {thumb.description}")
    logger.info(f"Thumb Sensor status: {thumb.status}")
    logger.info(f"Thumb normal_force1: {thumb.normal_force1}")  # Normal force 1
    logger.info(f"Thumb normal_force2: {thumb.normal_force2}")  # Normal force 2
    logger.info(f"Thumb tangential_force1: {thumb.tangential_force1}")  # Tangential force 1
    logger.info(f"Thumb tangential_force2: {thumb.tangential_force2}")  # Tangential force 2
    logger.info(f"Thumb tangential_direction1: {thumb.tangential_direction1}")  # Tangential direction 1
    logger.info(f"Thumb tangential_direction2: {thumb.tangential_direction2}")  # Tangential direction 2
    logger.info(f"Thumb self_proximity1: {thumb.self_proximity1}")  # Self proximity 1

    # Get sensor raw channel value (optional, for debugging)
    # touch_raw_data = await client.get_touch_sensor_raw_data(slave_id)
    # logger.info(f"Touch Sensor Raw Data: {touch_raw_data.description}")
    # logger.debug(f"Touch Sensor Raw Data Thumb: {touch_raw_data.thumb}")
    # logger.debug(f"Touch Sensor Raw Data Index: {touch_raw_data.index}")
    # logger.debug(f"Touch Sensor Raw Data Middle: {touch_raw_data.middle}")
    # logger.debug(f"Touch Sensor Raw Data Ring: {touch_raw_data.ring}")
    # logger.debug(f"Touch Sensor Raw Data Pinky: {touch_raw_data.pinky}")

async def get_motor_status_periodically(client, slave_id):
    """
    Periodically get motor status and execute automatic control

    This function will continuously monitor the motor status and execute simple open and close actions based on finger position:
    - When the finger is opened, execute the handshake action
    - When the finger is closed, execute the open action

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    logger.info("Motor status monitoring started")
    index = 0

    while True:
        try:
            # Get tactile sensor status
            logger.debug("Getting tactile sensor status...")
            await get_touch_status(client, slave_id)

            # Get motor status
            logger.debug("Getting motor status...")
            start = time.perf_counter()
            status: libstark.MotorStatusData = await client.get_motor_status(slave_id)
            cost_ms = (time.perf_counter() - start) * 1000

            # Record status information
            logger.info(
                f"[{index}] Motor status - cost: {cost_ms:.2f}ms, "
                f"is_idle: {status.is_idle()}, "
                f"is_closed: {status.is_closed()}, "
                f"is_opened: {status.is_opened()}"
            )
            logger.info(f"[{index}] Motor status: {status.description}")
            index += 1

            # Execute action based on current status
            if status.is_idle():
                if status.is_opened():
                    # When the finger is opened, execute the handshake action
                    await client.set_finger_positions(
                        slave_id, [60, 60, 100, 100, 100, 100]
                    )
                elif status.is_closed():
                    # When the finger is closed, execute the open action
                    await client.set_finger_positions(slave_id, [0] * 6)

            # Add delay to avoid too frequent queries
            await asyncio.sleep(0.001)

        except Exception as e:
            logger.error(f"Error in motor status monitoring: {e}")
            await asyncio.sleep(1)  # Wait longer before retrying when an error occurs

async def set_finger_current(client, slave_id: int):
    """
    Set finger current control

    Current control parameter range: -100~-20, 20~100
    Positive number represents the closing direction, negative number represents the opening direction

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Select the finger to control
    # finger_id = libstark.FingerId.Thumb     # Thumb
    # finger_id = libstark.FingerId.ThumbAux  # Thumb auxiliary
    # finger_id = libstark.FingerId.Index     # Index
    # finger_id = libstark.FingerId.Middle    # Middle
    finger_id = libstark.FingerId.Ring        # Ring
    # finger_id = libstark.FingerId.Pinky     # Pinky

    # Set single finger current
    await client.set_finger_current(slave_id, finger_id, -20)

    # Or set all fingers current
    # await client.set_finger_currents(slave_id, [-100] * 6)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
