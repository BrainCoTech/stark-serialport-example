"""
Revo1 Dexterous Hand Multi-Device Control Example

This example demonstrates how to control multiple Revo1 dexterous hand devices simultaneously, including:
- Multi-device connection methods (single serial port multi-device vs multiple serial ports multi-device)
- tactile sensor configuration and data acquisition (touch hand)
- Motor status monitoring and automatic control
- Finger position and current control
- Concurrent task management and exception handling

Connection method description:
- Method 1: Single serial port connecting multiple devices (requires configuring different device IDs)
- Method 2: Multiple serial ports connecting different devices separately (can use the same device ID)

Applicable scenarios:
- Dual-hand control systems
- Multi-device collaborative operations
- Device performance comparison testing
- Production line automation control
"""

import asyncio
import sys
import time
from revo1_utils import *
from utils import setup_shutdown_event

# Multi-device control configuration
# Method 1: Single serial port connecting multiple devices (requires configuring different device IDs)
# Method 2: Multiple serial ports connecting different devices separately (can use the same device ID)
# Example:
# client_left_hand = await libstark.modbus_open(port_name_left, baudrate)
# client_right_hand = await libstark.modbus_open(port_name_right, baudrate)

# Device ID list configuration
slave_ids = [1]        # Single serial port connecting one device
# slave_ids = [1, 2]   # Single serial port connecting two devices with device IDs 1 and 2 respectively

# Device touch hand flag dictionary
# Used to record whether each device is a touch hand for subsequent processing differentiation
slave_is_touch = {1: False, 2: False}


async def main():
    """
    Main function: Initialize device connections and control tasks
    """
    # Setup shutdown event listener
    shutdown_event = setup_shutdown_event(logger)

    # Open Modbus connection
    (client, detected_slave_id) = await open_modbus_revo1()

    # Configure tactile sensors for each device and start status monitoring tasks
    for slave_id in slave_ids:
        await setup_touch_sensor(client, slave_id)
        asyncio.create_task(get_motor_status_periodically(client, slave_id))
    logger.info("Status monitoring tasks started")

    # Example of multiple serial ports connecting different devices separately (optional)
    # Suitable for scenarios requiring completely independent control of two devices
    # client_left_hand = await libstark.modbus_open("/dev/ttyUSB0", libstark.Baudrate.Baud115200)
    # client_right_hand = await libstark.modbus_open("/dev/ttyUSB1", libstark.Baudrate.Baud115200)
    # slave_id = 1  # Both devices use default ID=1
    # await setup_touch_sensor(client_left_hand, slave_id) # If it's a touch hand
    # await setup_touch_sensor(client_right_hand, slave_id) # If it's a touch hand
    # asyncio.create_task(get_motor_status_periodically(client_left_hand, slave_id))
    # asyncio.create_task(get_motor_status_periodically(client_right_hand, slave_id))

    # Wait for shutdown event
    await shutdown_event.wait()

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def setup_touch_sensor(client, slave_id):
    """
    Configure tactile sensor

    Configure according to device type (touch hand/basic version).
    Touch hand devices will enable tactile sensor functionality, basic version devices will skip this step.

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Verify device type
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device {slave_id} info: {device_info.description}")
    uses_revo1_touch = device_info.uses_revo1_touch_api()
    slave_is_touch[slave_id] = uses_revo1_touch

    if not uses_revo1_touch:
        logger.warning(f"Device {slave_id} is not Revo1 Touch hardware, skipping tactile sensor setup")
        return

    # Enable tactile sensor functionality (disabled by default on startup)
    bits = 0x1F  # 0x1f: Enable all tactile sensors on 5 fingers
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)  # Wait for tactile sensor to be ready

    # Verify sensor enable status
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Device {slave_id} Touch Sensor Enabled: {(bits & 0x1F):05b}")

    # Get tactile sensor firmware version (can only be obtained after setup)
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Device {slave_id} Touch Fw Versions: {touch_fw_versions}")


async def get_touch_status(client, slave_id):
    """
    Get tactile sensor status and data

    tactile sensor data description:
    - Normal force: Force perpendicular to the contact surface (pressure)
    - Tangential force: Force along the tangent direction of the contact surface (friction, inertial force, etc.)
    - Self proximity: Self-capacitance change value, reflecting object proximity
    - Mutual proximity: Mutual capacitance change value, reflecting object proximity
    - Sensor status: 0=normal, 1=data error, 2=communication error

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Get tactile sensor status for all fingers
    touch_status: list[libstark.TouchFingerItem] = await client.get_touch_sensor_status(slave_id)
    thumb = touch_status[0]   # Thumb
    index = touch_status[1]   # Index finger
    middle = touch_status[2]  # Middle finger
    ring = touch_status[3]    # Ring finger
    pinky = touch_status[4]   # Pinky finger

    # Record index finger sensor detailed data
    logger.debug(f"Device {slave_id} Index Sensor Desc: {index.description}")
    logger.info(f"Device {slave_id} Index Sensor status: {index.status}")
    logger.info(f"Device {slave_id} Index normal_force1: {index.normal_force1}")      # Normal force 1
    logger.info(f"Device {slave_id} Index normal_force2: {index.normal_force2}")      # Normal force 2
    logger.info(f"Device {slave_id} Index normal_force3: {index.normal_force3}")      # Normal force 3
    logger.info(f"Device {slave_id} Index tangential_force1: {index.tangential_force1}")         # Tangential force 1
    logger.info(f"Device {slave_id} Index tangential_force2: {index.tangential_force2}")         # Tangential force 2
    logger.info(f"Device {slave_id} Index tangential_force3: {index.tangential_force3}")         # Tangential force 3
    logger.info(f"Device {slave_id} Index tangential_direction1: {index.tangential_direction1}") # Tangential direction 1
    logger.info(f"Device {slave_id} Index tangential_direction2: {index.tangential_direction2}") # Tangential direction 2
    logger.info(f"Device {slave_id} Index tangential_direction3: {index.tangential_direction3}") # Tangential direction 3
    logger.info(f"Device {slave_id} Index self_proximity1: {index.self_proximity1}")             # Self proximity 1
    logger.info(f"Device {slave_id} Index self_proximity2: {index.self_proximity2}")             # Self proximity 2
    logger.info(f"Device {slave_id} Index mutual_proximity: {index.mutual_proximity}")           # Mutual proximity

    # Get tactile sensor status for a single finger (another way to get data)
    thumb = await client.get_single_touch_sensor_status(slave_id, 0)
    index = await client.get_single_touch_sensor_status(slave_id, 1)
    middle = await client.get_single_touch_sensor_status(slave_id, 2)
    ring = await client.get_single_touch_sensor_status(slave_id, 3)
    pinky = await client.get_single_touch_sensor_status(slave_id, 4)

    # Record thumb sensor data
    logger.debug(f"Device {slave_id} Thumb Sensor Desc: {thumb.description}")
    logger.info(f"Device {slave_id} Thumb Sensor status: {thumb.status}")
    logger.info(f"Device {slave_id} Thumb normal_force1: {thumb.normal_force1}")           # Normal force 1
    logger.info(f"Device {slave_id} Thumb normal_force2: {thumb.normal_force2}")           # Normal force 2
    logger.info(f"Device {slave_id} Thumb tangential_force1: {thumb.tangential_force1}")   # Tangential force 1
    logger.info(f"Device {slave_id} Thumb tangential_force2: {thumb.tangential_force2}")   # Tangential force 2
    logger.info(f"Device {slave_id} Thumb tangential_direction1: {thumb.tangential_direction1}") # Tangential direction 1
    logger.info(f"Device {slave_id} Thumb tangential_direction2: {thumb.tangential_direction2}") # Tangential direction 2
    logger.info(f"Device {slave_id} Thumb self_proximity1: {thumb.self_proximity1}")       # Self proximity 1

    # Get sensor raw channel values (optional, for debugging and advanced analysis)
    # touch_raw_data = await client.get_touch_sensor_raw_data(slave_id)
    # logger.info(f"Device {slave_id} Touch Sensor Raw Data: {touch_raw_data.description}")
    # logger.debug(f"Device {slave_id} Touch Sensor Raw Data Thumb: {touch_raw_data.thumb}")
    # logger.debug(f"Device {slave_id} Touch Sensor Raw Data Index: {touch_raw_data.index}")
    # logger.debug(f"Device {slave_id} Touch Sensor Raw Data Middle: {touch_raw_data.middle}")
    # logger.debug(f"Device {slave_id} Touch Sensor Raw Data Ring: {touch_raw_data.ring}")
    # logger.debug(f"Device {slave_id} Touch Sensor Raw Data Pinky: {touch_raw_data.pinky}")


async def get_motor_status_periodically(client, slave_id):
    """
    Periodically get motor status and execute automatic control

    This function continuously monitors motor status and executes simple open/close actions based on finger position:
    - When fingers are opened, execute grip action
    - When fingers are closed, execute open action
    - For touch hand devices, also get tactile sensor data

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    logger.info(f"Motor status monitoring started for device {slave_id}")
    index = 0

    while True:
        try:
            # Check if it's a touch hand, if so get tactile sensor status
            if slave_is_touch.get(slave_id, False):
                logger.debug(f"Getting tactile sensor status for device {slave_id}...")
                await get_touch_status(client, slave_id)

            # Get motor status
            logger.debug(f"Getting motor status for device {slave_id}...")
            start = time.perf_counter()
            status: libstark.MotorStatusData = await client.get_motor_status(slave_id)
            cost_ms = (time.perf_counter() - start) * 1000

            # Record status information
            logger.info(
                f"Device {slave_id} [{index}] Motor status - cost: {cost_ms:.2f}ms, "
                f"is_idle: {status.is_idle()}, "
                f"is_closed: {status.is_closed()}, "
                f"is_opened: {status.is_opened()}"
            )
            logger.info(f"Device {slave_id} [{index}] Motor status: {status.description}")
            index += 1

            # Execute automatic control based on current status
            if status.is_idle():
                if status.is_opened():
                    # When fingers are opened, execute grip action
                    # Position values: [Thumb, Thumb auxiliary, Index, Middle, Ring, Pinky]
                    await client.set_finger_positions(
                        slave_id, [60, 60, 100, 100, 100, 100]
                    )
                    logger.debug(f"Device {slave_id} executing grip action")
                elif status.is_closed():
                    # When fingers are closed, execute open action
                    await client.set_finger_positions(slave_id, [0] * 6)
                    logger.debug(f"Device {slave_id} executing open action")

            # Add delay to avoid overly frequent queries
            await asyncio.sleep(0.001)

        except Exception as e:
            logger.error(f"Error in motor status monitoring for device {slave_id}: {e}")
            await asyncio.sleep(1)  # Wait longer before retrying when an error occurs


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)