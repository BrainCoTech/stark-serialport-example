"""
Revo2 dexterous hand multi-device control example

This example demonstrates how to control multiple Revo2 dexterous hand devices simultaneously, including:
- Multiple device connection methods (single serial port with multiple devices vs multiple serial ports with multiple devices)
- tactile sensor configuration and data acquisition (for touch hands)
- Motor status monitoring and automatic control
- Finger position and current control
- Concurrent task management and exception handling

Connection method explanation:
- Method 1: Connect multiple devices using a single serial port (requires configuring different device IDs)
- Method 2: Connect different devices using multiple serial ports (can use the same device ID)

Revo2 device ID explanation:
- Default left hand ID: 0x7e (126)
- Default right hand ID: 0x7f (127)
- Device ID can be changed via configuration

Applicable scenarios:
- Dual-hand control systems
- Multi-device collaborative operation
- Device performance comparison testing
- Automated production line control
"""

import asyncio
import sys
import time
from revo2_utils import *
from utils import setup_shutdown_event

# Multi-device control configuration
# Method 1: Connect multiple devices using a single serial port (requires configuring different device IDs)
# Method 2: Connect different devices using multiple serial ports (can use the same device ID)
# For example:
# client_left_hand = await libstark.modbus_open(port_name_left, baudrate)
# client_right_hand = await libstark.modbus_open(port_name_right, baudrate)
# Device ID list configuration
# slave_ids = [0x7e]        # Connect one device using a single serial port (left hand)
slave_ids = [0x7e, 0x7f]   # Connect two devices using a single serial port, left hand ID is 0x7e, right hand ID is 0x7f
# Device touch hand marker dictionary
# Used to record whether each device is a touch hand, so that it can be distinguished later
slave_is_touch = {0x7e: False, 0x7f: False}


async def main():
    """
    Main function: initialize device connection and control task
    """
    # Set shutdown event listener
    shutdown_event = setup_shutdown_event(logger)

    # Open Modbus connection
    (client, detected_slave_id) = await open_modbus_revo2()

    # Configure tactile sensor for each device and start status monitoring task
    for slave_id in slave_ids:
        await setup_touch_sensor(client, slave_id)
        asyncio.create_task(get_motor_status_periodically(client, slave_id))
    logger.info("Status monitoring tasks started")

    # Multiple serial ports connect different devices example (optional)
    # Suitable for scenarios where completely independent control of two devices is required
    # client_left_hand = await libstark.modbus_open("/dev/ttyUSB0", libstark.Baudrate.Baud460800)
    # client_right_hand = await libstark.modbus_open("/dev/ttyUSB1", libstark.Baudrate.Baud460800)
    # left_slave_id = 0x7e
    # right_slave_id = 0x7f
    # await setup_touch_sensor(client_left_hand, left_slave_id)
    # await setup_touch_sensor(client_right_hand, right_slave_id)
    # asyncio.create_task(get_motor_status_periodically(client_left_hand, left_slave_id))
    # asyncio.create_task(get_motor_status_periodically(client_right_hand, right_slave_id))

    # Wait for shutdown event
    await shutdown_event.wait()

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def setup_touch_sensor(client, slave_id):
    """
    Configure tactile sensor

    Configure the tactile sensor according to the device type (touch hand/basic version).
    The touch hand device will enable the tactile sensor function, and the basic version device will skip this step.

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Verify device type
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device {slave_id:02x} info: {device_info.description}")
    uses_revo2_touch = device_info.uses_revo2_touch_api()
    slave_is_touch[slave_id] = uses_revo2_touch

    if not uses_revo2_touch:
        logger.warning(f"Device {slave_id:02x} is not Revo2 Touch hardware, skipping tactile sensor setup")
        return

    # Enable tactile sensor function (disabled by default on startup)
    bits = 0x1F  # 0x1f: Enable all tactile sensors on 5 fingers
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1.0)  # Wait for tactile sensor to be ready

    # Verify sensor enabled status
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Device {slave_id:02x} Touch Sensor Enabled: {(bits & 0x1F):05b}")

    # Get tactile sensor firmware version (can only be obtained after setup)
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Device {slave_id:02x} Touch Fw Versions: {touch_fw_versions}")


async def get_touch_status(client, slave_id):
    """
    Get tactile sensor status and data

    Tactile sensor data说明：
    - Normal force: Force perpendicular to the contact surface (pressure)
    - Tangential force: Force along the tangential direction of the contact surface (friction, inertia, etc.)
    - Proximity: Capacitance change value, reflecting the degree of object proximity
    - Sensor status: 0=normal, 1=data error, 2=communication error

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Get all finger tactile sensor status
    touch_status: list[libstark.TouchFingerItem] = await client.get_touch_sensor_status(slave_id)
    thumb: libstark.TouchFingerItem = touch_status[0]   # Thumb
    index: libstark.TouchFingerItem = touch_status[1]   # Index finger
    middle: libstark.TouchFingerItem = touch_status[2]  # Middle finger
    ring: libstark.TouchFingerItem = touch_status[3]    # Ring finger
    pinky: libstark.TouchFingerItem = touch_status[4]   # Pinky finger

    # Record index finger sensor detailed data
    logger.debug(f"Device {slave_id:02x} Index Sensor Desc: {index.description}")
    logger.info(f"Device {slave_id:02x} Index Sensor status: {index.status}")                           # Tactile sensor status
    logger.info(f"Device {slave_id:02x} Index normal_force: {index.normal_force1}")                     # Normal force
    logger.info(f"Device {slave_id:02x} Index tangential_force: {index.tangential_force1}")             # Tangential force
    logger.info(f"Device {slave_id:02x} Index tangential_direction: {index.tangential_direction1}")     # Tangential force direction
    logger.info(f"Device {slave_id:02x} Index proximity: {index.self_proximity1}")                      # Proximity

    # Get single finger tactile sensor status (another way to get)
    thumb = await client.get_single_touch_sensor_status(slave_id, 0)
    index = await client.get_single_touch_sensor_status(slave_id, 1)
    middle = await client.get_single_touch_sensor_status(slave_id, 2)
    ring = await client.get_single_touch_sensor_status(slave_id, 3)
    pinky = await client.get_single_touch_sensor_status(slave_id, 4)

    # Record thumb sensor detailed data
    logger.debug(f"Device {slave_id:02x} Thumb Sensor Desc: {thumb.description}")
    logger.info(f"Device {slave_id:02x} Thumb Sensor status: {thumb.status}")                           # Tactile sensor status
    logger.info(f"Device {slave_id:02x} Thumb normal_force: {thumb.normal_force1}")                     # Normal force
    logger.info(f"Device {slave_id:02x} Thumb tangential_force: {thumb.tangential_force1}")             # Tangential force
    logger.info(f"Device {slave_id:02x} Thumb tangential_direction: {thumb.tangential_direction1}")     # Tangential force direction
    logger.info(f"Device {slave_id:02x} Thumb proximity: {thumb.self_proximity1}")                      # Proximity

    # Get sensor raw channel value (optional, for debugging and advanced analysis)
    # touch_raw_data = await client.get_touch_sensor_raw_data(slave_id)
    # logger.info(f"Device {slave_id:02x} Touch Sensor Raw Data: {touch_raw_data.description}")
    # logger.debug(f"Device {slave_id:02x} Touch Sensor Raw Data Thumb: {touch_raw_data.thumb}")
    # logger.debug(f"Device {slave_id:02x} Touch Sensor Raw Data Index: {touch_raw_data.index}")
    # logger.debug(f"Device {slave_id:02x} Touch Sensor Raw Data Middle: {touch_raw_data.middle}")
    # logger.debug(f"Device {slave_id:02x} Touch Sensor Raw Data Ring: {touch_raw_data.ring}")
    # logger.debug(f"Device {slave_id:02x} Touch Sensor Raw Data Pinky: {touch_raw_data.pinky}")


async def get_motor_status_periodically(client, slave_id):
    """
    Periodically get motor status and execute automatic control

    This function will continuously monitor the motor status and execute simple open/close actions based on finger positions:
    - When fingers are open, execute grip action
    - When fingers are closed, execute open action
    - For touch hand devices, also get tactile sensor data

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    logger.info(f"Motor status monitoring started for device {slave_id:02x}")
    index = 0

    while True:
        try:
            # Check if it is a touch hand, if so, get tactile sensor status
            if slave_is_touch.get(slave_id, False):
                logger.debug(f"Getting tactile sensor status for device {slave_id:02x}...")
                await get_touch_status(client, slave_id)

            # Get motor status
            logger.debug(f"Getting motor status for device {slave_id:02x}...")
            start = time.perf_counter()
            status: libstark.MotorStatusData = await client.get_motor_status(slave_id)
            cost_ms = (time.perf_counter() - start) * 1000

            # Record status information
            logger.info(
                f"Device {slave_id:02x} [{index}] Motor status - cost: {cost_ms:.2f}ms, "
                f"is_idle: {status.is_idle()}"
            )
            logger.info(f"Device {slave_id:02x} [{index}] Motor status: {status.description}")
            index += 1

            # Execute automatic control based on current status
            if status.is_idle():
                if is_positions_open(status):
                    # When fingers are open, execute grip action
                    # Position values: [thumb, index, middle, ring, pinky, wrist]
                    await client.set_finger_positions(slave_id, [500, 500, 1000, 1000, 1000, 1000])
                    logger.debug(f"Device {slave_id:02x} executing grip action")
                elif is_positions_closed(status):
                    # When fingers are closed, execute open action
                    await client.set_finger_positions(slave_id, [400, 400, 0, 0, 0, 0])
                    logger.debug(f"Device {slave_id:02x} executing open action")

            # Add delay to avoid too frequent queries
            await asyncio.sleep(0.001)

        except Exception as e:
            logger.error(f"Error in motor status monitoring for device {slave_id:02x}: {e}")
            await asyncio.sleep(1)  # Wait longer before retrying on error


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
