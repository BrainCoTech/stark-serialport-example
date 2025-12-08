"""
Revo2 Touch Version Dexterous Hand Control Example

This example demonstrates how to control the Revo2 Touch Version (capacitive sensor) dexterous hand device, including:
- Configuration and enabling of tactile sensors
- Acquisition and processing of 3D force data (normal force, tangential force)
- Monitoring proximity values (capacitance changes)
- Reading data from individual finger sensors
- Sensor status monitoring and exception handling
- Sensor calibration and maintenance operations

Notes:
- This example is only applicable to Revo2 Touch Version (capacitive sensor) hardware
- Tactile sensors are enabled by default on startup
- Do not apply force to the sensor during zero-drift calibration
"""

import asyncio
import sys
from revo2_utils import *


async def main():
    """
    Main function: initialize the Revo2 Touch Device (capacitive sensor) dexterous hand device and execute the tactile sensor control
    """
    # Connect Revo2 device
    (client, slave_id) = await open_modbus_revo2()  # type: ignore

    # Verify that the device type is Revo2 Touch Device (capacitive sensor)
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    if not device_info.is_revo2_touch():
        logger.error("This example is only for Revo2 Touch hardware")
        sys.exit(1)

    # Configure tactile sensors
    await setup_touch_sensors(client, slave_id)

    # Monitor tactile sensor data
    await monitor_touch_sensors(client, slave_id)

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def setup_touch_sensors(client, slave_id):
    """
    Configure and enable tactile sensors

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Enable tactile sensor function (disabled by default on startup)
    bits = 0x1F  # 0x1F: Enable all tactile sensors on five fingers
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)  # Wait for tactile sensors to be ready

    # Verify sensor enabled status
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x1F):05b}")

    # Get tactile sensor firmware version (can only be obtained after enabling)
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")


async def monitor_touch_sensors(client, slave_id):
    """
    Monitor tactile sensor data

    Get tactile sensor status, normal force, tangential force, channel value (capacitance)

    Sensor data description:
    - Normal force: Force perpendicular to the contact surface, can be simply understood as pressure
    - Tangential force: Any force along the tangential direction of the contact surface, such as inertial force, elastic force, friction force
    - Proximity value: Capacitance change value
    - Sensor status: 0=normal, 1=data exception, 2=sensor communication exception

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    while True:
        # Get all finger tactile sensor status
        touch_status: list[libstark.TouchFingerItem] = await client.get_touch_sensor_status(slave_id)
        thumb: libstark.TouchFingerItem = touch_status[0]   # Thumb
        index: libstark.TouchFingerItem = touch_status[1]   # Index
        middle: libstark.TouchFingerItem = touch_status[2]  # Middle
        ring: libstark.TouchFingerItem = touch_status[3]    # Ring
        pinky: libstark.TouchFingerItem = touch_status[4]   # Pinky

        # Record index sensor detailed data
        logger.debug(f"Index Sensor Desc: {index.description}")
        logger.info(f"Index Sensor status: {index.status}")                           # Tactile sensor status
        logger.info(f"Index normal_force: {index.normal_force1}")                     # Normal force
        logger.info(f"Index tangential_force: {index.tangential_force1}")             # Tangential force
        logger.info(f"Index tangential_direction: {index.tangential_direction1}")     # Tangential force direction
        logger.info(f"Index proximity: {index.self_proximity1}")                      # Proximity value

        # Get single finger tactile sensor status (optional way)
        thumb = await client.get_single_touch_sensor_status(slave_id, 0)
        index = await client.get_single_touch_sensor_status(slave_id, 1)
        middle = await client.get_single_touch_sensor_status(slave_id, 2)
        ring = await client.get_single_touch_sensor_status(slave_id, 3)
        pinky = await client.get_single_touch_sensor_status(slave_id, 4)

        # Record thumb sensor detailed data
        logger.debug(f"Thumb Sensor Desc: {thumb.description}")
        logger.info(f"Thumb Sensor status: {thumb.status}")
        logger.info(f"Thumb normal_force: {thumb.normal_force1}")                     # Normal force
        logger.info(f"Thumb tangential_force: {thumb.tangential_force1}")             # Tangential force
        logger.info(f"Thumb tangential_direction: {thumb.tangential_direction1}")     # Tangential force direction
        logger.info(f"Thumb proximity: {thumb.self_proximity1}")                      # Proximity value

        # Get sensor raw channel value (optional, for debugging)
        # touch_raw_data = await client.get_touch_sensor_raw_data(slave_id)
        # logger.info(f"Touch Sensor Raw Data: {touch_raw_data.description}")
        # logger.debug(f"Touch Sensor Raw Data Thumb: {touch_raw_data.thumb}")
        # logger.debug(f"Touch Sensor Raw Data Index: {touch_raw_data.index}")
        # logger.debug(f"Touch Sensor Raw Data Middle: {touch_raw_data.middle}")
        # logger.debug(f"Touch Sensor Raw Data Ring: {touch_raw_data.ring}")
        # logger.debug(f"Touch Sensor Raw Data Pinky: {touch_raw_data.pinky}")

        await asyncio.sleep(0.05)  # Control data acquisition frequency
        break  # Only execute once in the example, adjust as needed in actual applications


async def perform_sensor_maintenance(client, slave_id):
    """
    Perform sensor maintenance operations (optional)

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Tactile sensor reset
    # Send sensor acquisition channel reset instruction, do not apply force to the finger sensor during execution
    # await client.touch_sensor_reset(slave_id, 0x1f)  # Reset the sensor acquisition channel of the specified finger

    # Tactile sensor parameter calibration
    # When the 3D force value in the idle state is not zero, you can calibrate using this command.
    # This command takes a long time to execute, and data collected during its execution should not be used as reference.
    # It is recommended to ignore sensor data for ten seconds after calibration; do not apply force to the finger sensors during calibration.
    # await client.touch_sensor_calibrate(slave_id, 0x1f)  # Calibrate the sensor data channel for the specified fingers


if __name__ == "__main__":
    asyncio.run(main())
