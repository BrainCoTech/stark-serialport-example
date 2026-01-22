"""
Revo2 Touch Version Dexterous Hand Control Example

This example demonstrates how to control the Revo2 Touch Version (pressure sensor) dexterous hand device, including:
- Configuration and enabling of the tactile sensors
- Acquiring pressure sensing data from five fingers and the palm

Notes:
- This example is only applicable to Revo2 Touch Version (pressure sensor) hardware
- Tactile sensors are enabled by default at startup
- Do not apply force to the sensor when performing zero-drift calibration
"""

import asyncio
import sys
from revo2_utils import *


async def main():
    """
    Main function: initialize the Revo2 Touch Version (pressure sensor) dexterous hand device and execute the tactile sensor control
    """
    # Connect Revo2 device
    (client, slave_id) = await open_modbus_revo2()

    # Verify that the device type is Revo2 Touch Version (pressure sensor)
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    if not device_info.is_revo2_touch():
        logger.error("This example is only for Revo2 Touch hardware")
        sys.exit(1)

    if not client.is_touch_pressure():
        logger.error("This example is only for Revo2 Touch Pressure hardware")
        libstark.modbus_close(client)
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
    # bits = 0x3F  # 0x3F: Enable five fingers + palm
    # await client.touch_sensor_setup(slave_id, bits)
    # await asyncio.sleep(1)  # Wait for tactile sensors to be ready

    # Verify sensor enabled status
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x3F):06b}")

    # Get tactile sensor firmware version (can only be obtained after enabling)
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    # await client.get_modulus_touch_data_type(slave_id, libstark.ModulusTouchDataType.Calibrated)
    data_tpye = await client.get_modulus_touch_data_type(slave_id)
    logger.info(f"Modulus Touch Data Type: {data_tpye}")


async def monitor_touch_sensors(client, slave_id):
    """
    Monitor tactile sensor data

    Get tactile sensor five fingers + palm合力数据
    Get tactile sensor five fingers + palm all sampling points data

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    while True:
        # Get single finger tactile sensor status (optional way)
        thumb = await client.get_single_modulus_touch_summary(slave_id, 0)
        pinky = await client.get_single_modulus_touch_summary(slave_id, 4)
        palm = await client.get_single_modulus_touch_summary(slave_id, 5)
        logger.info(f"Thumb: {thumb}")
        logger.info(f"Pinky: {pinky}")
        logger.info(f"Palm: {palm}")

        # Get all finger tactile sensor status
        touch_summary: list[int] = await client.get_modulus_touch_summary(slave_id)
        logger.info(f"Touch summary: {touch_summary}")
        touch_data: list[int] = await client.get_modulus_touch_data(slave_id)
        logger.info(f"Touch Data: {touch_data}")

        await asyncio.sleep(0.05)  # type: ignore  # Control data acquisition frequency
        # break  # Only execute once in the example, adjust as needed in actual applications


async def perform_sensor_maintenance(client, slave_id):
    """
    Perform sensor maintenance operations (optional)

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """

    # Zero drift calibration
    # When the value in idle state is not 0, it can be calibrated through this instruction
    # It is recommended to ignore the data within ten seconds after calibration, and the finger sensor cannot be applied during execution
    # await client.touch_sensor_calibrate(slave_id, 0x3f)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
