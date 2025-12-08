"""
Revo1 dexterous hand basic control example

This example demonstrates how to control Revo1 dexterous hand devices, including:
- Device information acquisition and serial port configuration
- Voltage and force level monitoring
- Finger position control and action execution
- Motor status monitoring and data reading
- Speed control and current control example
"""

import asyncio
import sys
from revo1_utils import *

async def main():
    """
    Main function: Initialize connection to the dexterous hand and perform basic control operations
    """
    # Detect baud rate and device ID of the dexterous hand, initialize client
    (client, slave_id) = await open_modbus_revo1()

    # Get serial port configuration information
    logger.debug("get_serialport_cfg")  # Get serial port configuration, baud rate
    baudrate = await client.get_serialport_baudrate(slave_id)
    logger.info(f"Baudrate: {baudrate}")

    # Get device information and check device type
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)

    # Touch hand uses current control interface
    # Non-touch hand uses force level configuration
    if not device_info.is_revo1_touch():
        logger.debug("get_force_level")  # Get force level: large-medium-small
        force_level = await client.get_force_level(slave_id)
        logger.info(f"Force level: {force_level}")

    # Get device voltage status
    logger.debug("get_voltage")  # Get battery voltage
    voltage = await client.get_voltage(slave_id)
    logger.info(f"Voltage: {voltage:.1f} mV")

    # Speed control example (commented out)
    # await client.set_finger_speeds(slave_id, [100] * 6)  # Set finger speed, speed loop, fingers closed
    # await client.set_finger_speeds(slave_id, [-100] * 6)  # Set finger speed, speed loop, fingers open

    # Execute finger movement sequence
    await execute_finger_movements(client, slave_id)

    # Get and display motor status information
    await get_and_display_motor_status(client, slave_id)

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def execute_finger_movements(client, slave_id):
    """
    Execute finger movement sequence: set angle -> grip -> open

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Set finger positions using physical angles
    angles = [200] * 6  # First generation hand angle range [550, 900, 700, 700, 700, 700]
    await client.set_finger_positions(slave_id, convert_to_position(angles))
    await asyncio.sleep(1.0)

    # Execute grip action (set all fingers to maximum position value)
    await client.set_finger_positions(slave_id, [600, 600, 1000, 1000, 1000, 1000])

    # Wait for fingers to reach target position
    await asyncio.sleep(1.0)

    # Execute open action (set all fingers to minimum position value)
    await client.set_finger_positions(slave_id, [0] * 6)
    await asyncio.sleep(1.5)


async def get_and_display_motor_status(client, slave_id):
    """
    Get and display motor status information

    Includes:
    - Finger positions (0~100 range and physical angles)
    - Motor currents (original values and mA units)
    - Motor running status

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    logger.debug("get_motor_status")  # Get motor status: position, current, motor status
    status: libstark.MotorStatusData = await client.get_motor_status(slave_id)

    # Display position information
    logger.info(f"positions(0~100): {list(status.positions)}")
    logger.info(f"angles(degree): {convert_to_angle(list(status.positions))}")  # Physical angle

    # Display current information
    logger.info(f"currents: {list(status.currents)}")
    logger.info(f"currents(mA): {convert_to_mA(list(status.currents))}")  # Convert to mA units

    # Display motor status
    logger.info(f"states: {list(status.states)}")
    logger.debug(f"motor status: {status.description}")


if __name__ == "__main__":
    asyncio.run(main())
