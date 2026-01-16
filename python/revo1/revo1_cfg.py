"""
Revo1 Dexterous Hand Configuration Management Example

This example demonstrates how to manage various configuration parameters of the Revo1 dexterous hand, including:
- Serial port configuration (slave ID, baud rate)
- Force level settings (large, medium, small)
- Turbo mode configuration and management
- Position calibration settings and execution
- Device reboot and reconnection

Important notes:
- Device will automatically reboot after modifying slave ID or baud rate
- Need to reconnect device with new configuration parameters after reboot
- Touch hand recommended to use current control directly, standard version uses force level control
- Position calibration will affect finger reference position
"""

import asyncio
import sys
from revo1_utils import *


async def set_slave_id(client, slave_id, new_slave_id):
    """
    Modify device slave ID

    Device will automatically reboot after modification, need to reconnect with new slave ID.

    Args:
        client: Modbus client instance
        slave_id: Current slave ID
        new_slave_id: New slave ID (range: 1-247)
    """
    await client.set_slave_id(slave_id, new_slave_id)
    logger.info(f"Slave ID set to {new_slave_id}, device will reboot in 3 seconds")

    # Close current connection
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    logger.info(f"New Slave ID: {new_slave_id}, please reconnect the device with new Slave ID")
    sys.exit(0)


async def set_baudrate(client, slave_id, new_baudrate):
    """
    Modify device baud rate

    Device will automatically reboot after modification, need to reconnect with new baud rate.

    Args:
        client: Modbus client instance
        slave_id: Device slave ID
        new_baudrate: New baud rate (see libstark.Baudrate enum for supported baud rates)
    """
    # Get current baud rate
    baudrate = await client.get_serialport_baudrate(slave_id)
    logger.info(f"Current Baudrate: {baudrate}")

    # Set new baud rate
    await client.set_serialport_baudrate(slave_id, new_baudrate)
    logger.info(f"Baudrate set to {new_baudrate}, device will reboot in 3 seconds")

    # Close current connection
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    logger.info(f"New Baudrate: {new_baudrate}, please reconnect the device with new Baudrate")
    sys.exit(0)


async def configure_force_level(client, slave_id):
    """
    Configure force level

    Touch hand can use current control directly, standard version uses this function to set force level.

    Args:
        client: Modbus client instance
        slave_id: Device slave ID
    """
    # Set force level (optional)
    # await client.set_force_level(slave_id, force_level=libstark.ForceLevel.Full)    # Maximum force
    # await client.set_force_level(slave_id, force_level=libstark.ForceLevel.Normal)  # Normal force
    # await client.set_force_level(slave_id, force_level=libstark.ForceLevel.Small)   # Small force

    # Get and display current force level
    level = await client.get_force_level(slave_id)
    logger.info(f"Current Force level: {level}")


async def configure_turbo_mode(client, slave_id):
    """
    Configure Turbo mode

    Turbo mode is based on stall function. After stalling, the motor stops moving for a period of time (stall time),
    then continues moving for a period of time (continue motion time), periodically executing stop and continue motion.

    Args:
        client: Modbus client instance
        slave_id: Device slave ID
    """
    # Set Turbo mode switch (optional)
    # await client.set_turbo_mode_enabled(slave_id, True)   # Enable Turbo mode
    # await client.set_turbo_mode_enabled(slave_id, False)  # Disable Turbo mode

    # Get and display Turbo mode status
    turbo_mode = await client.get_turbo_mode_enabled(slave_id)
    if turbo_mode:
        logger.info("Turbo mode: enabled")
    else:
        logger.info("Turbo mode: disabled")

    # Configure Turbo parameters (optional)
    # logger.debug("set_turbo_conf")
    # turbo_interval = 2000  # Turbo interval time (milliseconds)
    # turbo_duration = 3000  # Turbo duration time (milliseconds)
    # turbo_conf = libstark.TurboConfig(turbo_interval, turbo_duration)
    # await client.set_turbo_config(slave_id, turbo_conf)
    #
    # # Get and display Turbo configuration
    # turbo_conf = await client.get_turbo_config(slave_id)
    # logger.info(f"Turbo conf: {turbo_conf.description}")


async def configure_position_calibration(client, slave_id):
    """
    Configure position calibration

    Position calibration is used to set finger reference position, ensuring position control accuracy.

    Args:
        client: Modbus client instance
        slave_id: Device slave ID
    """
    # Set whether to automatically execute position calibration after power-on (optional)
    # Note: Device will automatically reboot after modifying this setting
    # await client.set_auto_calibration(slave_id, False)

    # Get and display auto calibration status
    # calibration_enabled = await client.get_auto_calibration_enabled(slave_id)
    # logger.info(f"Auto calibration enabled: {calibration_enabled}")

    # Manually execute position calibration (optional)
    # Recommended to move fingers to appropriate position before executing calibration
    # await client.set_finger_positions(slave_id, [600, 600, 1000, 1000, 1000, 1000])  # Grip action
    # await client.calibrate_position(slave_id)  # Execute position calibration
    # logger.info("Position calibration completed")


async def main():
    """
    Main function: Device configuration management
    """
    # Connect to Revo1 device
    (client, slave_id) = await open_modbus_revo1()

    # Get and display serial port configuration information
    logger.debug("get_serialport_cfg")
    serialport_cfg = await client.get_serialport_cfg(slave_id)
    logger.info(f"Serial Port Config: {serialport_cfg.description}")

    # If only need to view current configuration, can exit here
    # exit(0)

    # Choose one of the following configuration operations as needed:

    # 1. Modify slave ID (uncomment to use)
    # logger.debug("set_slave_id")
    # await set_slave_id(client, slave_id, new_slave_id=1)  # Modify slave_id to 1
    # await set_slave_id(client, slave_id, new_slave_id=2)  # Modify slave_id to 2
    # return  # Will exit program after modification

    # 2. Modify baud rate (uncomment to use)
    # logger.debug("set_baudrate")
    # await set_baudrate(client, slave_id, new_baudrate=libstark.Baudrate.Baud115200)  # Modify baud rate to 115200
    # await set_baudrate(client, slave_id, new_baudrate=libstark.Baudrate.Baud460800)  # Modify baud rate to 460800
    # return  # Will exit program after modification

    # 3. Configure force level (only for non-touch hand)
    await configure_force_level(client, slave_id)

    # 4. Configure Turbo mode
    await configure_turbo_mode(client, slave_id)

    # 5. Configure position calibration
    await configure_position_calibration(client, slave_id)

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
