"""
Revo2 Dexterous Hand Configuration Management Example

This example demonstrates how to manage various configuration parameters of the Revo2 dexterous hand, including:
- Serial port configuration (slave ID, baud rate)
- Device state control (LED, buzzer, vibration)
- Turbo mode configuration and management
- Position calibration settings and execution
- Device reboot and reconnection

Important notes:
- Device will automatically reboot after modifying slave ID or baud rate
- Need to reconnect device with new configuration parameters after reboot
- Position calibration will affect finger reference position
- Please confirm current device state before modifying configuration
"""

import asyncio
import sys
from revo2_utils import *


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


async def configure_device_features(client: libstark.PyDeviceContext, slave_id: int):
    """
    Configure device feature settings

    Args:
        client: Modbus client instance
        slave_id: Device slave ID
    """
    # Set LED, buzzer, vibration functions (optional)
    # await client.set_led_enabled(slave_id, True)        # Enable LED
    # await client.set_buzzer_enabled(slave_id, True)     # Enable buzzer
    # await client.set_vibration_enabled(slave_id, True)  # Enable vibration

    # Get and display device feature status
    led_enabled: bool = await client.get_led_enabled(slave_id)
    logger.info(f"LED Enabled: {led_enabled}")

    buzzer_enabled: bool = await client.get_buzzer_enabled(slave_id)
    logger.info(f"Buzzer Enabled: {buzzer_enabled}")

    vibration_enabled: bool = await client.get_vibration_enabled(slave_id)
    logger.info(f"Vibration Enabled: {vibration_enabled}")


async def configure_turbo_mode(client: libstark.PyDeviceContext, slave_id: int):
    """
    Configure Turbo mode

    Turbo mode can improve device response speed and performance.

    Args:
        client: Modbus client instance
        slave_id: Device slave ID
    """
    # Set Turbo mode switch (optional)
    # await client.set_turbo_mode_enabled(slave_id, True)   # Enable Turbo mode
    # await client.set_turbo_mode_enabled(slave_id, False)  # Disable Turbo mode

    # Get and display Turbo mode status
    turbo_mode_enabled: bool = await client.get_turbo_mode_enabled(slave_id)
    if turbo_mode_enabled:
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
    # turbo_conf: libstark.TurboConfig = await client.get_turbo_config(slave_id)
    # logger.info(f"Turbo conf: {turbo_conf.description}")


async def configure_position_calibration(client: libstark.PyDeviceContext, slave_id: int):
    """
    Configure position calibration

    Position calibration is used to set finger reference position, ensuring position control accuracy.

    Args:
        client: Modbus client instance
        slave_id: Device slave ID
    """
    # Set whether to automatically execute position calibration after power-on (optional)
    # Note: Device will automatically reboot after modifying this setting
    await client.set_auto_calibration(slave_id, True)

    # Get and display auto calibration status
    # auto_calibration_enabled: bool = await client.get_auto_calibration_enabled(slave_id)
    # logger.info(f"Auto calibration enabled: {auto_calibration_enabled}")

    # Manually execute position calibration (optional)
    # Recommended to move fingers to appropriate position before executing calibration
    # await client.set_finger_positions(slave_id, [60, 60, 100, 100, 100, 100])  # Grip action
    # await client.calibrate_position(slave_id)  # Execute position calibration
    # logger.info("Position calibration completed")


async def main():
    """
    Main function: Device configuration management
    """
    # Connect to Revo2 device
    (client, slave_id) = await open_modbus_revo2()

    # Get and display serial port configuration information
    logger.debug("get_serialport_cfg")
    serialport_cfg: libstark.SerialPortCfg = await client.get_serialport_cfg(slave_id)
    logger.info(f"Serial Port Config: {serialport_cfg.description}")

    # Choose one of the following configuration operations as needed:

    # 1. Modify slave ID (uncomment to use)
    # logger.debug("set_slave_id")
    # await set_slave_id(client, slave_id, new_slave_id=0x7e)  # Modify slave_id to 0x7e
    # sys.exit(0) # Will exit program after modification

    # 2. Modify baud rate (uncomment to use)
    # logger.debug("set_baudrate")
    # # Supported baud rates: Baud5Mbps, Baud2Mbps, Baud1Mbps, Baud460800, etc.
    # await set_baudrate(client, slave_id, new_baudrate=libstark.Baudrate.Baud2Mbps)
    sys.exit(0)  # Will exit program after modification

    # 3. Configure device feature settings
    await configure_device_features(client, slave_id)

    # 4. Configure Turbo mode (uncomment to use)
    await configure_turbo_mode(client, slave_id)

    # 5. Configure position calibration (uncomment to use)
    await configure_position_calibration(client, slave_id)

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
