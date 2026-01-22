"""
Revo2 Dexterous Hand Control Example

This example demonstrates how to control the Revo2 dexterous hand device, including:
- Setting finger control modes (normalized mode/physical mode)
- Configuring finger parameters (position, speed, current limits)
- Multiple control methods (speed control, current control, PWM control, position control)
- Control of individual fingers and all fingers
- Getting and monitoring motor status

Important notes:
- Sign of control parameters indicates direction: positive for closing direction, negative for opening direction
- Parameter ranges for each finger can be found in official documentation
- Recommended to wait appropriate time after executing control commands for fingers to reach target position
"""

import asyncio
import sys
from revo2_utils import *


async def main():
    """Main function: Initialize Revo2 dexterous hand and execute control examples"""
    # Connect to Revo2 device
    (client, slave_id) = await open_modbus_revo2(port_name=None) # Replace with actual serial port name, passing None will attempt auto-detection

    # Directly specify device ID, serial port, and baud rate
    # slave_id = 0x7e
    # client: libstark.PyDeviceContext = await libstark.modbus_open(port_name='/dev/ttyUSB0', baudrate=libstark.Baudrate.Baud115200)

    device_info = client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info}")

    # Configure control mode
    await configure_control_mode(client, slave_id)

    # Configure finger parameters (optional)
    await configure_finger_parameters(client, slave_id)

    # Execute control examples
    await execute_control_examples(client, slave_id)

    # Get and display motor status
    await get_and_display_motor_status(client, slave_id)

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def configure_control_mode(client, slave_id):
    """Configure finger control mode

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Set unit mode for finger control parameters
    logger.debug("set_finger_unit_mode")
    await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized)  # Normalized mode (per-thousand)
    # await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Physical)  # Physical mode

    # Get and verify finger control mode
    logger.debug("get_finger_unit_mode")
    finger_unit_mode = await client.get_finger_unit_mode(slave_id)
    logger.info(f"Finger unit mode: {finger_unit_mode}")

    # Reference documentation: https://www.brainco-hz.com/docs/revolimb-hand/revo2/modbus_foundation.html#%E5%8D%95%E4%BD%8D%E6%A8%A1%E5%BC%8F%E8%AE%BE%E7%BD%AE-937


async def configure_finger_parameters(client, slave_id):
    """Configure finger parameters (optional)

    Set parameters such as maximum angle, minimum angle, maximum speed, maximum current, protection current for fingers.
    Parameter ranges for each finger can be found in official documentation.

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    protected_currents = await client.get_finger_protected_currents(slave_id)
    logger.info(f"Protected currents: {protected_currents}")
    # await client.set_finger_protected_currents(slave_id, [600] * 6)
    # protected_currents = await client.get_finger_protected_currents(slave_id)
    # logger.info(f"Protected currents: {protected_currents}")
    # sys.exit(0)

    all_fingers_settings = await client.get_all_finger_settings(slave_id)
    for i in range(6):
        logger.info(f"Finger[{i}] settings: {all_fingers_settings[i].description}")

    finger_id = libstark.FingerId.Index  # Select index finger as example
    index_finger_settings = await client.get_finger_settings(slave_id, finger_id)
    logger.info(f"Finger[{finger_id}] settings: {index_finger_settings.description}")
    # await client.set_finger_settings(slave_id, finger_id, index_finger_settings)

    # Set minimum position (angle)
    # await client.set_finger_min_position(slave_id, finger_id, 0)
    # min_position = await client.get_finger_min_position(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] min position: {min_position}")

    # Set maximum position (angle)
    # await client.set_finger_max_position(slave_id, finger_id, 80)
    # max_position = await client.get_finger_max_position(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max position: {max_position}")

    # Set maximum speed
    # await client.set_finger_max_speed(slave_id, finger_id, 130)
    # max_speed = await client.get_finger_max_speed(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max speed: {max_speed}")

    # Set maximum current
    # await client.set_finger_max_current(slave_id, finger_id, 1000)
    # max_current = await client.get_finger_max_current(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max current: {max_current}")

    # Set protection current
    # await client.set_finger_protected_current(slave_id, finger_id, 500)
    # protected_current = await client.get_finger_protected_current(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] protected current: {protected_current}")


async def execute_control_examples(client, slave_id):
    """Execute examples of various control methods

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    finger_id = libstark.FingerId.Middle  # Select middle finger as example

    # Single finger control examples
    await single_finger_control_examples(client, slave_id, finger_id)

    # All fingers control examples
    await all_fingers_control_examples(client, slave_id)


async def single_finger_control_examples(client, slave_id, finger_id):
    """Single finger control examples

    Args:
        client: Modbus client instance
        slave_id: Device ID
        finger_id: Finger ID
    """
    # Speed control: sign indicates direction, positive for closing, negative for opening
    # await client.set_finger_speed(slave_id, finger_id, 500)  # Range: -1000 ~ 1000
    # await asyncio.sleep(1.0)  # Wait for finger to reach target position

    # Current control: sign indicates direction, positive for closing, negative for opening
    # await client.set_finger_current(slave_id, finger_id, -300)  # Range: -1000 ~ 1000
    # await asyncio.sleep(1.0)  # Wait for finger to reach target position

    # PWM control: sign indicates direction, positive for closing, negative for opening
    # await client.set_finger_pwm(slave_id, finger_id, 700)  # Range: -1000 ~ 1000
    # await asyncio.sleep(1.0)  # Wait for finger to reach target position

    # Position control: target position + expected time
    # await client.set_finger_position_with_millis(slave_id, finger_id, 1000, 1000)
    # await asyncio.sleep(1.0)  # Wait for finger to reach target position

    # Position control: target position + expected speed
    # await client.set_finger_position_with_speed(slave_id, finger_id, 1, 50)
    # await asyncio.sleep(1.0)  # Wait for finger to reach target position


async def all_fingers_control_examples(client, slave_id):
    """All fingers control examples

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # All fingers speed control: sign indicates direction, positive for closing, negative for opening
    # await client.set_finger_speeds(slave_id, [500] * 6)
    # await asyncio.sleep(1.0)  # Wait for fingers to reach target position

    # All fingers current control: sign indicates direction, positive for closing, negative for opening
    # await client.set_finger_currents(slave_id, [-300] * 6)
    # await asyncio.sleep(1.0)  # Wait for fingers to reach target position

    # All fingers PWM control: sign indicates direction, positive for closing, negative for opening
    # await client.set_finger_pwms(slave_id, [700] * 6)
    # await asyncio.sleep(1.0)  # Wait for fingers to reach target position

    # All fingers position control: target position + expected time
    # Position values: [Thumb, Index, Middle, Ring, Pinky, Wrist]
    positions = [500] * 6
    durations = [300] * 6  # Expected time to reach target position (milliseconds)
    await client.set_finger_positions_and_durations(slave_id, positions, durations)
    await asyncio.sleep(1.0)  # Wait for fingers to reach target position

    # All fingers position control: target position + expected speed
    positions = [500, 500] + [1000] * 4
    speeds = [500] * 6  # Expected speed to reach target position
    await client.set_finger_positions_and_speeds(slave_id, positions, speeds)
    await asyncio.sleep(1.0)  # Wait for fingers to reach target position


async def get_and_display_motor_status(client, slave_id):
    """Get and display motor status information

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    logger.debug("get_motor_status")
    status: libstark.MotorStatusData = await client.get_motor_status(slave_id)

    # Display detailed status information
    # logger.info(f"positions: {list(status.positions)}")  # Position
    # logger.info(f"speeds: {list(status.speeds)}")        # Speed
    # logger.info(f"currents: {list(status.currents)}")    # Current
    # logger.info(f"states: {list(status.states)}")        # State
    logger.info(f"Finger status: {status.description}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
