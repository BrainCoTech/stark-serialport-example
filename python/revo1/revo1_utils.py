import json
import logging
import sys
import os

# Add parent directory to path to import logger
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from logger import getLogger

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

from bc_stark_sdk import main_mod

libstark = main_mod


async def open_modbus_revo1(port_name = None, quick = True):
    """
    Automatically detect and open Modbus connection for Revo1 dexterous hand

    Revo1 dexterous hand supports both Modbus and Protobuf communication protocols, Modbus is recommended.
    This function automatically detects port, baud rate and device ID, and establishes connection.

    Args:
        port_name (str, optional): Serial port name, None by default means auto-detect the first available port.
            Can specify a specific port when multiple ports exist, e.g.: "/dev/ttyUSB0"
        quick (bool, optional): Quick detection mode configuration, True by default.
            True: Only detect common baud rates and default device ID, faster
            False: Detect device ID range 1~247, more comprehensive but takes longer

    Returns:
        tuple: (client, slave_id) - Modbus client instance and device slave ID

    Raises:
        AssertionError: When detected protocol is not Modbus

    Example:
        client, slave_id = await open_modbus_revo1()
        # Or manually specify parameters:
        # client: libstark.PyDeviceContext = await libstark.modbus_open(port_name, baudrate)
    """
    # Auto-detect the first available slave device
    (protocol, detected_port_name, baudrate, slave_id) = await libstark.auto_detect_modbus_revo1(
        port_name, quick
    )

    # Verify detected protocol type
    assert (
        protocol == libstark.StarkProtocolType.Modbus
    ), "Only Modbus protocol is supported for Revo1"

    # Establish Modbus connection
    client: libstark.PyDeviceContext = await libstark.modbus_open(detected_port_name, baudrate)

    # Get device information
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    if device_info.is_revo1():
        if device_info.is_revo1_touch():
            logger.info(f"Touch hand")
        else:
            logger.info(f"Standard version")

    return (client, slave_id)


def get_stark_port_name():
    """
    Get the first available Stark device port name

    Scan all available serial port devices in the system and return the first detected port name.

    Returns:
        str: Name of the first available port, returns None if no port found or parsing failed

    Note:
        The port name returned by this function can be used to manually specify connection port
    """
    # Get list of all available ports in the system
    ports = libstark.list_available_ports()
    logger.info(f"available_ports: {ports}")

    # Decode byte data and parse JSON format port information
    ports_json = json.loads(ports.decode("utf-8"))
    if not ports_json:
        logger.error("parse ports failed")
        return

    if len(ports) == 0:
        logger.error("No ports found")
        return

    # Select the first available port
    port_name = ports_json[0]["port_name"]
    logger.info(f"Using port: {port_name}")
    return port_name


# Maximum angle limits for each joint (unit: degrees)
# Index corresponds to joints: [Thumb, Index, Middle, Ring, Pinky, Wrist]
MAX_ANGLES = [55, 90, 70, 70, 70, 70]


def convert_to_position(angles):
    """
    Convert angle values to position percentages

    Map actual angle values to 0-100 position percentage range based on maximum angle limits for each joint.

    Args:
        angles (list): List of angle values for each joint, length should be 6

    Returns:
        list: Corresponding position percentage list, range [0-100]

    Example:
        angles = [30, 45, 35, 35, 35, 35]
        positions = convert_to_position(angles)  # [55, 50, 50, 50, 50, 50]
    """
    # Map angles proportionally to [0, 100] range and limit boundaries
    mapped = [
        max(0, min(100, round(angle * 100.0 / max_angle)))
        for angle, max_angle in zip(angles, MAX_ANGLES)
    ]
    return mapped


def convert_to_angle(positions):
    """
    Convert position percentages to angle values

    Map 0-100 position percentages back to actual angle values based on maximum angle limits for each joint.

    Args:
        positions (list): List of position percentages for each joint, range [0-100]

    Returns:
        list: Corresponding angle value list

    Example:
        positions = [50, 50, 50, 50, 50, 50]
        angles = convert_to_angle(positions)  # [27.5, 45.0, 35.0, 35.0, 35.0, 35.0]
    """
    # Limit position values to [0, 100] range and map proportionally to angles
    mapped = [
        min(max(0, pos), 100) * max_angle / 100.0
        for pos, max_angle in zip(positions, MAX_ANGLES)
    ]
    return mapped


def convert_to_mA(currents):
    """
    Convert current values to milliampere units

    Convert current values to milliampere units based on the hardware characteristics of the Revo1 dexterous hand.

    Args:
        currents (list): List of raw current values

    Returns:
        list: List of current values in milliampere units

    Note:
        Conversion factor is 6, this factor is specifically for Revo1 dexterous hand hardware
    """
    return [int(current * 6) for current in currents]
