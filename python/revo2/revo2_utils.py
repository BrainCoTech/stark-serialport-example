"""
Revo2 Dexterous Hand Utility Functions Module

This module provides common utility functions for Revo2 dexterous hand, including:
- Automatic detection and establishment of Modbus connection
- Scanning and selection of device ports
- Data type conversion and processing functions
- Device information retrieval and verification
"""

import json
import sys
import logging
from logger import getLogger

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

from bc_stark_sdk import main_mod

libstark = main_mod
libstark.init_config(libstark.StarkProtocolType.Modbus)


async def open_modbus_revo2(port_name=None, quick=True):
    """
    Automatically detect and open Modbus connection for Revo2 dexterous hand

    Revo2 dexterous hand supports Modbus, CANFD, and EtherCAT communication protocols. This function automatically
    detects port, baud rate and device ID, and establishes connection.

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
        client, slave_id = await open_modbus_revo2()
        # Or manually specify parameters:
        # client: libstark.PyDeviceContext = await libstark.modbus_open(port_name, baudrate)
    """
    try:
        # Auto-detect the first available slave device
        (protocol, detected_port_name, baudrate, slave_id) = (
            await libstark.auto_detect_modbus_revo2(port_name, quick)
        )
        # Verify detected protocol type
        assert (
            protocol == libstark.StarkProtocolType.Modbus
        ), "Only Modbus protocol is supported for Revo2"
    except Exception as e:
        logger.error(e)
        sys.exit(1)

    # set_latency_by_com_or_serial(detected_port_name)
    # Establish Modbus connection
    client: libstark.PyDeviceContext = await libstark.modbus_open(detected_port_name, baudrate)

    # Get device information
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    if device_info.is_revo2():
        if device_info.is_revo2_touch():
            logger.info(f"Touch version")
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

def is_positions_open(status: main_mod.MotorStatusData) -> bool:
    """
    Check if fingers are in open state

    Args:
        status (main_mod.MotorStatusData): Motor status data object

    Returns:
        bool: Returns True if fingers are open, otherwise returns False
    """
    return status.positions <= [400, 400, 0, 0, 0, 0]

def is_positions_closed(status: main_mod.MotorStatusData) -> bool:
    """
    Check if fingers are in closed state

    Args:
        status (main_mod.MotorStatusData): Motor status data object

    Returns:
        bool: Returns True if fingers are closed, otherwise returns False
    """

    return status.positions >= [399, 399, 950, 950, 950, 950]
