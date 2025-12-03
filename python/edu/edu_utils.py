"""
Education SDK Utilities

This module provides utility functions for interacting with `bc-edu-sdk`,
mainly for device discovery, port management, and common helper utilities.
"""

import json
import logging
from typing import Optional, List, Dict, Any

from logger import getLogger
from bc_edu_sdk import main_mod

# Configuration constants
VENDOR_ID = 21059  # BrainCo vendor ID
GLOVE_PRODUCT_ID = 6  # Glove device product ID
ARMBAND_PRODUCT_ID_PRIMARY = 1  # Primary armband device product ID
ARMBAND_PRODUCT_ID_SECONDARY = 5  # Secondary armband device product ID

# Initialize logger and SDK module
# logger = getLogger(logging.DEBUG)  # Optional: use DEBUG level logging
logger = getLogger(logging.INFO)  # Default: use INFO level logging
libedu = main_mod

def get_usb_available_ports() -> None:
    """
    Get information for all available USB ports.

    This is a convenience function to display all available USB ports
    in the current system. It is mainly used for debugging and port discovery.
    """
    libedu.get_usb_available_ports()


def _get_first_port_name(ports_data: bytes, device_type: str) -> Optional[str]:
    """
    Generic helper to extract the first available port name from port data.

    This function parses the port scan result of various devices and
    extracts the first available port name. It supports JSON-formatted
    port data and includes error handling and logging.

    Args:
        ports_data: Port data, JSON-formatted bytes
            Expected format: [{"port_name": "COM1", ...}, {"port_name": "COM2", ...}]
        device_type: Device type name, used in logs (e.g. "Stark", "Glove", "Armband")

    Returns:
        The first available port name, or None if parsing failed.
        On success, returns something like "/dev/ttyUSB0" or "COM3".

    Note:
        - The first port in the list is chosen by default
        - Includes full error handling to ensure stability
        - All operations are logged to help with debugging
    """
    logger.info(f"Available {device_type} ports: {ports_data}")

    try:
        # Decode bytes and parse JSON-formatted data
        ports_json: List[Dict[str, Any]] = json.loads(ports_data.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        logger.error(f"Failed to parse {device_type} ports data: {e}")
        return None

    # Check whether the parsed result is an empty list
    if not ports_json:
        logger.warning(f"No {device_type} ports found in scan results")
        return None

    # Extract the first port name
    try:
        port_name = ports_json[0]["port_name"]
        logger.info(f"Using {device_type} port: {port_name}")
        return port_name
    except (KeyError, IndexError) as e:
        logger.error(f"Invalid port data structure for {device_type}: {e}")
        return None

def get_glove_port_name() -> Optional[str]:
    """
    Get the first available glove device port name.

    Scan glove devices via USB VID/PID and return the first detected
    port name. A specific vendor ID and product ID are used to identify
    glove devices.

    Returns:
        The first available port name, or None if no port was found
        or parsing failed. On success, returns something like
        "/dev/ttyUSB0" or "COM3".

    Note:
        - Uses VID=21059, PID=6 to identify glove devices
        - This function is dedicated to detecting USB glove devices
        - If multiple glove devices are present, only the first
          detected port is returned

    Example:
        >>> port = get_glove_port_name()
        >>> if port:
        ...     print(f"Found glove device at: {port}")
        ... else:
        ...     print("No glove device found")
    """
    try:
        ports = libedu.available_usb_ports(VENDOR_ID, GLOVE_PRODUCT_ID)
        return _get_first_port_name(ports, "Glove")
    except Exception as e:
        logger.error(f"Error scanning for glove devices: {e}")
        return None


def get_armband_port_name() -> Optional[str]:
    """
    Get the first available armband device port name.

    Scan armband devices via USB VID/PID and return the first detected
    port name. Multiple product IDs are supported to improve compatibility.

    Returns:
        The first available port name, or None if no port was found
        or parsing failed. On success, returns something like
        "/dev/ttyUSB0" or "COM3".

    Note:
        - Tries the primary product ID (PID=1) first, then the secondary
          product ID (PID=5)
        - This function is dedicated to detecting USB armband devices
        - If multiple armband devices are present, only the first
          detected port is returned

    Example:
        >>> port = get_armband_port_name()
        >>> if port:
        ...     print(f"Found armband device at: {port}")
        ... else:
        ...     print("No armband device found")
    """
    try:
        # 首先尝试主要产品ID
        ports = libedu.available_usb_ports(VENDOR_ID, ARMBAND_PRODUCT_ID_PRIMARY)
        port_name = _get_first_port_name(ports, "Armband")

        if port_name:
            return port_name

        # 如果主要产品ID未找到设备，尝试备用产品ID
        logger.info("Primary armband product ID not found, trying secondary ID...")
        ports = libedu.available_usb_ports(VENDOR_ID, ARMBAND_PRODUCT_ID_SECONDARY)
        return _get_first_port_name(ports, "Armband")

    except Exception as e:
        logger.error(f"Error scanning for armband devices: {e}")
        return None


def get_all_device_ports() -> Dict[str, Optional[str]]:
    """
    Get the port information for all supported devices.

    Returns:
        A dictionary containing all device types and their ports.
        Format: {"glove": "COM3", "armband": "/dev/ttyUSB0"}
    """
    devices = {
        "glove": get_glove_port_name(),
        "armband": get_armband_port_name()
    }

    logger.info(f"Device scan results: {devices}")
    return devices


def is_device_connected(device_type: str) -> bool:
    """
    Check whether a specific type of device is connected.

    Args:
        device_type: Device type ("glove" or "armband")

    Returns:
        True if the device is connected, otherwise False.
    """
    if device_type.lower() == "glove":
        return get_glove_port_name() is not None
    elif device_type.lower() == "armband":
        return get_armband_port_name() is not None
    else:
        logger.warning(f"Unknown device type: {device_type}")
        return False


def scan_and_report_devices() -> None:
    """
    Scan and report all connected devices.

    This is a convenience function for quickly checking the status
    of all currently connected devices.
    """
    logger.info("Scanning for connected devices...")

    devices = get_all_device_ports()
    connected_devices = [name for name, port in devices.items() if port is not None]

    if connected_devices:
        logger.info(f"Found {len(connected_devices)} connected device(s):")
        for device_name, port in devices.items():
            if port:
                logger.info(f"  - {device_name.capitalize()}: {port}")
    else:
        logger.warning("No devices found. Please check connections.")

    # Display all available ports as reference
    logger.info("All available USB ports:")
    get_usb_available_ports()


def print_afe_timestamps(logger, data) -> None:
    """
    Print timestamp information of AFE data.

    Args:
        logger: Logger instance
        data: List of AFE data
    """
    if len(data) <= 6:
        for item in data:
            logger.info(f"{item}")
        return
    for item in data[:3]:
        logger.info(f"{item}")
    logger.info("...")
    for item in data[-3:]:
        logger.info(f"{item}")
