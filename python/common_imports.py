"""
Common imports for all Python examples

Provides unified SDK import, logging, and utility functions.
Use this in all example scripts to avoid duplication.
"""

import sys
import os
import logging

# Ensure parent directory is in path
_current_dir = os.path.dirname(os.path.abspath(__file__))
if _current_dir not in sys.path:
    sys.path.insert(0, _current_dir)

# Import logger
from logger import getLogger

# # Initialize logger
logger = getLogger(logging.INFO)
# logger = getLogger(logging.DEBUG)

# Import SDK
try:
    from bc_stark_sdk import main_mod as sdk
    libstark = sdk  # Alias for backward compatibility
except ImportError:
    sdk = None
    libstark = None
    logger.error("bc_stark_sdk not found. Install: pip install bc_stark_sdk")


def check_sdk():
    """Check if SDK is available, exit if not"""
    if sdk is None:
        print("Error: bc_stark_sdk not found.")
        print("Install: pip install bc_stark_sdk")
        sys.exit(1)
    return sdk


def int_to_baudrate(value: int):
    """Convert int baudrate to Baudrate enum"""
    if sdk is None:
        return None
    return sdk.Baudrate.from_int(value) if hasattr(sdk.Baudrate, 'from_int') else _int_to_baudrate_fallback(value)


def _int_to_baudrate_fallback(value: int):
    """Fallback baudrate conversion for older SDK versions"""
    baudrate_map = {
        115200: sdk.Baudrate.Baud115200,
        57600: sdk.Baudrate.Baud57600,
        19200: sdk.Baudrate.Baud19200,
        460800: sdk.Baudrate.Baud460800,
        1000000: sdk.Baudrate.Baud1Mbps,
        2000000: sdk.Baudrate.Baud2Mbps,
        5000000: sdk.Baudrate.Baud5Mbps,
    }
    return baudrate_map.get(value, sdk.Baudrate.Baud460800)


def str_to_protocol_type(protocol_str: str):
    """Convert protocol string to StarkProtocolType enum (deprecated)

    Note: DetectedDevice.protocol_type is now an enum directly.
    This function is kept for backward compatibility.

    Args:
        protocol_str: Protocol string ("CanFd", "Can", "Modbus", "Protobuf")

    Returns:
        StarkProtocolType enum or None if unknown
    """
    if sdk is None:
        return None
    protocol_map = {
        "CanFd": sdk.StarkProtocolType.CanFd,
        "Can": sdk.StarkProtocolType.Can,
        "Modbus": sdk.StarkProtocolType.Modbus,
    }
    return protocol_map.get(protocol_str)


def get_protocol_display_name(protocol_type) -> str:
    """Get display name for protocol type enum

    Args:
        protocol_type: StarkProtocolType enum

    Returns:
        Human-readable protocol name
    """
    if sdk is None:
        return "Unknown"
    if protocol_type == sdk.StarkProtocolType.Modbus:
        return "Modbus (RS485)"
    elif protocol_type == sdk.StarkProtocolType.Can:
        return "CAN 2.0"
    elif protocol_type == sdk.StarkProtocolType.CanFd:
        return "CANFD"
    else:
        return str(protocol_type)


def get_hw_type_name(hw_type) -> str:
    """Get hardware type name

    Args:
        hw_type: StarkHardwareType enum or int value
    """
    if sdk is None:
        return "Unknown"

    # If it's a StarkHardwareType enum, use str() to get name
    if hasattr(hw_type, 'int_value'):
        name = str(hw_type)
        # Add friendly description for all types
        descriptions = {
            "Revo1Protobuf": "Revo1 Protobuf (Legacy)",
            "Revo1Basic": "Revo1 Basic",
            "Revo1Touch": "Revo1 Touch",
            "Revo1Advanced": "Revo1 Advanced",
            "Revo1AdvancedTouch": "Revo1 Advanced Touch",
            "Revo2Basic": "Revo2 Basic",
            "Revo2Touch": "Revo2 Touch (Capacitive)",
            "Revo2TouchPressure": "Revo2 Touch (Pressure)",
        }
        return descriptions.get(name, name)

    # Fallback to manual mapping using int values
    names = {
        0: "Revo1 Protobuf (Legacy)",
        1: "Revo1 Basic",
        2: "Revo1 Touch",
        3: "Revo1 Advanced",
        4: "Revo1 Advanced Touch",
        5: "Revo2 Basic",
        6: "Revo2 Touch (Capacitive)",
        7: "Revo2 Touch (Pressure)",
    }
    value = hw_type if isinstance(hw_type, int) else -1
    return names.get(value, f"Unknown ({value})")


# Hardware type helpers - use enum methods if available, otherwise fallback
def uses_revo1_motor_api(hw_type) -> bool:
    """Check if device uses Revo1 Motor API

    Revo1 Basic/Touch use Revo1 Motor API.
    Revo1 Advanced/AdvancedTouch and all Revo2 use Revo2 Motor API.
    """
    if hasattr(hw_type, 'uses_revo1_motor_api'):
        return hw_type.uses_revo1_motor_api()

    # Fallback using enum comparison
    if sdk is None:
        return False
    return hw_type in [
        sdk.StarkHardwareType.Revo1Protobuf,
        sdk.StarkHardwareType.Revo1Basic,
        sdk.StarkHardwareType.Revo1Touch,
    ]


def uses_revo2_motor_api(hw_type) -> bool:
    """Check if device uses Revo2 Motor API

    Revo1 Advanced/AdvancedTouch and all Revo2 use Revo2 Motor API.
    """
    return not uses_revo1_motor_api(hw_type)


# Legacy aliases for backward compatibility
is_revo1 = uses_revo1_motor_api
is_revo2 = uses_revo2_motor_api


def has_touch(hw_type) -> bool:
    """Check if device has touch sensor"""
    if hasattr(hw_type, 'has_touch'):
        return hw_type.has_touch()

    # Fallback using enum comparison
    if sdk is None:
        return False
    return hw_type in [
        sdk.StarkHardwareType.Revo1Touch,
        sdk.StarkHardwareType.Revo1AdvancedTouch,
        sdk.StarkHardwareType.Revo2Touch,
        sdk.StarkHardwareType.Revo2TouchPressure,
    ]


def uses_revo1_touch_api(hw_type) -> bool:
    """Check if device uses Revo1 Touch API (capacitive, different sensor counts per finger)

    Revo1Touch and Revo1AdvancedTouch use Revo1 Touch API.
    """
    if hasattr(hw_type, 'uses_revo1_touch_api'):
        return hw_type.uses_revo1_touch_api()

    # Fallback using enum comparison
    if sdk is None:
        return False
    return hw_type in [
        sdk.StarkHardwareType.Revo1Touch,
        sdk.StarkHardwareType.Revo1AdvancedTouch,
    ]


def uses_revo2_touch_api(hw_type) -> bool:
    """Check if device uses Revo2 Touch API (capacitive, uniform sensor counts per finger)

    Only Revo2Touch uses Revo2 Touch API.
    Revo2TouchPressure uses Pressure Touch API instead.
    """
    if hasattr(hw_type, 'uses_revo2_touch_api'):
        return hw_type.uses_revo2_touch_api()

    # Fallback using enum comparison
    if sdk is None:
        return False
    return hw_type == sdk.StarkHardwareType.Revo2Touch


def uses_pressure_touch_api(hw_type) -> bool:
    """Check if device uses Pressure/Modulus Touch API

    Only Revo2TouchPressure uses Pressure Touch API.
    """
    if hasattr(hw_type, 'uses_pressure_touch_api'):
        return hw_type.uses_pressure_touch_api()

    # Fallback using enum comparison
    if sdk is None:
        return False
    return hw_type == sdk.StarkHardwareType.Revo2TouchPressure


# Legacy alias
def has_pressure_touch(hw_type) -> bool:
    """Check if device has pressure touch sensor (alias for uses_pressure_touch_api)"""
    return uses_pressure_touch_api(hw_type)


def is_protobuf_device(hw_type) -> bool:
    """Check if device uses Protobuf protocol (legacy Revo1)
    
    Protobuf devices don't support single finger control,
    must use set_finger_positions() instead of set_finger_position().
    """
    if sdk is None:
        return False
    return hw_type == sdk.StarkHardwareType.Revo1Protobuf


def supports_durations_api(hw_type) -> bool:
    """Check if device supports set_finger_positions_and_durations API
    
    Only Revo2 motor API (Revo1Advanced, Revo1AdvancedTouch, Revo2*) supports durations.
    Revo1 motor API (Revo1Protobuf, Revo1Basic, Revo1Touch) does not support durations.
    """
    return uses_revo2_motor_api(hw_type)
