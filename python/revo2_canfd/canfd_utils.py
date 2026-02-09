import sys
import os

# Import from common_imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_imports import logger, libstark
from common_utils import setup_shutdown_event


# Pressure touch finger names
PRESSURE_FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"]


def print_pressure_touch_summary(touch_summary: list[int]):
    """Print pressure touch summary with finger labels"""
    if len(touch_summary) >= 6:
        logger.info(
            f"Pressure Summary: "
            f"Thumb={touch_summary[0]:4d}, Index={touch_summary[1]:4d}, "
            f"Middle={touch_summary[2]:4d}, Ring={touch_summary[3]:4d}, "
            f"Pinky={touch_summary[4]:4d}, Palm={touch_summary[5]:4d}"
        )
    else:
        logger.info(f"Pressure Summary: {touch_summary}")


def print_pressure_touch_data(touch_data: list[int]):
    """Print pressure touch detailed data"""
    logger.info(f"Pressure Data ({len(touch_data)} values): {touch_data[:12]}...")


async def display_pressure_touch_status(client, slave_id: int):
    """
    Display pressure touch sensor status with formatted output
    
    Args:
        client: Device handler instance
        slave_id: Device ID
    """
    # Get motor status
    status: libstark.MotorStatusData = await client.get_motor_status(slave_id)
    logger.info(f"Motor: {status.description}")

    # Get pressure touch summary (6 values: thumb, index, middle, ring, pinky, palm)
    touch_summary: list[int] = await client.get_modulus_touch_summary(slave_id)
    print_pressure_touch_summary(touch_summary)

    # Get pressure touch detailed data
    touch_data: list[int] = await client.get_modulus_touch_data(slave_id)
    print_pressure_touch_data(touch_data)


async def setup_pressure_touch_sensors(client, slave_id: int):
    """
    Setup and display pressure touch sensor configuration
    
    Args:
        client: Device handler instance
        slave_id: Device ID
    """
    # Check enabled sensors
    bits = await client.get_touch_sensor_enabled(slave_id)
    enabled_fingers = [PRESSURE_FINGER_NAMES[i] for i in range(6) if bits & (1 << i)]
    logger.info(f"Touch Sensors Enabled: {', '.join(enabled_fingers)} ({bits & 0x3F:06b})")

    # Get firmware versions
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Touch FW Versions: {touch_fw_versions}")

    # Get data type
    data_type = await client.get_modulus_touch_data_type(slave_id)
    logger.info(f"Touch Data Type: {data_type}")
