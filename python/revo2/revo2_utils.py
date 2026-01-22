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
import os
import math
import time
import asyncio

# Add parent directory to path to import logger
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from logger import getLogger

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

from bc_stark_sdk import main_mod as libstark
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

def is_positions_open(status: libstark.MotorStatusData) -> bool:
    """
    Check if fingers are in open state

    Args:
        status (MotorStatusData): Motor status data object

    Returns:
        bool: Returns True if fingers are open, otherwise returns False
    """
    return status.positions <= [400, 400, 0, 0, 0, 0]

def is_positions_closed(status: libstark.MotorStatusData) -> bool:
    """
    Check if fingers are in closed state

    Args:
        status (MotorStatusData): Motor status data object

    Returns:
        bool: Returns True if fingers are closed, otherwise returns False
    """

    return status.positions >= [399, 399, 950, 950, 950, 950]


def init_cosine_trajectory(traj_len=20, min_val=0.0, max_val=100.0, precision=0.1):
    """
    Initialize cosine trajectory for position control testing

    Generate a complete cosine wave trajectory for target position sequence.
    Trajectory range: configurable (default 0-100%), precision: configurable (default 0.1%)

    Note: Thumb finger trajectory will be automatically clamped to max 50% by
    trajectory_control_task() because Thumb has 2 joints controlled together.

    Args:
        traj_len (int): Number of trajectory points (cosine wave sampling points), default 20
        min_val (float): Minimum value of trajectory range (%), default 0.0
        max_val (float): Maximum value of trajectory range (%), default 100.0
        precision (float): Trajectory precision (%), default 0.1

    Returns:
        list: Trajectory point list, each point value range depends on precision
              For 0.1% precision: [0, 1000] representing [0%, 100%]

    Example:
        # Generate default trajectory: 20 points, 0-100%, 0.1% precision
        trajectory = init_cosine_trajectory()

        # Generate custom trajectory: 30 points, 20-80%, 0.1% precision
        trajectory = init_cosine_trajectory(traj_len=30, min_val=20.0, max_val=80.0)
    """
    traj = []
    amplitude = (max_val - min_val) / 2.0
    offset = (max_val + min_val) / 2.0

    for i in range(traj_len):
        # Generate cosine wave: offset + amplitude * cos(2πi/N)
        value = offset + amplitude * math.cos(2 * math.pi * i / traj_len)
        # Convert to specified precision (e.g., 0.1% precision means multiply by 10)
        traj.append(int(value / precision))

    return traj


async def trajectory_control_task(
    client,
    slave_id,
    finger_id,
    trajectory,
    ctrl_interval=0.01,
    stop_flag_ref=None,
    index_ref=None,
    logger_instance=None
):
    """
    Trajectory control task: Send target positions at specified frequency

    Execute high-frequency position control, cyclically traverse trajectory points and send control commands.
    Supports 100Hz control frequency (10ms interval) by default.

    IMPORTANT: For Thumb finger, trajectory values are automatically clamped to max 500
    because the Thumb has 2 joints controlled together.

    Args:
        client: Modbus client instance
        slave_id: Device slave ID
        finger_id: Target finger ID (libstark.FingerId enum)
        trajectory: Trajectory point list, values in 0.1% precision [0, 1000]
        ctrl_interval (float): Control interval in seconds, default 0.01 (100Hz)
        stop_flag_ref (dict): Stop flag reference, e.g., {'value': False}
        index_ref (dict): Trajectory index reference, e.g., {'value': 0}
        logger_instance: Logger instance, uses global logger if None

    Example:
        trajectory = init_cosine_trajectory()
        stop_flag = {'value': False}
        index = {'value': 0}

        # Start control task
        control = asyncio.create_task(
            trajectory_control_task(
                client, slave_id, libstark.FingerId.Ring,
                trajectory, stop_flag_ref=stop_flag, index_ref=index
            )
        )

        # Run for 5 seconds then stop
        await asyncio.sleep(5.0)
        stop_flag['value'] = True
        await control
    """
    # IMPORTANT: Thumb has 2 joints controlled together, clamp trajectory to max 500
    if finger_id == libstark.FingerId.Thumb:
        trajectory = [min(val, 500) for val in trajectory]
        logger.warning("Thumb finger detected, trajectory clamped to max 500")
    
    local_index = 0
    while True:
        # Check stop flag
        if stop_flag_ref and stop_flag_ref.get('value', False):
            break

        task_start = time.perf_counter()

        # Cyclically traverse trajectory points
        local_index = (local_index + 1) % len(trajectory)
        if index_ref is not None:
            index_ref['value'] = local_index

        # Get target position (already in 0.1% precision [0, 1000])
        target = trajectory[local_index]

        # Send position control command, 1ms means fastest response
        await client.set_finger_position(slave_id, finger_id, target)

        elapsed = (time.perf_counter() - task_start) * 1000
        logger.debug(f"[{local_index}] Target: {target} (0.1%), Control elapsed: {elapsed:.1f}ms")

        # Maintain control frequency
        sleep_time = ctrl_interval - (elapsed / 1000.0)
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)

