"""
Revo2 Dexterous Hand Firmware Upgrade (DFU) Example

This example demonstrates how to perform a firmware upgrade for the Revo2 dexterous hand, including:
- Automatic device type and protocol detection
- Firmware file validation and path configuration
- Executing the firmware upgrade process and monitoring progress
- Handling upgrade status and result feedback

Important Warnings:
- Different hardware versions must use their corresponding firmware files
- Using the wrong firmware may cause the device to fail to start and require disassembly and reflashing
- Do not disconnect the device during the upgrade process
- Revo2 devices only support the basic Modbus firmware
"""

import asyncio
import sys
import pathlib
import os
import time

from utils import setup_shutdown_event
from revo2_utils import *

# Important!!!: Different hardware versions must use their corresponding firmware files, otherwise the device needs to be disassembled and reflashed
# Firmware upgrade file path configuration
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
logger.info(f"parent_dir: {parent_dir}")

# Revo2 dexterous hand basic Modbus firmware path
# Revo2 devices only support the basic Modbus firmware
revo2_basic_ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "stark2",
    "Revo2_V1.0.20.U_2601091030.bin",
)

# Global variables for asynchronous event handling
shutdown_event = None
main_loop = None


def on_dfu_state(_slave_id, state):
    """
    DFU state change callback function

    Called when the state changes during the firmware upgrade process.

    Args:
        _slave_id: Device ID (not used)
        state: DFU state enumeration value
    """
    logger.info(f"DFU STATE: {libstark.DfuState(state)}")
    dfu_state = libstark.DfuState(state)

    # When the upgrade is completed or aborted, set the shutdown event
    if (
        dfu_state == libstark.DfuState.Completed
        or dfu_state == libstark.DfuState.Aborted
    ):
        if main_loop and shutdown_event:
            if not shutdown_event.is_set():
                logger.info("Using call_soon_threadsafe to set event")
                main_loop.call_soon_threadsafe(shutdown_event.set)


def on_dfu_progress(_slave_id, progress):
    """
    DFU progress update callback function

    Called periodically during the firmware upgrade process to report progress.

    Args:
        _slave_id: Device ID (not used)
        progress: Upgrade progress, range 0.0-1.0
    """
    logger.info(f"progress: {progress * 100.0 :.2f}%")


async def main():
    """
    Main function: execute the firmware upgrade process
    """
    global main_loop
    main_loop = asyncio.get_running_loop()

    # Automatically detect device protocol and connection parameters
    (protocol, port_name, baudrate, slave_id) = await libstark.auto_detect_device() # Replace with actual serial port name, None will try to automatically detect
    logger.info(
        f"Detected protocol: {protocol}, port: {port_name}, baudrate: {baudrate}, slave_id: {slave_id}"
    )

    # Use Revo2 basic Modbus firmware
    ota_bin_path = revo2_basic_ota_bin_path

    # Verify that the firmware file exists
    if not os.path.exists(ota_bin_path):
        logger.warning(f"OTA file does not exist: {ota_bin_path}")
        exit(0)
    else:
        logger.info(f"OTA file path: {ota_bin_path}")

    # Execute firmware upgrade
    await start_dfu(port_name, baudrate, slave_id, ota_bin_path)

    # Optional: test again to upgrade
    # await asyncio.sleep(5)
    # await start_dfu(port_name, baudrate, slave_id, ota_bin_path)

    sys.exit(0)


async def start_dfu(port_name, baudrate, slave_id, ota_bin_path):
    """
    Start the firmware upgrade process

    Args:
        port_name: Serial port name
        baudrate: Baud rate
        slave_id: Device ID
        ota_bin_path: Firmware file path
    """
    global shutdown_event
    shutdown_event = setup_shutdown_event(logger)

    # Establish Modbus connection
    client: libstark.PyDeviceContext = await libstark.modbus_open(port_name, baudrate)

    # Get device information
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    start_time = time.perf_counter()

    # Start firmware upgrade
    wait_seconds = 5  # Wait for the device to enter DFU mode
    await client.start_dfu(
        slave_id,
        ota_bin_path,
        wait_seconds,
        on_dfu_state,  # State change callback
        on_dfu_progress,  # Progress update callback
    )

    # Wait for upgrade to complete
    logger.info("Revo2 Modbus DFU, Waiting for DFU to complete...")
    await shutdown_event.wait()
    logger.info("DFU completed, shutdown event received!")
    elapsed = time.perf_counter() - start_time
    logger.info(f"Elapsed: {elapsed:.1f}s")

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
