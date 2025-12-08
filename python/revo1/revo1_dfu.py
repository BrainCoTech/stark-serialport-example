"""
Revo1 Dexterous Hand Firmware Upgrade (DFU) Example

This example demonstrates how to perform firmware upgrade for Revo1 dexterous hand, including:
- Automatic detection of device type and protocol
- Selection of corresponding firmware file based on device type
- Execution of firmware upgrade process and progress monitoring
- Handling of upgrade status and result feedback

Important warnings:
- Different hardware versions must use corresponding firmware files
- Using wrong firmware may cause device to fail to boot, requiring disassembly and re-flashing
- Do not disconnect device during upgrade process
"""

import asyncio
import sys
import pathlib
import os
from utils import setup_shutdown_event
from revo1_utils import *

# IMPORTANT!!!: Different hardware requires corresponding firmware, otherwise device disassembly and re-flashing is required
# Firmware upgrade file path configuration
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
logger.info(f"parent_dir: {parent_dir}")

# Revo1 touch hand firmware path
# Download link: https://app.brainco.cn/universal/bc-stark-sdk/firmware/touch/FW_MotorController_Release_SecureOTA_V1.8.53.F.ota
revo1_touch_ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "touch",
    "FW_MotorController_Release_SecureOTA_V1.8.53.F.ota",
)

# Revo1 basic version Modbus firmware path
# Download link: https://app.brainco.cn/universal/bc-stark-sdk/firmware/modbus/FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota
revo1_basic_ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "modbus",
    # "FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota",
    "FW_MotorController_Release_SecureOTA_0.3.1.C.ota",
)

# Revo1 Protobuf protocol firmware path
revo1_protobuf_ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "protobuf",
    "FW_MotorController_Release_SecureOTA_485_9.2.7.ota",
)

# Global variables for async event handling
shutdown_event = None
main_loop = None


def on_dfu_state(_slave_id, state):
    """
    DFU state change callback function

    Called when state changes during firmware upgrade process.

    Args:
        _slave_id: Device ID (unused)
        state: DFU state enum value
    """
    logger.info(f"DFU STATE: {libstark.DfuState(state)}")
    dfu_state = libstark.DfuState(state)

    # Set shutdown event when upgrade is completed or aborted
    if state == libstark.DfuState.Completed or dfu_state == libstark.DfuState.Aborted:
        if main_loop and shutdown_event:
            if not shutdown_event.is_set():
                logger.info("Using call_soon_threadsafe to set event")
                main_loop.call_soon_threadsafe(shutdown_event.set)


def on_dfu_progress(_slave_id, progress):
    """
    DFU progress update callback function

    Periodically reports upgrade progress during firmware upgrade process.

    Args:
        _slave_id: Device ID (unused)
        progress: Upgrade progress, range 0.0-1.0
    """
    logger.info(f"progress: {progress * 100.0 :.2f}%")


async def main():
    """
    Main function: Execute firmware upgrade process
    """
    global shutdown_event, main_loop
    main_loop = asyncio.get_running_loop()
    shutdown_event = setup_shutdown_event(logger)

    # Automatically detect device protocol and connection parameters
    (protocol, port_name, baudrate, slave_id) = await libstark.auto_detect_device()
    logger.info(
        f"Detected protocol: {protocol}, port: {port_name}, baudrate: {baudrate}, slave_id: {slave_id}"
    )

    # Establish Modbus connection
    client: libstark.PyDeviceContext = await libstark.modbus_open(port_name, baudrate)

    # Select corresponding firmware file based on device type
    ota_bin_path = ""

    # IMPORTANT!!!: Different hardware requires corresponding firmware, otherwise device disassembly and re-flashing is required
    if protocol == libstark.StarkProtocolType.Modbus:
        # Get device info to determine hardware type
        device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
        logger.info(f"Device info: {device_info.description}")

        if device_info.is_revo1():
            if baudrate != libstark.Baudrate.Baud115200:
                logger.warning(
                    f"Non-115200 baud rate may not be able to upgrade, please set baud rate to 115200 before upgrading, current baud rate: {baudrate}"
                )

            if device_info.is_revo1_touch():
                # Touch hand uses touch firmware
                ota_bin_path = revo1_touch_ota_bin_path
            else:
                # Basic version uses Modbus firmware
                ota_bin_path = revo1_basic_ota_bin_path
                # Optional: Use Protobuf firmware
                # ota_bin_path = revo1_protobuf_ota_bin_path
    elif protocol == libstark.StarkProtocolType.Protobuf:
        # Protobuf protocol device uses Modbus basic version firmware
        ota_bin_path = revo1_basic_ota_bin_path

    # Verify firmware file exists
    if not os.path.exists(ota_bin_path):
        logger.warning(f"OTA file does not exist: {ota_bin_path}")
        exit(0)
    else:
        logger.info(f"OTA file path: {ota_bin_path}")

    import time

    start_time = time.perf_counter()

    # Start firmware upgrade
    logger.info("start_dfu")
    wait_seconds = 5  # Time to wait for device to enter DFU mode
    await client.start_dfu(
        slave_id,
        ota_bin_path,
        wait_seconds,
        on_dfu_state,  # State change callback
        on_dfu_progress,  # Progress update callback
    )

    # Wait for upgrade completion or shutdown event
    await shutdown_event.wait()
    elapsed = time.perf_counter() - start_time
    logger.info(f"Elapsed: {elapsed:.1f}s")

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
