#!/usr/bin/env python3
"""
Stark Hand DFU - Firmware Upgrade

Auto-detects device and performs firmware upgrade.
Supports: Modbus (RS485), CAN 2.0, CANFD, SocketCAN, ZLG

Run:
    python hand_dfu.py [firmware_file]           # Auto-detect
    python hand_dfu.py -s can0 1 firmware.bin    # SocketCAN
    python hand_dfu.py -z 2 firmware.bin         # ZLG CAN
    python hand_dfu.py -h                        # Show help

Example:
    python hand_dfu.py ../ota_bin/revo2_v1.0.20.bin
    python hand_dfu.py  # Auto-select firmware based on detected device
"""

import asyncio
import sys
import os
import pathlib
import argparse

# Setup path and imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_imports import (
    sdk, check_sdk, get_hw_type_name, uses_revo2_motor_api, logger
)
from common_init import (
    DeviceContext, parse_args_and_init, cleanup_context, print_init_usage
)

check_sdk()

# Firmware paths configuration
SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
OTA_DIR = SCRIPT_DIR.parent / "ota_bin"

FIRMWARE_PATHS = {
    "revo1_basic": OTA_DIR / "modbus" / "FW_MotorController_Release_SecureOTA_0.1.7.C.ota",
    "revo1_touch": OTA_DIR / "touch" / "FW_MotorController_Release_SecureOTA_V1.8.53.F.ota",
    "revo1_advanced": OTA_DIR / "stark2" / "Revo1.8_V1.0.3.C_2602031800.bin",
    "revo2_485_canfd": OTA_DIR / "stark2" / "Revo2_V1.0.20.U_2501091030.bin",
    "revo2_ethercat_canfd": OTA_DIR / "stark2" / "Revo2_V1.0.16.F_2512051618.bin",
    # "revo2_485_canfd": OTA_DIR / "stark2" / "Revo2_V1.0.16.F_2512051618.bin", 
}

# DFU state tracking
dfu_completed = False
dfu_failed = False


def on_dfu_state(slave_id: int, state: int):
    """DFU state callback"""
    global dfu_completed, dfu_failed
    dfu_state = sdk.DfuState(state)
    logger.info(f"Slave {slave_id} DFU state: {dfu_state}")

    if dfu_state == sdk.DfuState.Completed:
        dfu_completed = True
    elif dfu_state == sdk.DfuState.Aborted:
        dfu_failed = True


def on_dfu_progress(slave_id: int, progress: float):
    """DFU progress callback"""
    if progress >= 1.0:
        print(f"\r[DFU] Slave {slave_id} progress: 100%")
    else:
        print(f"[DFU] Slave {slave_id} progress: {progress * 100:.1f}%\r", end="", flush=True)


def select_firmware(hw_type, protocol_type) -> str:
    """Select appropriate firmware based on hardware type"""
    if hw_type == sdk.StarkHardwareType.Revo1Basic:
        return str(FIRMWARE_PATHS["revo1_basic"])
    if hw_type == sdk.StarkHardwareType.Revo1Touch:
        return str(FIRMWARE_PATHS["revo1_touch"])
    if hw_type in [sdk.StarkHardwareType.Revo1Advanced, sdk.StarkHardwareType.Revo1AdvancedTouch]:
        return str(FIRMWARE_PATHS["revo1_advanced"])
    if uses_revo2_motor_api(hw_type):
        return str(FIRMWARE_PATHS["revo2_485_canfd"])
    raise ValueError(f"Cannot determine firmware for hw_type={hw_type}")


def print_usage():
    print("Usage: python hand_dfu.py [options] [firmware_file]")
    print("\nIf firmware_file is not specified, auto-selects based on device type.")
    print_init_usage("python hand_dfu.py")
    print("\nExamples:")
    print("  python hand_dfu.py firmware.bin")
    print("  python hand_dfu.py -s can0 1 firmware.bin")
    print("  python hand_dfu.py -z 2 firmware.bin")


async def main():
    global dfu_completed, dfu_failed

    # Check for help
    if '-h' in sys.argv or '--help' in sys.argv:
        print_usage()
        return 0

    # Create extra parser for DFU-specific args
    extra_parser = argparse.ArgumentParser(add_help=False)
    extra_parser.add_argument('firmware', nargs='?', default=None,
                             help='Firmware file path')

    print("=== Stark Hand DFU ===\n")

    # Parse args and initialize
    device_ctx, extra_args, remaining = await parse_args_and_init(sys.argv, extra_parser)
    if device_ctx is None:
        return 1

    firmware_path = extra_args.firmware if extra_args else None

    # Auto-select firmware if not provided
    if firmware_path is None:
        try:
            firmware_path = select_firmware(device_ctx.hw_type, device_ctx.protocol_type)
            logger.info(f"Auto-selected firmware: {firmware_path}")
        except ValueError as e:
            logger.error(str(e))
            await cleanup_context(device_ctx)
            return 1

    if not os.path.exists(firmware_path):
        logger.error(f"Firmware file not found: {firmware_path}")
        logger.error(f"Please download firmware to: {OTA_DIR}")
        await cleanup_context(device_ctx)
        return 1

    print(f"\n[WARNING] Firmware upgrade will begin.")
    print(f"Firmware: {firmware_path}")
    input("\nPress Enter to continue or Ctrl+C to cancel...\n")

    # Start DFU
    logger.info("Starting firmware upgrade...")
    try:
        device_ctx.ctx.start_dfu(device_ctx.slave_id, firmware_path, 5, on_dfu_state, on_dfu_progress)

        while not dfu_completed and not dfu_failed:
            await asyncio.sleep(0.5)

        if dfu_completed:
            print("\n")
            logger.info("Firmware upgrade completed successfully!")
            result = 0
        else:
            print("\n")
            logger.error("Firmware upgrade failed!")
            result = 1

    except Exception as e:
        logger.error(f"DFU error: {e}")
        result = 1

    await cleanup_context(device_ctx)
    return result


if __name__ == "__main__":
    try:
        result = asyncio.run(main())
        sys.exit(result or 0)
    except KeyboardInterrupt:
        print("\nUser cancelled")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
