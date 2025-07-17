import asyncio
import sys
import pathlib
import os
import time

from utils import setup_shutdown_event
from revo2_utils import libstark, logger

# 重要！！！：不同硬件需使用相应固件，否则需要拆设备重新烧录
# 固件升级文件路径
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
logger.info(f"parent_dir: {parent_dir}")

# Modbus固件，二代灵巧手基础版
revo2_basic_ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "stark2",
    "stark2_fw_V0.0.10_20250714180644.bin",
)

shutdown_event = None
main_loop = None


def on_dfu_state(_slave_id, state):
    logger.info(f"DFU STATE: {libstark.DfuState(state)}")
    dfu_state = libstark.DfuState(state)
    if dfu_state == libstark.DfuState.Completed or dfu_state == libstark.DfuState.Aborted:
        if main_loop and shutdown_event:
            if not shutdown_event.is_set():
                logger.info("Using call_soon_threadsafe to set event")
                main_loop.call_soon_threadsafe(shutdown_event.set)
                logger.info("call_soon_threadsafe called")


def on_dfu_progress(_slave_id, progress):
    logger.info(f"progress: {progress * 100.0 :.2f}%")

# Main
async def main():
    global  main_loop
    main_loop = asyncio.get_running_loop()

    (protocol, port_name, baudrate, slave_id) = await libstark.auto_detect_device()
    logger.info(f"Detected protocol: {protocol}, port: {port_name}, baudrate: {baudrate}, slave_id: {slave_id}")

    ota_bin_path = revo2_basic_ota_bin_path
    if not os.path.exists(ota_bin_path):
        logger.warning(f"OTA文件不存在: {ota_bin_path}")
        exit(0)
    else:
        logger.info(f"OTA文件路径: {ota_bin_path}")

    await start_dfu(port_name, baudrate, slave_id, ota_bin_path)
    # 测试再次升级
    # await asyncio.sleep(5)
    # await start_dfu(port_name, baudrate, slave_id, ota_bin_path)

    sys.exit(0)

async def start_dfu(port_name, baudrate, slave_id, ota_bin_path):
    global shutdown_event
    shutdown_event = setup_shutdown_event(logger)

    client = await libstark.modbus_open(port_name, baudrate)
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    wait_seconds = 5  # wait device enter DFU mode
    await client.start_dfu(
        slave_id,
        ota_bin_path,
        wait_seconds,
        on_dfu_state,
        on_dfu_progress,
    )

    # 等待关闭事件
    logger.info("Waiting for DFU to complete...")
    await shutdown_event.wait()
    logger.info("DFU completed, shutdown event received!")
    logger.info("start modbus close")
    libstark.modbus_close(client)
    logger.info("Modbus client closed")

if __name__ == "__main__":
    asyncio.run(main())
