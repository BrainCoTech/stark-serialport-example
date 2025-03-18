import asyncio
import logging
import sys
import pathlib
import os
from logger import getLogger
from stark_utils import get_stark_port_name
from utils import setup_shutdown_event
import bc_device_sdk

libstark = bc_device_sdk.stark

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

# 固件升级文件路径
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent
logger.info(f"parent_dir: {parent_dir}")

ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "modbus",
    "FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota",
)  # Modbus固件

if not os.path.exists(ota_bin_path):
    logger.warning(f"OTA文件不存在: {ota_bin_path}")
    exit(0)
else:
    logger.info(f"OTA文件路径: {ota_bin_path}")

shutdown_event = None

def on_dfu_state(_slave_id, state):
    logger.info(f"DFU STATE: {libstark.DfuState(state)}")
    dfu_state = libstark.DfuState(state)
    if state == libstark.DfuState.Completed or dfu_state == libstark.DfuState.Aborted:
        logger.info(f"ota done")
        shutdown_event.set()
        # sys.exit(0)

def on_dfu_progress(_slave_id, progress):
    logger.info(f"progress: {progress * 100.0 :.2f}%")

# Main
async def main():
    global shutdown_event
    shutdown_event = setup_shutdown_event(logger)

    port_name = get_stark_port_name()
    if port_name is None:
        return
    slave_id = 1
    client = await libstark.modbus_open(port_name, libstark.Baudrate.Baud115200, slave_id)

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.firmware_version}")  # 固件版本
    logger.info(f"Device info: {device_info.description}")

    # 固件升级
    logger.info("start_dfu")
    wait_seconds = 5  # wait device enter DFU mode
    await client.start_dfu(
        slave_id,
        ota_bin_path,
        wait_seconds,
        on_dfu_state,
        on_dfu_progress,
    )

    # 等待关闭事件
    await shutdown_event.wait()

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
