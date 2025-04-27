import asyncio
import sys
import pathlib
import os
from utils import setup_shutdown_event
import asyncio
from stark_utils import get_stark_port_name, libstark, logger

# 固件升级文件路径
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent
logger.info(f"parent_dir: {parent_dir}")

stark_toch = True
stark_v2 = False

if stark_toch:
    # 触觉固件
    ota_bin_path = os.path.join(
        parent_dir,
        "ota_bin",
        "touch",
        "FW_MotorController_Release_SecureOTA_V1.8.24F.ota",
        # "FW_MotorController_Release_SecureOTA_V1.8.21F.ota",
    )

elif stark_v2:
    # ModbusV2固件
    ota_bin_path = os.path.join(
        parent_dir,
        "ota_bin",
        "stark2",
        "stark2_fw_V0.0.7_20250409094916.bin",
    )
else:
    # ModbusV1固件
    ota_bin_path = os.path.join(
        parent_dir,
        "ota_bin",
        "modbus",
        "FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota",
    )

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
        logger.info(f"DFU Stopped")
        shutdown_event.set()
        # sys.exit(0)


def on_dfu_progress(_slave_id, progress):
    logger.info(f"progress: {progress * 100.0 :.2f}%")


# Main
async def main():
    libstark.init_config(
            libstark.StarkFirmwareType.V1Touch
        if stark_toch
        else
            libstark.StarkFirmwareType.V2Standard
        if stark_v2
        else libstark.StarkFirmwareType.V1Standard
    )

    global shutdown_event
    shutdown_event = setup_shutdown_event(logger)

    port_name = get_stark_port_name()
    if port_name is None:
        return

    # 一代手默认ID为1
    # 左手默认ID为0x7e，右手默认ID为0x7f
    slave_id = 0x7E if stark_v2 else 1
    # 一代手默认波特率115200
    # 二代手默认波特率460800
    baurate = libstark.Baudrate.Baud460800 if stark_v2 or stark_toch else libstark.Baudrate.Baud115200
    logger.info(f"slave_id: {slave_id}, baurate: {baurate}")
    logger.info(f"port_name: {port_name}")
    # 打开串口
    # fmt: off
    client = await libstark.modbus_open(port_name, baurate, slave_id)

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
