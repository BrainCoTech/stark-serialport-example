import asyncio
import sys
import pathlib
import os
from utils import setup_shutdown_event
from revo1_utils import get_stark_port_name, libstark, logger

# 重要：不同设备，请使用相应固件升级，否则需要拆设备，重新烧录固件
# 固件升级文件路径
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
logger.info(f"parent_dir: {parent_dir}")

stark_touch = True
stark_v2 = False

if stark_touch:
    # Modbus固件，一代灵巧手触觉版
    ota_bin_path = os.path.join(
        parent_dir,
        "ota_bin",
        "touch",
        "FW_MotorController_Release_SecureOTA_V1.8.32.F.ota",
    )

elif stark_v2:
    # Modbus固件，二代灵巧手基础版
    ota_bin_path = os.path.join(
        parent_dir,
        "ota_bin",
        "stark2",
        "stark2_fw_V0.0.10_20250513154323.bin",
    )
else:
    # Modbus固件, 一代灵巧手基础版
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
        if stark_touch
        else
            libstark.StarkFirmwareType.V2Basic
        if stark_v2
        else libstark.StarkFirmwareType.V1Basic
    )

    global shutdown_event
    shutdown_event = setup_shutdown_event(logger)

    port_name = get_stark_port_name()
    if port_name is None:
        return

    # 一代手默认ID为1
    # 左手默认ID为0x7e，右手默认ID为0x7f
    slave_id = 0x7f if stark_v2 else 1
    # 一代手默认波特率115200, 注意：一代手升级仅支持115200波特率
    # 二代手默认波特率460800
    baurate = libstark.Baudrate.Baud460800 if stark_v2 else libstark.Baudrate.Baud115200
    logger.info(f"slave_id: {slave_id}, baurate: {baurate}")
    logger.info(f"port_name: {port_name}")
    # 打开串口

    client = await libstark.modbus_open(port_name, baurate)

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
