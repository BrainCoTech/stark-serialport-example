import asyncio
import sys
import pathlib
import os
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
    global shutdown_event
    shutdown_event = setup_shutdown_event(logger)

    (protocol, port_name, baudrate, slave_id) = await libstark.auto_detect_device()
    logger.info(f"Detected protocol: {protocol}, port: {port_name}, baudrate: {baudrate}, slave_id: {slave_id}")
    client = await libstark.modbus_open(port_name, baudrate)

    ota_bin_path = ""
    # 重要！！！：不同硬件需使用相应固件，否则需要拆设备重新烧录
    if protocol == libstark.StarkProtocolType.Modbus:
      device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
      logger.info(f"Device info: {device_info.description}")
      if device_info.is_revo2():
          # ota_bin_path = revo2_basic_ota_bin_path
          if device_info.is_revo2_touch():
              logger.error("Revo2 Touch firmware is not supported in this script.")
              sys.exit(1)
          else:
              ota_bin_path = revo2_basic_ota_bin_path
    else:
        logger.error(f"Unsupported protocol: {protocol}")
        sys.exit(1)

    if not os.path.exists(ota_bin_path):
        logger.warning(f"OTA文件不存在: {ota_bin_path}")
        exit(0)
    else:
        logger.info(f"OTA文件路径: {ota_bin_path}")

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
