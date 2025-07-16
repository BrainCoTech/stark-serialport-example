import asyncio
import sys
import pathlib
import os
from utils import setup_shutdown_event
from revo1_utils import libstark, logger

# 重要！！！：不同硬件需使用相应固件，否则需要拆设备重新烧录
# 固件升级文件路径
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
logger.info(f"parent_dir: {parent_dir}")

# Modbus固件，一代灵巧手触觉版
revo1_touch_ota_bin_path = os.path.join(
  parent_dir,
  "ota_bin",
  "touch",
  "FW_MotorController_Release_SecureOTA_V1.8.32.F.ota",
)
# Modbus固件, 一代灵巧手基础版
# https://app.brainco.cn/universal/bc-stark-sdk/firmware/modbus/FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota
revo1_basic_ota_bin_path = os.path.join(
  parent_dir,
  "ota_bin",
  "modbus",
  "FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota",
)
# Modbus固件, 一代灵巧手Protobuf版
revo1_protobuf_ota_bin_path = os.path.join(
  parent_dir,
  "ota_bin",
  "protobuf",
  "FW_MotorController_Release_SecureOTA_485_9.2.7.ota",
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
      if device_info.is_revo1():
          if device_info.is_revo1_touch():
              ota_bin_path = revo1_touch_ota_bin_path
          else:
              ota_bin_path = revo1_basic_ota_bin_path
              # ota_bin_path = revo1_protobuf_ota_bin_path
    elif protocol == libstark.StarkProtocolType.Protobuf:
        ota_bin_path = revo1_basic_ota_bin_path


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
