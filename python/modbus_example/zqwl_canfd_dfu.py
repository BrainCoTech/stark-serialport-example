import asyncio
import sys
import pathlib
import os
from utils import setup_shutdown_event
import asyncio
import bc_stark_sdk
from stark_utils import libstark, logger
# from zlgcan import ZCAN_USBCANFD_100U
from zqwl_win import zcan_open, zcan_close, zcan_send_message, zcan_receive_message

# 固件升级文件路径
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent
logger.info(f"parent_dir: {parent_dir}")

# ModbusV2固件
ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "stark2",
    # "stark2_fw_V0.0.7_20250403131952.bin",
    "stark2_fw_V0.0.7_20250409094916.bin",
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

def canfd_send(slave_id: int, can_id: int, data: list):
    # logger.debug(f"Sending CAN ID: {can_id}, Data: {data}, type: {type(data)}")
    if not zcan_send_message(slave_id, can_id, bytes(data)):
        logger.error("Failed to send CANFD message")
        return

def canfd_read(slave_id: int):
    recv_msg = zcan_receive_message(dely_retries=10)
    if recv_msg is None:
        logger.error("No message received")
        return 0, bytes([])

    can_id = recv_msg.frame.can_id
    data = [recv_msg.frame.data[i] for i in range(recv_msg.frame.len)]
    logger.debug(f"Received CAN ID: {can_id:029b}, Data: {bytes(data).hex()}")
    return can_id, data

# Main
async def main():
    global shutdown_event
    shutdown_event = setup_shutdown_event(logger)

    libstark.init_config(libstark.StarkFirmwareType.V2Basic, libstark.StarkProtocolType.CanFd)
    slave_id = 0x7e # 左手默认ID为0x7e，右手默认ID为0x7f
    client = await libstark.canfd_open(libstark.BaudrateCAN.Baud5Mbps, slave_id)

    # ZCAN_USBCANFD_100U
    zcan_open(device_type=42, channel=0, baudrate=5000000)
    bc_stark_sdk.set_canfd_tx_callback(canfd_send)
    bc_stark_sdk.set_canfd_rx_callback(canfd_read)

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD, Baudrate: {baudrate}")

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
    zcan_close()
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
