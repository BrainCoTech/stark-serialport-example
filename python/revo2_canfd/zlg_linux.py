import time
import sys
from pathlib import Path
from typing import Optional
from canfd_utils import logger

current_dir = Path(__file__).resolve().parent
parent_dir = current_dir.parent
sys.path.append(str(parent_dir))
from dll.zlgcan.zlgcan_linux import *

DEVICE_TYPE = 33  # USBCANFD
DEVICE_INDEX = 0  # 卡索引

def zlgcan_open():
    """打开 ZLG CAN 设备"""
    try:
        # 打开设备
        open_device(DEVICE_TYPE, DEVICE_INDEX)

        # 启动通道0
        canfd_start(DEVICE_TYPE, DEVICE_INDEX, 0)

    except Exception as e:
        logger.error(f"打开设备失败: {e}")


def zlgcan_close():
    """关闭 ZLG CAN 设备"""
    try:
        close_device(DEVICE_TYPE, DEVICE_INDEX)
    except Exception as e:
        logger.error(f"关闭设备失败: {e}")


def zlgcan_send_message(can_id: int, data: bytes) -> bool:
    """发送 CANFD 消息"""
    try:
        logger.debug(f"发送 CANFD - ID: 0x{can_id:x}, Data: {data.hex()}")
        if not canfd_send(DEVICE_TYPE, DEVICE_INDEX, 0, can_id, data):
            logger.error("发送数据失败!")
            return False
        logger.debug("发送数据成功")
        return True
    except Exception as e:
        logger.error(f"发送消息异常: {e}")
        return False

def zlgcan_receive_message(quick_retries: int = 2, dely_retries: int = 0) -> Optional[tuple[int, list[int]]]:
    """
    接收CAN总线消息，先尝试快速接收，若失败则尝试DFU模式下的慢速接收。

    参数:
        quick_retries (int): 快速接收尝试次数，默认2次。
        dely_retries (int): DFU模式下的慢速接收尝试次数，默认0次。

    返回:
        接收到的消息或None(接收失败)
    """
    # 快速接收尝试
    for _attempt in range(quick_retries):
        time.sleep(0.001)  # 较短延时
        message = _zlgcan_read_messages()
        if message is not None:
            return message

    logger.warning("快速接收超时")

    # 慢速接收尝试
    for _attempt in range(dely_retries):
        time.sleep(2.0)  # 较长延时，适用于DFU模式最后一包数据
        message = _zlgcan_read_messages()
        if message is not None:
            return message

    if dely_retries > 0:
        logger.error("慢速接收尝试也超时!")
    return None


def _zlgcan_read_messages() -> Optional[tuple[int, list[int]]]:
    """
    从 ZLG CAN 设备读取消息，支持多帧响应拼接

    返回:
        (can_id, data) 元组，其中 data 可能包含多帧拼接后的数据
    """
    try:
        # 接收消息
        can_msgs = canfd_receive(DEVICE_TYPE, DEVICE_INDEX, 0)

        if can_msgs is None:
            return None

        # 处理第一条接收消息
        first_msg = can_msgs[0]
        can_id = first_msg.hdr.id
        data = [first_msg.dat[i] for i in range(first_msg.hdr.len)]
        return (can_id, data)

    except Exception as e:
        logger.error(f"读取消息异常: {e}")
        import traceback
        logger.error(traceback.format_exc())

    return None
