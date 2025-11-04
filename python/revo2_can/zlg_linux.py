import time
import sys
from pathlib import Path
from typing import Optional
from can_utils import logger

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
    """发送 CAN 消息"""
    try:
        logger.debug(f"发送 CAN - ID: 0x{can_id:x}, Data: {data.hex()}")
        if not can_send(DEVICE_TYPE, DEVICE_INDEX, 0, can_id, data):
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
        can_msgs = can_receive(DEVICE_TYPE, DEVICE_INDEX, 0)

        if can_msgs is None:
            return None

        start_idx = 0
        rcount = len(can_msgs)

        # 处理第一条接收消息
        first_msg = can_msgs[start_idx]
        can_id = first_msg.hdr.id
        data = [first_msg.dat[i] for i in range(first_msg.hdr.len)]

        # 接收到多条消息, can_id 相同时，读取到多帧响应进行拼接
        if rcount > (start_idx + 1):
            # logger.debug(f"接收到 {rcount - start_idx} 条有效消息，尝试拼接多帧数据")
            for i in range(start_idx + 1, rcount):
                msg = can_msgs[i]
                # 跳过 TX 回显
                if msg.hdr.inf.tx == 1:
                    continue

                if msg.hdr.id != can_id:
                    logger.warning(
                        f"接收到多条消息, can_id 不同: 0x{can_id:x} != 0x{msg.hdr.id:x}"
                    )
                    break

                # 多数据帧，长度应该等于8
                if msg.hdr.len != 8:
                    logger.warning(
                        f"接收到多条消息, data 长度不等于8: {msg.hdr.len}"
                    )
                    break

                # 拼接数据
                data.extend([msg.dat[j] for j in range(msg.hdr.len)])

            # logger.debug(f"多帧拼接完成，总长度: {len(data)} 字节")

        return (can_id, data)

    except Exception as e:
        logger.error(f"读取消息异常: {e}")
        import traceback
        logger.error(traceback.format_exc())

    return None
