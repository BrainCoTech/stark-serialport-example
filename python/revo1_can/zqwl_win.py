import sys
import os
from pathlib import Path

current_dir = Path(__file__).resolve().parent
parent_dir = current_dir.parent
sys.path.append(str(parent_dir))

from dll.zqwl.zqwl import *
from can_utils import logger

logger.info(f"parent_dir: {parent_dir}")

read_timeout_ms = 50

zcan = ZCAN()
zcan_handler = None
zcan_device_handler = None


def zcan_open(device_type: int, channel: int, baudrate: int):
    """
    打开标准 CAN 2.0 通道

    参数:
        device_type: 设备类型
        channel: 通道号
        baudrate: 波特率，默认 1Mbps (支持 125K, 250K, 500K, 1M)
    """
    try:
        # open device
        dev_handler = zcan.OpenDevice(ZCAN_DEVICE_TYPE(device_type), 0, 0)
        if dev_handler == INVALID_DEVICE_HANDLE:
            logger.error("打开设备失败")
            return

        # 配置标准 CAN 2.0 参数
        set_can_baudrate(dev_handler, channel, baudrate)
        zcan.ZCAN_SetValue(dev_handler, f"{channel}/filter_clear", "0")  # 清除滤波器
        # zcan.ZCAN_SetValue(dev_handler, f"{channel}/filter_mode", "0")               # 设置滤波模式
        # zcan.ZCAN_SetValue(dev_handler, f"{channel}/acc_code", "0")                  # 验收码
        # zcan.ZCAN_SetValue(dev_handler, f"{channel}/acc_mask", "0xffffffff")         # 验收屏蔽码
        # zcan.ZCAN_SetValue(dev_handler, f"{channel}/brp", "1")                       # 波特率预分频

        # 初始化通道配置
        chn_cfg = ZCAN_CHANNEL_INIT_CONFIG()
        chn_cfg.can_type = ZCAN_TYPE_CAN  # 标准 CAN 2.0

        # 配置标准 CAN 参数
        # chn_cfg.config.can.acc_code = 0x00000000      # 验收码
        # chn_cfg.config.can.acc_mask = 0xffffffff      # 验收屏蔽码 (接收所有帧)
        # chn_cfg.config.can.filter = 0                 # 滤波器设置
        chn_cfg.config.can.mode = 0  # 0: 正常模式, 1: 只听模式

        can_handler = zcan.InitCAN(dev_handler, channel, chn_cfg)
        if can_handler == INVALID_CHANNEL_HANDLE:
            logger.error("初始化通道失败!")
            zcan.CloseDevice(dev_handler)
            return

        ret = zcan.StartCAN(can_handler)
        if ret != ZCAN_STATUS_OK:
            logger.error("打开通道失败!")
            return

        global zcan_handler, zcan_device_handler
        zcan_handler = can_handler
        zcan_device_handler = dev_handler

        logger.info(f"CAN 2.0 通道已成功开启: 波特率={baudrate}bps, 采样点=80%")

    except Exception as e:
        logger.error(f"CAN初始化异常: {str(e)}")


def zcan_close():
    """关闭CAN通道和设备"""
    global zcan_handler, zcan_device_handler
    if zcan_handler is not None:
        zcan.ResetCAN(zcan_handler)
        zcan_handler = None

    if zcan_device_handler is not None:
        zcan.CloseDevice(zcan_device_handler)
        zcan_device_handler = None

    logger.info("CAN设备已关闭")


def zcan_clear_buffer():
    """清空接收缓冲区"""
    if zcan_handler is not None:
        zcan.ClearBuffer(zcan_handler)


def zcan_send_message(slave_id: int, can_id: int, data: bytes):
    """
    发送标准CAN消息

    参数:
        slave_id: 从站ID (未使用，保持兼容性)
        can_id: CAN标识符 (11-bit 标准帧)
        data: 数据 (最大8字节)
    """
    if zcan_handler is None:
        logger.error("CAN handler is None")
        return False

    # 检查数据长度
    if len(data) > 8:
        logger.error(f"CAN 2.0 数据长度超限: {len(data)}字节 (最大8字节)")
        return False

    # 检查CAN ID范围 (11-bit 标准帧)
    if can_id > 0x7FF:
        logger.warning(f"CAN ID {can_id:03X} 超出标准帧范围，将使用扩展帧")

    logger.debug(f"发送CAN消息 - ID: 0x{can_id:03X}, 数据: {data.hex()}")

    # 创建标准CAN消息
    msg = ZCAN_Transmit_Data()
    msg.transmit_type = 0  # 0: 正常发送
    msg.frame.can_id = can_id  # CAN标识符
    msg.frame.can_dlc = len(data)  # 数据长度

    # 填充数据
    for i in range(len(data)):
        msg.frame.data[i] = data[i]

    # 发送消息
    ret = zcan.Transmit(zcan_handler, msg, 1)
    if ret != 1:
        logger.error("发送CAN消息失败!")
        return False

    logger.debug(f"发送数据成功: {ret}")
    return True


def zqwl_can_receive_message(quick_retries: int = 2, dely_retries: int = 0):
    """
    接收CAN总线消息，先尝试快速接收，若失败则尝试DFU模式下的慢速接收。

    参数:
        quick_retries (int): 快速接收尝试次数，默认2次。
        dely_retries (int): DFU模式下的慢速接收尝试次数，默认0次。

    返回:
        接收到的消息或None(接收失败)
    """
    if zcan_handler is None:
        logger.error("CANFD handler is None")
        return None

    # 快速接收尝试
    for attempt in range(quick_retries):
        # time.sleep(0.0001)  # 极短延时
        time.sleep(0.02)  # 极短延时
        message = _zcan_read_messages(attempt)
        if message is not None:
            return message

    logger.warning("快速接收超时")

    # 慢速接收尝试
    for attempt in range(dely_retries):
        time.sleep(0.5)  # 较长延时，适用于DFU模式
        message = _zcan_read_messages(attempt)
        if message is not None:
            return message

    if dely_retries > 0:
        logger.error("延时接收也超时!")
    return None


def _zcan_read_messages(index: int = 0):
    """内部函数: 读取CAN消息"""
    num = zcan.GetReceiveNum(zcan_handler, ZCAN_TYPE_CAN)  # 改为标准CAN类型
    if num == 0:
        logger.debug(f"未收到消息, 尝试次数: {index}")
        return None

    # 接收标准CAN消息
    recv_msgs, act_num = zcan.Receive(zcan_handler, num, c_int(read_timeout_ms))
    if act_num == 0:
        logger.error("接收CAN消息失败!")
        return None

    logger.debug(f"接收到 {act_num} 条CAN消息")
    debug_text = "接收数据: "
    for i in range(act_num):
        msg = recv_msgs[i]
        can_id = msg.frame.can_id
        len = msg.frame.can_dlc
        debug_text += (
            "\nline[%d], timestamp=%d, can_id=%s, err=%d, rtr=%d, eff=%d, len=%d, data: %s"
            % (
                i,
                msg.timestamp,
                "{:011b}".format(can_id),
                msg.frame.err,
                msg.frame.rtr,
                msg.frame.eff,
                len,
                " ".join("%02x" % msg.frame.data[j] for j in range(len)),
            )
        )
    logger.debug(f"Received message: %s" % debug_text)

    # 收到多条消息时，返回第一条消息
    if act_num > 0:
        recv_msg = recv_msgs[0]
        can_id = recv_msg.frame.can_id
        data = [recv_msg.frame.data[i] for i in range(recv_msg.frame.can_dlc)]

        if act_num > 1:
            # 接收到多条消息, can_id 相同时，读取到多帧响应进行拼接
            for i in range(act_num - 1):
                msg = recv_msgs[i + 1]
                if msg.frame.can_id != can_id:
                    logger.warning(
                        f"接收到多条消息, can_id 不同: {can_id} != {msg.frame.can_id}"
                    )
                    break
                # if msg.frame.can_dlc <= 2: # 多数据帧，长度大于2
                if msg.frame.can_dlc != 8:  # 多数据帧，长度不等于8
                    logger.warning(
                        f"接收到多条消息, data 长度小于等于2: {msg.frame.can_dlc}"
                    )
                    break  # 数据长度小于等于2，认为是单条消息
                data.extend([msg.frame.data[j] for j in range(msg.frame.can_dlc)])

        return (can_id, data)

    return None


# 波特率预设配置
# CAN 的一个 bit 时间 = Sync + TSEG1 + TSEG2
# timing1 = 0x1C（TSEG1 = 12，TSEG2 = 7）→ 采样点靠后，容错性好，适用于低速率（如 125Kbps～500Kbps）
# timing1 = 0x14（TSEG1 = 4，TSEG2 = 5） → 时隙短，采样点靠中间，适用于高速率（如 1Mbps）
# 采样点 % = (1 + TSEG1) / (1 + TSEG1 + TSEG2) × 100%
# 采样点为80%, 应该使用 0x1C（即 TSEG1 = 12, TSEG2 = 3）
CAN_BAUDRATE_CONFIGS = {
    125000: {"timing0": 0x03, "timing1": 0x1C},
    250000: {"timing0": 0x01, "timing1": 0x1C},
    500000: {"timing0": 0x00, "timing1": 0x1C},
    1000000: {"timing0": 0x00, "timing1": 0x14},
}


def set_can_baudrate(device_handler, channel: int, baudrate: int) -> bool:
    """
    设置 CAN 波特率。

    参数:
        device_handler: 设备句柄
        channel: 通道号（如 0、1）
        baudrate: 波特率（仅支持 125K, 250K, 500K, 1M）

    返回:
        True 表示设置成功，False 表示失败
    """
    config = CAN_BAUDRATE_CONFIGS.get(baudrate)
    if not config:
        logger.error(f"不支持的 CAN 波特率: {baudrate}")
        return False

    try:
        zcan.ZCAN_SetValue(device_handler, f"{channel}/baud_rate", str(baudrate))
        zcan.ZCAN_SetValue(
            device_handler, f"{channel}/abit_timing0", f"0x{config['timing0']:02X}"
        )
        zcan.ZCAN_SetValue(
            device_handler, f"{channel}/abit_timing1", f"0x{config['timing1']:02X}"
        )
        logger.info(f"CAN波特率已设置为: {baudrate} bps")
        return True
    except Exception as e:
        logger.exception(f"设置 CAN 波特率失败: {e}")
        return False
