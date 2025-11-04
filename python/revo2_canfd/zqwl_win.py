import sys
import os
from pathlib import Path

current_dir = Path(__file__).resolve().parent
parent_dir = current_dir.parent
sys.path.append(str(parent_dir))

from dll.zqwl.zqwl import *
from canfd_utils import logger
logger.info(f"parent_dir: {parent_dir}")

read_timeout_ms = 50

zcan = ZCAN()
zcan_handler = None
zcan_device_handler = None

def zcan_open(device_type: int, channel: int, baudrate: int):
    try:
        # open device
        dev_handler = zcan.OpenDevice(ZCAN_DEVICE_TYPE(device_type), 0, 0)
        if dev_handler == INVALID_DEVICE_HANDLE:
            logger.error("打开设备失败")
            return

        # open channel
        zcan.ZCAN_SetValue(dev_handler, "0/canfd_standard", "0")
        zcan.ZCAN_SetValue(dev_handler, "0/canfd_abit_baud_rate", "5000000")  # 设置仲裁域波特率, TODO: check 1M
        zcan.ZCAN_SetValue(dev_handler, "0/canfd_dbit_baud_rate", f"{baudrate}")  # 设置数据域波特率
        zcan.ZCAN_SetValue(dev_handler, "0/filter_clear", "0") # 清除滤波

        chn_cfg = ZCAN_CHANNEL_INIT_CONFIG()
        chn_cfg.can_type = ZCAN_TYPE_CANFD
        # chn_cfg.config.canfd.filter = 0
        # chn_cfg.config.canfd.acc_code = 0
        # chn_cfg.config.canfd.acc_mask = 0xffffffff
        chn_cfg.config.canfd.mode = 0  # 0: 正常模式, 1: 只听模式

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

    except Exception as e:
        logger.error(str(e))

def zcan_close():
    global zcan_handler, zcan_device_handler
    if zcan_handler is not None:
        zcan.ResetCAN(zcan_handler)
        zcan_handler = None

    if zcan_device_handler is not None:
        zcan.CloseDevice(zcan_device_handler)
        zcan_device_handler = None

def zcan_clear_buffer():
    if zcan_handler is not None:
        zcan.ClearBuffer(zcan_handler)

def zcan_send_message(slave_id: int, can_id: int, data: bytes):
    if zcan_handler is None:
        logger.error("CANFD handler is None")
        return False

    logger.debug(f"can_id: {can_id:029b}")
    logger.debug(f"data: {data.hex()}")

    msg = ZCAN_TransmitFD_Data()
    msg.transmit_type = 0  # 0: 正常发送, 1: 单次发送, 2: 自发自收 3: 单次自发自收
    msg.frame.eff = 1 # is extended frame
    msg.frame.rtr = 0 # is remote frame
    msg.frame.can_id = can_id
    msg.frame.brs = 1 # is BRS frame, 使用更高的波特率发送数据
    msg.frame.len = len(data) # 固件使用CANfd扩展帧
    for i in range(msg.frame.len):
        msg.frame.data[i] = data[i]

    ret = zcan.TransmitFD(zcan_handler, msg, 1) # write_num=1
    if ret != 1:
        logger.error("发送数据失败!")
        return False
    logger.debug(f"发送数据成功: {ret}")
    return True

def zcan_receive_message(quick_retries: int = 2, dely_retries: int = 0):
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
        time.sleep(0.001)  # 较短延时
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
        logger.error("慢速接收尝试也超时!")
    return None

def _zcan_read_messages(index: int = 0):
    num = zcan.GetReceiveNum(zcan_handler, ZCAN_TYPE_CANFD)
    if num == 0:
        logger.info(f"No messages received, idx: {index}")
        return None
    # if num != self._write_num:
    #     logger.error(f"接收数据量错误: {num}")
    #     return

    recv_msgs, act_num = zcan.ReceiveFD(zcan_handler, num, c_int(read_timeout_ms))
    if act_num == 0:
        logger.error("接收数据失败!")
        return None

    logger.debug(f"接收数据量: {act_num}")
    debug_text = "CANFD接收数据: "
    for i in range(act_num):
        debug_text += ("\nline[%d], timestamp=%d, can_id=%s, len=%d, eff=%d, rtr=%d, esi=%d, brs=%d, data: %s" % (
                    i, recv_msgs[i].timestamp, "{:029b}".format(recv_msgs[i].frame.can_id), recv_msgs[i].frame.len,
                    recv_msgs[i].frame.eff, recv_msgs[i].frame.rtr,
                    recv_msgs[i].frame.esi, recv_msgs[i].frame.brs,
                    ' '.join('%02x' % recv_msgs[i].frame.data[j] for j in range(recv_msgs[i].frame.len))))
    logger.debug(f"Received message: %s" % debug_text)

    return recv_msgs[0]



