
import os
import pathlib
from stark_utils import logger
from zlgcan import *

current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent
logger.info(f"parent_dir: {parent_dir}")

read_timeout_ms = 50

dll_path = os.path.join(parent_dir, "dll", "zlgcan.dll")
zcan = ZCAN(dll_path)
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
    msg.transmit_type = 0  # 0: 正常发送, 1: 单次发送, 2: 自发自收
    msg.frame.eff = 1
    msg.frame.rtr = 0
    msg.frame.can_id = can_id
    msg.frame.brs = 0
    msg.frame.len = 64
    for i in range(msg.frame.len):
        msg.frame.data[i] = data[i]

    ret = zcan.TransmitFD(zcan_handler, msg, 1) # write_num
    if ret != 1:
        logger.error("发送数据失败!")
        return False
    logger.debug(f"发送数据成功: {ret}")
    return True

def zcan_read_messages(retry: int = 2):
    if zcan_handler is None:
        logger.error("CANFD handler is None")
        return None
    
    idx = 0
    while idx < retry:
        time.sleep(0.001)
        msg = _zcan_read_messages(idx)
        if msg is not None:
            return msg
        idx += 1
    logger.error("接收数据失败!")
    return None

def _zcan_read_messages(index: int = 0):
    num = zcan.GetReceiveNum(zcan_handler, ZCAN_TYPE_CANFD)
    if num == 0:
        logger.info(f"No messages received, idx: {index}")
        return None
    # if num != self._write_num:
    #     logger.error(f"接收数据量错误: {num}")
    #     return

    recv_msgs, act_num = zcan.ReceiveFD(zcan_handler, num, read_timeout_ms)
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



