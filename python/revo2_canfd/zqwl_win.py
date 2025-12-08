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
            logger.error("Failed to open device")
            return

        # open channel
        zcan.ZCAN_SetValue(dev_handler, "0/canfd_standard", "0")
        zcan.ZCAN_SetValue(dev_handler, "0/canfd_abit_baud_rate", "1000000")  # Set arbitration domain baud rate
        zcan.ZCAN_SetValue(dev_handler, "0/canfd_dbit_baud_rate", f"{baudrate}")  # Set data domain baud rate
        zcan.ZCAN_SetValue(dev_handler, "0/filter_clear", "0") # Clear filter

        chn_cfg = ZCAN_CHANNEL_INIT_CONFIG()
        chn_cfg.can_type = ZCAN_TYPE_CANFD
        # chn_cfg.config.canfd.filter = 0
        # chn_cfg.config.canfd.acc_code = 0
        # chn_cfg.config.canfd.acc_mask = 0xffffffff
        chn_cfg.config.canfd.mode = 0  # 0: Normal mode, 1: Listen-only mode

        can_handler = zcan.InitCAN(dev_handler, channel, chn_cfg)
        if can_handler == INVALID_CHANNEL_HANDLE:
            logger.error("Failed to initialize channel")
            zcan.CloseDevice(dev_handler)
            return

        ret = zcan.StartCAN(can_handler)
        if ret != ZCAN_STATUS_OK:
            logger.error("Failed to start channel")
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
    msg.transmit_type = 0  # 0: Normal transmission, 1: Single transmission, 2: Self-reception, 3: Single self-reception
    msg.frame.eff = 1 # is extended frame
    msg.frame.rtr = 0 # is remote frame
    msg.frame.can_id = can_id
    msg.frame.brs = 1 # is BRS frame, Use higher baud rate to send data
    msg.frame.len = len(data) # The firmware uses the CANfd extended frame
    for i in range(msg.frame.len):
        msg.frame.data[i] = data[i]

    ret = zcan.TransmitFD(zcan_handler, msg, 1) # write_num=1
    if ret != 1:
        logger.error("Failed to send data")
        return False
    logger.debug(f"Send data successfully: {ret}")
    return True

def zcan_receive_message(quick_retries: int = 2, dely_retries: int = 0):
    """
    Receive CAN bus messages, first try quick reception, if failed, try slow reception in DFU mode.

    Parameters:
        quick_retries (int): Number of quick reception attempts, default is 2.
        dely_retries (int): Number of slow reception attempts in DFU mode, default is 0.

    Returns:
        Received message or None (receive failed)
    """
    if zcan_handler is None:
        logger.error("CANFD handler is None")
        return None

    # Quick reception attempts
    for attempt in range(quick_retries):
        time.sleep(0.001)  # Short delay
        message = _zcan_read_messages(attempt)
        if message is not None:
            return message

    logger.warning("Quick reception timeout")

    # Slow reception attempt
    for attempt in range(dely_retries):
        time.sleep(0.5)  # Longer delay, suitable for the last packet of data in DFU mode
        message = _zcan_read_messages(attempt)
        if message is not None:
            return message

    if dely_retries > 0:
        logger.error("Slow reception timeout!")
    return None

def _zcan_read_messages(index: int = 0):
    num = zcan.GetReceiveNum(zcan_handler, ZCAN_TYPE_CANFD)
    if num == 0:
        logger.info(f"No messages received, idx: {index}")
        return None

    recv_msgs, act_num = zcan.ReceiveFD(zcan_handler, num, c_int(read_timeout_ms))
    if act_num == 0:
        logger.error("Failed to receive data")
        return None

    logger.debug(f"Received data count: {act_num}")
    debug_text = "CANFD received data: "
    for i in range(act_num):
        debug_text += ("\nline[%d], timestamp=%d, can_id=%s, len=%d, eff=%d, rtr=%d, esi=%d, brs=%d, data: %s" % (
                    i, recv_msgs[i].timestamp, "{:029b}".format(recv_msgs[i].frame.can_id), recv_msgs[i].frame.len,
                    recv_msgs[i].frame.eff, recv_msgs[i].frame.rtr,
                    recv_msgs[i].frame.esi, recv_msgs[i].frame.brs,
                    ' '.join('%02x' % recv_msgs[i].frame.data[j] for j in range(recv_msgs[i].frame.len))))
    logger.debug(f"Received message: %s" % debug_text)

    return recv_msgs[0]



