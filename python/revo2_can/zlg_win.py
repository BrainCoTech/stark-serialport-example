import threading
import time
import sys
from pathlib import Path
from typing import Optional
from can_utils import logger

current_dir = Path(__file__).resolve().parent
parent_dir = current_dir.parent
sys.path.append(str(parent_dir))
from dll.zlgcan.zlgcan import *

zcanlib = ZCAN()
device_handle = None
chn_handles = []
# threads = []

thread_flag = True
print_lock = threading.Lock()   # Thread lock, just to avoid printing conflicts
enable_merge_receive = 0        # Merge receive identifier

# Get device information
def get_device_info(device_handle):
    info = zcanlib.GetDeviceInf(device_handle)
    if info is None:
        logger.error("Failed to get device information")
        return None
    logger.info("Device information: \n%s" % info)

    can_number = info.can_num
    return can_number

def zlgcan_open():
    global device_handle
    try:
        # Open device
        device_handle = zcanlib.OpenDevice(ZCAN_USBCANFD_100U, 0, 0)
        if device_handle == INVALID_DEVICE_HANDLE:
            logger.error("Failed to open device!")
            exit(0)
        logger.debug("Device handle: %d." % device_handle)

        # Get device information
        can_number = get_device_info(device_handle)
        if can_number is None:
            logger.error("Failed to get device information")
            return

        # Start channel
        for i in range(can_number):
            chn_handle = start_channel(zcanlib, device_handle, i)
            if chn_handle is None:
                logger.error("Failed to start channel%d!" % i)
                exit(0)
            chn_handles.append(chn_handle)  # Add channel handle to list
            logger.debug("Channel handle: %d." % chn_handle)

    except Exception as e:
        logger.error(str(e))

    # if enable_merge_receive == 1:   #   If merge receive is enabled, all channels are handled by a single receive thread
    #     thread = threading.Thread(target=receive_thread, args=(handle,chn_handles[0]))    # Start independent receive thread
    #     threads.append(thread)
    #     thread.start()
    # else:                           #   If merge receive is not enabled, all channel data needs to receive in a separate thread
    #     for i in range(len(chn_handles)):
    #         thread = threading.Thread(target=receive_thread, args=(handle, chn_handles[i]))  # Start independent receive thread
    #         threads.append(thread)
    #         thread.start()

def zlgcan_close():
    # Close timed send/queue send task
    # Clear_Send_Task(handle, 0)

    # Close receive thread
    # if enable_merge_receive == 1:
    #     threads[0].join()
    # else:
    #     for i in range(len(chn_handles)):
    #         threads[i].join()

    # Close channel
    for i in range(len(chn_handles)):
        ret = zcanlib.ResetCAN(chn_handles[i])
        if ret == 1:
            print(f"Close channel{i} successfully")

    # Close device
    if device_handle is not None:
        ret = zcanlib.CloseDevice(device_handle)
        if ret == 1:
            print("Close device successfully")

def zlgcan_send_message(can_id: int, data: bytes):
    # if chn_handles is empty
    if len(chn_handles) == 0:
        logger.error("Channel handle list is empty")
        return False

    channel_handle = chn_handles[0]

    logger.debug(f"can_id: 0x{can_id:0x} 0b{can_id:029b}")
    logger.debug(f"data: 0x{data.hex()}")

    # Send CAN message
    transmit_can_num = 1
    can_msgs = (ZCAN_Transmit_Data * transmit_can_num)()
    for i in range(transmit_can_num):
        can_msgs[i].transmit_type = 0     # 0-Normal send, 2-Self-send/receive
        can_msgs[i].frame.can_id = can_id     # ID
        # can_msgs[i].frame.can_id |= 1 << 31   # The highest bit (bit31) is the extended frame/standard frame identifier, similarly bit30 is the data frame/remote frame identifier
        # CAN 最大数据长度为 8 字节
        data_len = min(len(data), 8)
        can_msgs[i].frame.can_dlc = data_len # Data length
        # can_msgs[i].frame._pad |= 0x20       # Send echo
        can_msgs[i].frame._res0 = 10         # res0, res1 together represent queue send interval (can be understood as a short 2Byte分开传入)
        can_msgs[i].frame._res1 = 0
        # Copy data
        for j in range(data_len):
            can_msgs[i].frame.data[j] = data[j]
    ret = zcanlib.Transmit(channel_handle, can_msgs, transmit_can_num)
    if ret != transmit_can_num:
        logger.error(f"Send data failed! ret: {ret}")
        return False
    logger.debug(f"Send data successfully: {ret}")
    return True

def zlgcan_receive_message(quick_retries: int = 2, dely_retries: int = 0) -> Optional[tuple[int, list[int]]]:
    """
    Receive CAN bus message, first try quick receive, if failed then try slow receive in DFU mode.

    Args:
        quick_retries (int): Quick receive attempt次数，默认2次。
        dely_retries (int): Slow receive attempt次数 in DFU mode，默认0次。

    Returns:
        Received message or None(receive failed)
    """
    # if chn_handles is empty
    if len(chn_handles) == 0:
        logger.error("Channel handle list is empty")
        return None

    channel_handle = chn_handles[0]

    # Quick receive attempt
    for _attempt in range(quick_retries):
        time.sleep(0.001)  # Short delay
        message = _zlgcan_read_messages(channel_handle)
        if message is not None:
            return message

    logger.warning("Quick receive timeout")

    # Slow receive attempt
    for _attempt in range(dely_retries):
        # time.sleep(0.5)  # Long delay, applicable to DFU mode
        time.sleep(0.005)
        message = _zlgcan_read_messages(channel_handle)
        if message is not None:
            return message

    if dely_retries > 0:
        logger.error("Slow receive attempt also timeout!")
    return None

def _zlgcan_read_messages(channel_handle: int) -> Optional[tuple[int, list[int]]]:
    """
    Read message from ZLG CAN device, support multi-frame response concatenation

    Returns:
        (can_id, data) tuple, where data may contain data from multiple frames concatenated
    """
    # For printing alignment -- no actual function
    CANType_width = len("CANFD加速    ")
    id_width = len(hex(0x1FFFFFFF))

    rcv_can_num = zcanlib.GetReceiveNum(channel_handle, ZCAN_TYPE_CAN)  # CAN
    # print(f"rcv_can_num: {rcv_can_num}")
    if rcv_can_num:
        if rcv_can_num > 100:
            rcv_can_msgs, rcv_can_num = zcanlib.Receive(channel_handle, 100, c_int(100))
        else:
            rcv_can_msgs, rcv_can_num = zcanlib.Receive(channel_handle, rcv_can_num, c_int(100))

        # Print received messages (for debugging)
        with print_lock:
            for msg in rcv_can_msgs[:rcv_can_num]:
                can_type = "CAN   "
                frame = msg.frame
                direction = "TX" if frame._pad & 0x20 else "RX"
                frame_type = "Extended frame" if frame.can_id & (1 << 31) else "Standard frame"
                frame_format = "Remote frame" if frame.can_id & (1 << 30) else "Data frame"
                can_id = hex(frame.can_id & 0x1FFFFFFF)

                if frame.can_id & (1 << 30):
                    data = ""
                    dlc = 0
                else:
                    dlc = frame.can_dlc
                    data = " ".join([f"{num:02X}" for num in frame.data[:dlc]])

                logger.debug(f"[{msg.timestamp}] CAN{channel_handle & 0xFF} {can_type:<{CANType_width}}\t{direction} ID: {can_id:<{id_width}}\t{frame_type} {frame_format}"
                        f" DLC: {dlc}\tDATA(hex): {data}")

        if rcv_can_num >= 1:
            # Find the first received message (skip TX echo)
            start_idx = 0
            for i in range(rcv_can_num):
                # Check if it is TX echo (_pad bit5 is 1 means TX)
                if not (rcv_can_msgs[i].frame._pad & 0x20):  # Non-TX echo
                    start_idx = i
                    break
            else:
                # All messages are TX echo, no valid received data
                logger.debug("All messages are TX echo, no valid received data")
                return None

            # Process the first received message
            recv_msg = rcv_can_msgs[start_idx]
            can_id = recv_msg.frame.can_id
            data = [recv_msg.frame.data[i] for i in range(recv_msg.frame.can_dlc)]

            # Received multiple messages, when can_id is the same, read multiple frames of response for concatenation
            if rcv_can_num > (start_idx + 1):
                logger.debug(f"Received {rcv_can_num - start_idx} valid messages, try to concatenate multiple frames of data")
                for i in range(start_idx + 1, rcv_can_num):
                    msg = rcv_can_msgs[i]
                    # Skip TX echo
                    if msg.frame._pad & 0x20:
                        continue

                    if msg.frame.can_id != can_id:
                        logger.warning(
                            f"Received multiple messages, can_id different: {can_id} != {msg.frame.can_id}"
                        )
                        break
                    # if msg.frame.can_dlc <= 2: # Multiple data frames, length greater than 2
                    if msg.frame.can_dlc != 8:  # Multiple data frames, length not equal to 8
                        logger.warning(
                            f"Received multiple messages, data length not equal to 8: {msg.frame.can_dlc}"
                        )
                        break  # Data length not equal to 8, considered not multi-frame data
                    data.extend([msg.frame.data[j] for j in range(msg.frame.can_dlc)])

                logger.debug(f"Multi-frame concatenation completed, total length: {len(data)} bytes")

            return (can_id, data)

    return None

# Start channel
def start_channel(zcanlib: ZCAN, device_handle: int, chn: int):

    # Set controller device type, if not Non-ISO CANFD, can be commented out according to device default, CAN/CANFD are default
    # ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/canfd_standard", "0".encode("utf-8"))

    # Arbitration domain baud rate and data domain baud rate
    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/canfd_abit_baud_rate", "1000000".encode("utf-8"))
    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/canfd_dbit_baud_rate", "5000000".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d baud failed!" % chn)
        return None

    # Custom baud rate     When the product baud rate has requirements for sampling points, or needs to set non-standard baud rate, use   --- default no matter
    # ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/baud_rate_custom", "500Kbps(80%),2.0Mbps(80%),(80,07C00002,01C00002)".encode("utf-8"))
    # if ret != ZCAN_STATUS_OK:
    #     print("Set CH%d baud failed!" % chn)
    #     return None

    # Terminal resistance
    # ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/initenal_resistance", "1".encode("utf-8"))
    # if ret != ZCAN_STATUS_OK:
    #     print("Open CH%d resistance failed!" % chn)
    #     return None

    # Initialize channel
    chn_init_cfg = ZCAN_CHANNEL_INIT_CONFIG()
    chn_init_cfg.can_type = ZCAN_TYPE_CANFD  # USBCANFD must select CANFD
    chn_init_cfg.config.canfd.mode = 0  # 0-Normal mode 1-Listen only mode
    chn_handle = zcanlib.InitCAN(device_handle, chn, chn_init_cfg)
    if chn_handle is None:
        print("initCAN failed!" % chn)
        return None

    # Set filter
    # Set_Filter(device_handle, chn)

    # Set send echo    USBCANFD series old card (no LIN port, device white label value version is V1.02 or below, including V1.02) needs the following operations, otherwise the CAN message structure _pad structure bit5 identifier, see Transmit_Test send example
    # ret = zcanlib.ZCAN_SetValue(device_handle,str(chn)+"/set_device_tx_echo","0".encode("utf-8"))   #Send echo setting, 0-disable, 1-enable
    # if ret != ZCAN_STATUS_OK:
    #     print("Set CH%d  set_device_tx_echo failed!" %(chn))
    #     return None

    # Enable/disable merge receive (before startCAN)    0-disable 1-enable
    ret = zcanlib.ZCAN_SetValue(device_handle, str(0) + "/set_device_recv_merge", repr(enable_merge_receive))
    if ret != ZCAN_STATUS_OK:
        print("Open CH%d recv merge failed!" % chn)
        return None

    # Start channel
    ret = zcanlib.StartCAN(chn_handle)
    if ret != ZCAN_STATUS_OK:
        print("startCAN failed!" % chn)
        return None

    return chn_handle
