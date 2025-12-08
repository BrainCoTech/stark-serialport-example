import threading
import time
import sys
from pathlib import Path
from typing import Optional
from canfd_utils import logger

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

    # if enable_merge_receive == 1:   #   If merge receive is enabled, all channels are handled by a receive thread
    #     thread = threading.Thread(target=receive_thread, args=(handle,chn_handles[0]))    # Start independent receive thread
    #     threads.append(thread)
    #     thread.start()
    # else:                           #   If merge receive is not enabled, all channel data needs to receive by a separate thread
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

def zlgcan_send_message(can_id: int, data: bytes) -> bool:
    # if chn_handles is empty
    if len(chn_handles) == 0:
        logger.error("Channel handle list is empty")
        return False

    channel_handle = chn_handles[0]

    logger.debug(f"CAN ID: 0x{can_id:02X}")
    logger.debug(f"Data: {data.hex()}")

    # Send CANFD message
    transmit_canfd_num = 1
    canfd_msgs = (ZCAN_TransmitFD_Data * transmit_canfd_num)()
    for i in range(transmit_canfd_num):
        canfd_msgs[i].transmit_type = 0     # 0-normal send, 2-self-send-receive
        canfd_msgs[i].frame.can_id = can_id     # ID
        canfd_msgs[i].frame.can_id |= 1 << 31   # Extended frame flag (bit 31)
        # CANFD maximum data length is 64 bytes
        data_len = min(len(data), 64)
        canfd_msgs[i].frame.len = data_len     # Data length
        canfd_msgs[i].frame.flags = 0         # Clear first
        # canfd_msgs[i].frame.flags |= 0x20    # bit 5: Send echo flag
        canfd_msgs[i].frame.flags |= 0x1     # bit 0: BRS acceleration flag: 0 no acceleration, 1 acceleration
        canfd_msgs[i].frame._res0 = 10      # Reserved field
        # Copy data
        for j in range(data_len):
            canfd_msgs[i].frame.data[j] = data[j]

    ret = zcanlib.TransmitFD(channel_handle, canfd_msgs, transmit_canfd_num)
    if ret != transmit_canfd_num:
        logger.error("Failed to send data!")
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
            can_id = message.frame.can_id
            data = [message.frame.data[i] for i in range(message.frame.len)]
            return (can_id, data)

    logger.warning("Quick receive timeout")

    # Slow receive attempt
    for _attempt in range(dely_retries):
        # time.sleep(0.5)  # Long delay, suitable for DFU mode
        time.sleep(0.005)
        message = _zlgcan_read_messages(channel_handle)
        if message is not None:
            can_id = message.frame.can_id
            data = [message.frame.data[i] for i in range(message.frame.len)]
            return (can_id, data)

    if dely_retries > 0:
        logger.error("Slow receive attempt also timeout!")
    return None

def _zlgcan_read_messages(channel_handle: int) -> Optional[ZCAN_ReceiveFD_Data]:
    """
    Read message from ZLG CANFD device, skip TX echo

    Returns:
        Last valid received message (skip TX echo)
    """
    # For printing alignment -- no actual function
    CANType_width = len("CANFD acceleration    ")
    id_width = len(hex(0x1FFFFFFF))

    rcv_canfd_num = zcanlib.GetReceiveNum(channel_handle, ZCAN_TYPE_CANFD)  # CANFD
    if rcv_canfd_num:
        if rcv_canfd_num > 100:
            rcv_canfd_msgs, rcv_canfd_num = zcanlib.ReceiveFD(channel_handle, 100, c_int(100))
        else:
            rcv_canfd_msgs, rcv_canfd_num = zcanlib.ReceiveFD(channel_handle, rcv_canfd_num, c_int(100))

        # Print received messages (for debugging)
        with print_lock:
            for msg in rcv_canfd_msgs[:rcv_canfd_num]:
                frame = msg.frame
                brs = "acceleration" if frame.flags & 0x1 else "   "
                can_type = "CANFD" + brs
                direction = "TX" if frame.flags & 0x20 else "RX"
                frame_type = "Extended frame" if frame.can_id & (1 << 31) else "Standard frame"
                frame_format = "Remote frame" if frame.can_id & (1 << 30) else "Data frame"  # CANFD has no remote frame
                can_id = hex(frame.can_id & 0x1FFFFFFF)
                data = " ".join([f"{num:02X}" for num in frame.data[:frame.len]])

                print(f"[{msg.timestamp}] CAN{channel_handle & 0xFF} {can_type:<{CANType_width}}\t{direction} ID: {can_id:<{id_width}}\t{frame_type} {frame_format}"
                      f" DLC: {frame.len}\tDATA(hex): {data}")

        # Find the last received message (skip TX echo)
        last_rx_msg = None
        for i in range(rcv_canfd_num - 1, -1, -1):  # Search from the end to the beginning
            # Check if it is a TX echo (flags bit5 is 1 means TX)
            if not (rcv_canfd_msgs[i].frame.flags & 0x20):  # Not TX echo
                last_rx_msg = rcv_canfd_msgs[i]
                break

        if last_rx_msg is None:
            # All messages are TX echo, no valid received data
            logger.debug("All messages are TX echo, no valid received data")
            return None

        return last_rx_msg

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

    # Custom baud rate     When the product baud rate has requirements for sampling points, or needs to set non-standard baud rates, use --- default no matter what
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
    chn_init_cfg.config.canfd.mode = 0  # 0-normal mode 1-listen only mode
    chn_handle = zcanlib.InitCAN(device_handle, chn, chn_init_cfg)
    if chn_handle is None:
        print("initCAN failed!" % chn)
        return None

    # Set filter
    # Set_Filter(device_handle, chn)

    # Set send echo    USBCANFD series old card (no LIN port, device white label value version is V1.02 or below, including V1.02) needs the following operation, otherwise the CAN message structure _pad structure bit5 identifier, see Transmit_Test send example
    # ret = zcanlib.ZCAN_SetValue(device_handle,str(chn)+"/set_device_tx_echo","0".encode("utf-8"))   #Send echo setting, 0-disable, 1-enable
    # if ret != ZCAN_STATUS_OK:
    #     print("Set CH%d  set_device_tx_echo failed!" %(chn))
    #     return None

    # Enable/disable merge receive(before startCAN)    0-disable 1-enable
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
