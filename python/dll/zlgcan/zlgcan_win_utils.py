"""
ZLG CAN Windows utilities - shared functions for CAN 2.0 communication.

This module provides common ZLG CAN functions used by both revo1_can and revo2_can examples.
"""
import threading
import time
from ctypes import c_int
from typing import Optional

from .zlgcan import (
    ZCAN, ZCAN_USBCANFD_100U, INVALID_DEVICE_HANDLE, ZCAN_STATUS_OK,
    ZCAN_TYPE_CAN, ZCAN_TYPE_CANFD, ZCAN_CHANNEL_INIT_CONFIG, ZCAN_Transmit_Data
)

zcanlib = ZCAN()
device_handle = None
chn_handles = []

thread_flag = True
print_lock = threading.Lock()
enable_merge_receive = 0

# Logger will be set by the importing module
_logger = None


def set_logger(logger):
    """Set the logger instance to use for this module."""
    global _logger
    _logger = logger


def _log_debug(msg: str):
    if _logger:
        _logger.debug(msg)


def _log_info(msg: str):
    if _logger:
        _logger.info(msg)


def _log_error(msg: str):
    if _logger:
        _logger.error(msg)


def _log_warning(msg: str):
    if _logger:
        _logger.warning(msg)


def get_device_info(dev_handle):
    """Get device information"""
    info = zcanlib.GetDeviceInf(dev_handle)
    if info is None:
        _log_error("Failed to get device information")
        return None
    _log_info("Device information: \n%s" % info)
    return info.can_num


def zlgcan_open():
    """Open ZLG CAN device"""
    global device_handle
    try:
        device_handle = zcanlib.OpenDevice(ZCAN_USBCANFD_100U, 0, 0)
        if device_handle == INVALID_DEVICE_HANDLE:
            _log_error("Failed to open device!")
            exit(0)
        _log_debug("Device handle: %d." % device_handle)

        can_number = get_device_info(device_handle)
        if can_number is None:
            _log_error("Failed to get device information")
            return

        for i in range(can_number):
            chn_handle = _start_channel(zcanlib, device_handle, i)
            if chn_handle is None:
                _log_error("Failed to start channel%d!" % i)
                exit(0)
            chn_handles.append(chn_handle)
            _log_debug("Channel handle: %d." % chn_handle)

    except Exception as e:
        _log_error(str(e))


def zlgcan_close():
    """Close ZLG CAN device"""
    for i in range(len(chn_handles)):
        ret = zcanlib.ResetCAN(chn_handles[i])
        if ret == 1:
            print(f"Close channel{i} successfully")

    if device_handle is not None:
        ret = zcanlib.CloseDevice(device_handle)
        if ret == 1:
            print("Close device successfully")


def zlgcan_send_message(can_id: int, data: bytes) -> bool:
    """Send CAN message"""
    if len(chn_handles) == 0:
        _log_error("Channel handle list is empty")
        return False

    channel_handle = chn_handles[0]

    _log_debug(f"can_id: 0x{can_id:0x} 0b{can_id:029b}")
    _log_debug(f"data: 0x{data.hex()}")

    transmit_can_num = 1
    can_msgs = (ZCAN_Transmit_Data * transmit_can_num)()
    for i in range(transmit_can_num):
        can_msgs[i].transmit_type = 0
        can_msgs[i].frame.can_id = can_id
        data_len = min(len(data), 8)
        can_msgs[i].frame.can_dlc = data_len
        can_msgs[i].frame._res0 = 10
        can_msgs[i].frame._res1 = 0
        for j in range(data_len):
            can_msgs[i].frame.data[j] = data[j]

    ret = zcanlib.Transmit(channel_handle, can_msgs, transmit_can_num)
    if ret != transmit_can_num:
        _log_error(f"Failed to send data! ret: {ret}")
        return False
    _log_debug(f"Send data successfully: {ret}")
    return True


def zlgcan_receive_message(quick_retries: int = 2, dely_retries: int = 0) -> Optional[tuple[int, list[int]]]:
    """
    Receive CAN bus message, first try quick receive, if failed then try slow receive.

    Args:
        quick_retries: Quick receive attempt count, default 2.
        dely_retries: Slow receive attempt count, default 0.

    Returns:
        Received message or None (receive failed)
    """
    if len(chn_handles) == 0:
        _log_error("Channel handle list is empty")
        return None

    channel_handle = chn_handles[0]

    for _attempt in range(quick_retries):
        time.sleep(0.01)
        message = _zlgcan_read_messages(channel_handle)
        if message is not None:
            return message

    _log_warning("Quick receive timeout")

    for _attempt in range(dely_retries):
        time.sleep(0.005)
        message = _zlgcan_read_messages(channel_handle)
        if message is not None:
            return message

    if dely_retries > 0:
        _log_error("Slow receive attempt also timeout!")
    return None


def zlgcan_receive_filtered(expected_can_id: int, expected_frames: int = 1, max_retries: int = 10) -> Optional[tuple[int, list[int], int]]:
    """
    Receive CAN bus message filtered by expected CAN ID with multi-frame protocol support.
    
    Args:
        expected_can_id: Expected CAN ID to filter by
        expected_frames: Expected frame count (hint from SDK)
        max_retries: Maximum retry attempts
        
    Returns:
        (can_id, data, frame_count) tuple or None if timeout
    """
    if len(chn_handles) == 0:
        _log_error("Channel handle list is empty")
        return None

    channel_handle = chn_handles[0]
    
    cmd = (expected_can_id >> 3) & 0x0F
    is_multi_frame_cmd = (cmd == 0x0B or cmd == 0x0D)
    is_dfu_mode = (expected_can_id == 0)
    
    # Determine retry strategy (aligned with SDK):
    # - DFU mode: 200 attempts (for CRC verification)
    # - Multi-frame commands: 5 attempts
    # - Single frame: 2 attempts
    if is_dfu_mode:
        max_retries = 200
    elif expected_frames > 1 or is_multi_frame_cmd:
        max_retries = 5
    else:
        max_retries = 2
    
    all_data = []
    frame_count = 0
    total_frames = 0
    is_multi_frame = False
    
    for attempt in range(max_retries):
        wait_ms = 0.002 if attempt < 5 else 0.005
        time.sleep(wait_ms)
        
        try:
            rcv_can_num = zcanlib.GetReceiveNum(channel_handle, ZCAN_TYPE_CAN)
            if not rcv_can_num:
                continue
                
            if rcv_can_num > 100:
                rcv_can_msgs, rcv_can_num = zcanlib.Receive(channel_handle, 100, c_int(100))
            else:
                rcv_can_msgs, rcv_can_num = zcanlib.Receive(channel_handle, rcv_can_num, c_int(100))
            
            for i in range(rcv_can_num):
                msg = rcv_can_msgs[i]
                if msg.frame._pad & 0x20:  # TX echo
                    continue
                
                frame_id = msg.frame.can_id & 0x1FFFFFFF
                can_dlc = msg.frame.can_dlc
                
                if frame_id != expected_can_id:
                    _log_debug(f"Skipping frame with different CAN ID: 0x{frame_id:x} (expected 0x{expected_can_id:x})")
                    continue
                
                frame_data = [msg.frame.data[j] for j in range(can_dlc)]
                
                if is_multi_frame_cmd and can_dlc > 0:
                    frame_header = frame_data[0]
                    
                    if cmd == 0x0B:
                        if can_dlc >= 2:
                            len_and_flag = frame_data[1]
                            is_last = (len_and_flag & 0x80) != 0
                            all_data.extend(frame_data)
                            frame_count += 1
                            if is_last:
                                return (expected_can_id, all_data, frame_count)
                            continue
                    
                    elif cmd == 0x0D:
                        total = (frame_header >> 4) & 0x0F
                        seq = frame_header & 0x0F
                        if total > 0 and seq > 0:
                            if not is_multi_frame:
                                is_multi_frame = True
                                total_frames = total
                            all_data.extend(frame_data)
                            frame_count += 1
                            if frame_count >= total_frames:
                                return (expected_can_id, all_data, frame_count)
                            continue
                
                all_data.extend(frame_data)
                frame_count += 1
                
                if expected_frames <= 1 and not is_multi_frame_cmd:
                    return (expected_can_id, all_data, frame_count)
                
                if expected_frames > 1 and frame_count >= expected_frames:
                    return (expected_can_id, all_data, frame_count)
                
        except Exception as e:
            _log_error(f"Receive filtered exception: {e}")
    
    if all_data:
        return (expected_can_id, all_data, frame_count)
    
    return None


def _zlgcan_read_messages(channel_handle: int) -> Optional[tuple[int, list[int]]]:
    """
    Read message from ZLG CAN device, support multi-frame response concatenation.

    Returns:
        (can_id, data) tuple
    """
    CANType_width = len("CANFD加速    ")
    id_width = len(hex(0x1FFFFFFF))

    rcv_can_num = zcanlib.GetReceiveNum(channel_handle, ZCAN_TYPE_CAN)
    if rcv_can_num:
        if rcv_can_num > 100:
            rcv_can_msgs, rcv_can_num = zcanlib.Receive(channel_handle, 100, c_int(100))
        else:
            rcv_can_msgs, rcv_can_num = zcanlib.Receive(channel_handle, rcv_can_num, c_int(100))

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

                _log_debug(f"[{msg.timestamp}] CAN{channel_handle & 0xFF} {can_type:<{CANType_width}}\t{direction} ID: {can_id:<{id_width}}\t{frame_type} {frame_format}"
                        f" DLC: {dlc}\tDATA(hex): {data}")

        if rcv_can_num >= 1:
            start_idx = 0
            for i in range(rcv_can_num):
                if not (rcv_can_msgs[i].frame._pad & 0x20):
                    start_idx = i
                    break
            else:
                _log_debug("All messages are TX echo, no valid received data")
                return None

            recv_msg = rcv_can_msgs[start_idx]
            can_id = recv_msg.frame.can_id
            data = [recv_msg.frame.data[i] for i in range(recv_msg.frame.can_dlc)]

            if rcv_can_num > (start_idx + 1):
                _log_debug(f"Received {rcv_can_num - start_idx} valid messages, try merging multiple frames data")
                for i in range(start_idx + 1, rcv_can_num):
                    msg = rcv_can_msgs[i]
                    if msg.frame._pad & 0x20:
                        continue
                    if msg.frame.can_id != can_id:
                        _log_warning(f"Received multiple messages, can_id different: {can_id} != {msg.frame.can_id}")
                        break
                    if msg.frame.can_dlc != 8:
                        _log_warning(f"Received multiple messages, data length not equal to 8: {msg.frame.can_dlc}")
                        break
                    data.extend([msg.frame.data[j] for j in range(msg.frame.can_dlc)])

                _log_debug(f"Multiple frames merged successfully, total length: {len(data)} bytes")

            return (can_id, data)

    return None


def _start_channel(zcanlib_inst: ZCAN, dev_handle: int, chn: int):
    """Start CAN channel"""
    ret = zcanlib_inst.ZCAN_SetValue(dev_handle, str(chn) + "/canfd_abit_baud_rate", "1000000".encode("utf-8"))
    ret = zcanlib_inst.ZCAN_SetValue(dev_handle, str(chn) + "/canfd_dbit_baud_rate", "5000000".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d baud failed!" % chn)
        return None

    chn_init_cfg = ZCAN_CHANNEL_INIT_CONFIG()
    chn_init_cfg.can_type = ZCAN_TYPE_CANFD
    chn_init_cfg.config.canfd.mode = 0
    chn_handle = zcanlib_inst.InitCAN(dev_handle, chn, chn_init_cfg)
    if chn_handle is None:
        print("initCAN failed!" % chn)
        return None

    ret = zcanlib_inst.ZCAN_SetValue(dev_handle, str(0) + "/set_device_recv_merge", repr(enable_merge_receive))
    if ret != ZCAN_STATUS_OK:
        print("Open CH%d recv merge failed!" % chn)
        return None

    ret = zcanlib_inst.StartCAN(chn_handle)
    if ret != ZCAN_STATUS_OK:
        print("startCAN failed!" % chn)
        return None

    return chn_handle
