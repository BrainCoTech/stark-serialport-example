"""
ZLG CAN Linux utilities - shared functions for CAN 2.0 communication.

This module provides common ZLG CAN functions used by both revo1_can and revo2_can examples.
"""
import time
from typing import Optional

from .zlgcan_linux import open_device, close_device, canfd_start, can_send, can_receive

DEVICE_TYPE = 33  # USBCANFD
DEVICE_INDEX = 0  # Card index

# Logger will be set by the importing module
_logger = None


def set_logger(logger):
    """Set the logger instance to use for this module."""
    global _logger
    _logger = logger


def _log_debug(msg: str):
    if _logger:
        _logger.debug(msg)


def _log_error(msg: str):
    if _logger:
        _logger.error(msg)


def _log_warning(msg: str):
    if _logger:
        _logger.warning(msg)


def zlgcan_open():
    """Open ZLG CAN device"""
    try:
        open_device(DEVICE_TYPE, DEVICE_INDEX)
        canfd_start(DEVICE_TYPE, DEVICE_INDEX, 0)
    except Exception as e:
        _log_error(f"Failed to open device: {e}")


def zlgcan_close():
    """Close ZLG CAN device"""
    try:
        close_device(DEVICE_TYPE, DEVICE_INDEX)
    except Exception as e:
        _log_error(f"Failed to close device: {e}")


def zlgcan_send_message(can_id: int, data: bytes) -> bool:
    """Send CAN message"""
    try:
        _log_debug(f"Send CAN - ID: 0x{can_id:x}, Data: {data.hex()}")
        if not can_send(DEVICE_TYPE, DEVICE_INDEX, 0, can_id, data):
            _log_error("Failed to send data!")
            return False
        _log_debug("Send data successfully")
        return True
    except Exception as e:
        _log_error(f"Send message exception: {e}")
        return False


def zlgcan_receive_message(quick_retries: int = 2, dely_retries: int = 0) -> Optional[tuple[int, list[int]]]:
    """
    Receive CAN bus message, first try quick receive, if failed then try slow receive in DFU mode.

    Args:
        quick_retries: Quick receive attempt count, default 2.
        dely_retries: Slow receive attempt count in DFU mode, default 0.

    Returns:
        Received message or None (receive failed)
    """
    for _attempt in range(quick_retries):
        time.sleep(0.001)
        message = _zlgcan_read_messages()
        if message is not None:
            return message

    _log_debug("Quick receive timeout")

    for _attempt in range(dely_retries):
        time.sleep(2.0)
        message = _zlgcan_read_messages()
        if message is not None:
            return message

    if dely_retries > 0:
        _log_error("Slow receive attempt also timeout!")
    return None


def zlgcan_receive_filtered(expected_can_id: int, expected_frames: int = 1, max_retries: int = 10) -> Optional[tuple[int, list[int], int]]:
    """
    Receive CAN bus message filtered by expected CAN ID with multi-frame protocol support.
    
    This function filters received frames to only return those matching the expected CAN ID.
    Non-matching frames are discarded (they belong to other concurrent requests).
    
    Supports multi-frame protocols:
    - MultiRead (0x0B): Detects is_last flag in byte[1] bit 7
    - TouchSensorRead (0x0D): Detects total/seq in byte[0] (total:4bit|seq:4bit)
    
    Args:
        expected_can_id: Expected CAN ID to filter by
        expected_frames: Expected frame count (hint from SDK)
        max_retries: Maximum retry attempts
        
    Returns:
        (can_id, data, frame_count) tuple or None if timeout
    """
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
            can_msgs = can_receive(DEVICE_TYPE, DEVICE_INDEX, 0)
            if can_msgs is None:
                continue
            
            for msg in can_msgs:
                if msg.hdr.inf.tx == 1:
                    continue
                
                frame_id = msg.hdr.id
                can_dlc = msg.hdr.len
                
                if frame_id != expected_can_id:
                    _log_debug(f"Skipping frame with different CAN ID: 0x{frame_id:x} (expected 0x{expected_can_id:x})")
                    continue
                
                frame_data = [msg.dat[j] for j in range(can_dlc)]
                
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
                        _log_debug(f"TouchSensorRead frame: total={total}, seq={seq}, data={[hex(b) for b in frame_data]}")
                        if total > 0 and seq > 0:
                            if not is_multi_frame:
                                is_multi_frame = True
                                total_frames = total
                                _log_debug(f"Multi-frame detected: expecting {total_frames} frames")
                            all_data.extend(frame_data)
                            frame_count += 1
                            _log_debug(f"Received frame {frame_count}/{total_frames}")
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


def _zlgcan_read_messages() -> Optional[tuple[int, list[int]]]:
    """
    Read message from ZLG CAN device, support multi-frame response concatenation.

    Returns:
        (can_id, data) tuple, where data may contain data from multiple frames concatenated
    """
    try:
        can_msgs = can_receive(DEVICE_TYPE, DEVICE_INDEX, 0)
        if can_msgs is None:
            return None

        start_idx = 0
        rcount = len(can_msgs)

        first_msg = can_msgs[start_idx]
        can_id = first_msg.hdr.id
        data = [first_msg.dat[i] for i in range(first_msg.hdr.len)]

        if rcount > (start_idx + 1):
            for i in range(start_idx + 1, rcount):
                msg = can_msgs[i]
                if msg.hdr.inf.tx == 1:
                    continue
                if msg.hdr.id != can_id:
                    _log_warning(f"Received multiple messages, can_id different: 0x{can_id:x} != 0x{msg.hdr.id:x}")
                    break
                if msg.hdr.len != 8:
                    _log_warning(f"Received multiple messages, data length not equal to 8: {msg.hdr.len}")
                    break
                data.extend([msg.dat[j] for j in range(msg.hdr.len)])

        return (can_id, data)

    except Exception as e:
        _log_error(f"Read message exception: {e}")
        import traceback
        _log_error(traceback.format_exc())

    return None
