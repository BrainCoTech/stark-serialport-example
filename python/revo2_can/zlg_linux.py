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
DEVICE_INDEX = 0  # Card index

def zlgcan_open():
    """Open ZLG CAN device"""
    try:
        # Open device
        open_device(DEVICE_TYPE, DEVICE_INDEX)

        # Start channel 0
        canfd_start(DEVICE_TYPE, DEVICE_INDEX, 0)

    except Exception as e:
        logger.error(f"Failed to open device: {e}")


def zlgcan_close():
    """Close ZLG CAN device"""
    try:
        close_device(DEVICE_TYPE, DEVICE_INDEX)
    except Exception as e:
        logger.error(f"Failed to close device: {e}")


def zlgcan_send_message(can_id: int, data: bytes) -> bool:
    """Send CAN message"""
    try:
        logger.debug(f"Send CAN - ID: 0x{can_id:x}, Data: {data.hex()}")
        if not can_send(DEVICE_TYPE, DEVICE_INDEX, 0, can_id, data):
            logger.error("Failed to send data!")
            return False
        logger.debug("Send data successfully")
        return True
    except Exception as e:
        logger.error(f"Send message exception: {e}")
        return False

def zlgcan_receive_message(quick_retries: int = 2, dely_retries: int = 0) -> Optional[tuple[int, list[int]]]:
    """
    Receive CAN bus message, first try quick receive, if failed then try slow receive in DFU mode.

    Args:
        quick_retries (int): Quick receive attempt次数，默认2次。
        dely_retries (int): Slow receive attempt次数 in DFU mode，默认0次。

    Returns:
        Received message or None(receive failed)
    """
    # Quick receive attempt with longer delay for device response
    for _attempt in range(quick_retries):
        time.sleep(0.001)  # 1ms delay - device needs time to respond
        message = _zlgcan_read_messages()
        if message is not None:
            return message

    logger.debug("Quick receive timeout")

    # Slow receive attempt
    for _attempt in range(dely_retries):
        time.sleep(2.0)  # Long delay, for the last packet in DFU mode
        message = _zlgcan_read_messages() # type: ignore [report]
        if message is not None:
            return message

    if dely_retries > 0:
        logger.error("Slow receive attempt also timeout!")
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
    # Extract command from CAN ID to detect multi-frame commands
    cmd = (expected_can_id >> 3) & 0x0F
    is_multi_frame_cmd = (cmd == 0x0B or cmd == 0x0D)  # MultiRead or TouchSensorRead
    
    # Check if this is DFU mode (expected_can_id == 0)
    is_dfu_mode = (expected_can_id == 0)
    
    # Determine retry strategy
    if is_dfu_mode:
        max_retries = 200
    elif expected_frames > 1 or is_multi_frame_cmd:
        max_retries = 30
    
    all_data = []
    frame_count = 0
    total_frames = 0  # For TouchSensorRead multi-frame detection
    is_multi_frame = False
    
    for attempt in range(max_retries):
        wait_ms = 0.002 if attempt < 5 else 0.005
        time.sleep(wait_ms)
        
        try:
            can_msgs = can_receive(DEVICE_TYPE, DEVICE_INDEX, 0)
            if can_msgs is None:
                continue
            
            for msg in can_msgs:
                # Skip TX echo
                if msg.hdr.inf.tx == 1:
                    continue
                
                frame_id = msg.hdr.id
                can_dlc = msg.hdr.len
                
                # Check if this frame matches expected CAN ID
                if frame_id != expected_can_id:
                    logger.debug(f"Skipping frame with different CAN ID: 0x{frame_id:x} (expected 0x{expected_can_id:x})")
                    continue
                
                frame_data = [msg.dat[j] for j in range(can_dlc)]
                
                # Check for multi-frame protocol format
                if is_multi_frame_cmd and can_dlc > 0:
                    frame_header = frame_data[0]
                    
                    if cmd == 0x0B:
                        # MultiRead format: [addr, len|flag, data...]
                        if can_dlc >= 2:
                            len_and_flag = frame_data[1]
                            is_last = (len_and_flag & 0x80) != 0
                            
                            # Append entire frame (8 bytes)
                            all_data.extend(frame_data)
                            frame_count += 1
                            
                            # Check if this is the last frame
                            if is_last:
                                return (expected_can_id, all_data, frame_count)
                            continue
                    
                    elif cmd == 0x0D:
                        # TouchSensorRead format: [total:4bit|seq:4bit, data...]
                        total = (frame_header >> 4) & 0x0F
                        seq = frame_header & 0x0F
                        
                        # Detect multi-frame data
                        if total > 0 and seq > 0:
                            if not is_multi_frame:
                                is_multi_frame = True
                                total_frames = total
                            
                            # Append entire frame (8 bytes)
                            all_data.extend(frame_data)
                            frame_count += 1
                            
                            # Check if all frames received
                            if frame_count >= total_frames:
                                return (expected_can_id, all_data, frame_count)
                            continue
                
                # Standard single-frame or non-protocol data
                all_data.extend(frame_data)
                frame_count += 1
                
                # For single frame request, return immediately
                if expected_frames <= 1 and not is_multi_frame_cmd:
                    return (expected_can_id, all_data, frame_count)
                
                # Check if we have enough frames (for non-protocol multi-frame)
                if expected_frames > 1 and frame_count >= expected_frames:
                    return (expected_can_id, all_data, frame_count)
                
        except Exception as e:
            logger.error(f"Receive filtered exception: {e}")
    
    # Timeout - return whatever we have
    if all_data:
        return (expected_can_id, all_data, frame_count)
    
    return None


def _zlgcan_read_messages() -> Optional[tuple[int, list[int]]]:
    """
    Read message from ZLG CAN device, support multi-frame response concatenation

    Returns:
        (can_id, data) tuple, where data may contain data from multiple frames concatenated
    """
    try:
        # Receive message
        can_msgs = can_receive(DEVICE_TYPE, DEVICE_INDEX, 0)

        if can_msgs is None:
            return None

        start_idx = 0
        rcount = len(can_msgs)

        # Process the first received message
        first_msg = can_msgs[start_idx]
        can_id = first_msg.hdr.id
        data = [first_msg.dat[i] for i in range(first_msg.hdr.len)]

        # Received multiple messages, when can_id is the same, read multiple frames of response for concatenation
        if rcount > (start_idx + 1):
            # logger.debug(f"Received {rcount - start_idx} valid messages, try to concatenate multiple frames of data")
            for i in range(start_idx + 1, rcount):
                msg = can_msgs[i]
                # Skip TX echo
                if msg.hdr.inf.tx == 1:
                    continue

                if msg.hdr.id != can_id:
                    logger.warning(
                        f"Received multiple messages, can_id different: 0x{can_id:x} != 0x{msg.hdr.id:x}"
                    )
                    break

                # Multiple data frames, length should be equal to 8 (8 bytes per frame)
                if msg.hdr.len != 8:
                    logger.warning(
                        f"Received multiple messages, data length not equal to 8: {msg.hdr.len}"
                    )
                    break

                # Concatenate data
                data.extend([msg.dat[j] for j in range(msg.hdr.len)])

            # logger.debug(f"Multi-frame concatenation completed, total length: {len(data)} bytes")

        return (can_id, data)

    except Exception as e:
        logger.error(f"Read message exception: {e}")
        import traceback
        logger.error(traceback.format_exc())

    return None
