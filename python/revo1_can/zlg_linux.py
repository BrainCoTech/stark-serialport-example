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


def zlgcan_receive_filtered(expected_can_id: int, max_retries: int = 5) -> Optional[tuple[int, list[int], int]]:
    """
    Receive CAN bus message filtered by expected CAN ID.
    
    This function filters received frames to only return those matching the expected CAN ID.
    Non-matching frames are discarded (they belong to other concurrent requests).
    
    Args:
        expected_can_id: Expected CAN ID to filter by
        max_retries: Maximum retry attempts
        
    Returns:
        (can_id, data, frame_count) tuple or None if timeout
    """
    for _attempt in range(max_retries):
        time.sleep(0.005)  # 5ms delay
        
        try:
            can_msgs = can_receive(DEVICE_TYPE, DEVICE_INDEX, 0)
            if can_msgs is None:
                continue
            
            # Filter frames by expected CAN ID
            matching_data = []
            frame_count = 0
            
            for msg in can_msgs:
                # Skip TX echo
                if msg.hdr.inf.tx == 1:
                    continue
                
                frame_id = msg.hdr.id
                
                # Check if this frame matches expected CAN ID
                if frame_id == expected_can_id:
                    # Append frame data
                    matching_data.extend([msg.dat[j] for j in range(msg.hdr.len)])
                    frame_count += 1
                else:
                    # Log but don't fail - other responses are for different requests
                    logger.debug(f"Skipping frame with different CAN ID: 0x{frame_id:x} (expected 0x{expected_can_id:x})")
            
            if frame_count > 0:
                return (expected_can_id, matching_data, frame_count)
                
        except Exception as e:
            logger.error(f"Receive filtered exception: {e}")
    
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
