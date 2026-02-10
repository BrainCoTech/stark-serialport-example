import time
import sys
from pathlib import Path
from typing import Optional
from canfd_utils import logger

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
    """Send CANFD message"""
    try:
        logger.debug(f"Send CANFD - ID: 0x{can_id:x}, Data: {data.hex()}")
        if not canfd_send(DEVICE_TYPE, DEVICE_INDEX, 0, can_id, data):
            logger.error("Failed to send data!")
            return False
        logger.debug("Send data successfully")
        return True
    except Exception as e:
        logger.error(f"Failed to send message: {e}")
        return False

def zlgcan_receive_message(quick_retries: int = 2, dely_retries: int = 0) -> Optional[tuple[int, list[int]]]:
    """
    Receive CAN bus messages, first try quick reception, if failed, try slow reception in DFU mode.

    Parameters:
        quick_retries (int): Number of quick reception attempts, default is 2.
        dely_retries (int): Number of slow reception attempts in DFU mode, default is 0.

    Returns:
        Received message or None (receive failed)
    """
    # Quick reception attempt
    for _attempt in range(quick_retries):
        time.sleep(0.01)  # short delay, Adjust based on actual situation
        message = _zlgcan_read_messages()
        if message is not None:
            return message

    logger.warning("Quick reception timeout")

    # Slow reception attempt
    for _attempt in range(dely_retries):
        time.sleep(2.0)  # Longer delay, suitable for the last packet of data in DFU mode
        message = _zlgcan_read_messages()
        if message is not None:
            return message

    if dely_retries > 0:
        logger.error("Slow reception timeout!")
    return None


def zlgcan_receive_canfd_filtered(
    expected_can_id: int, expected_frames: int = 1, max_retries: int = 100
) -> Optional[tuple[int, list[int], int]]:
    """
    Receive CANFD message filtered by slave_id and master_id.
    
    CANFD CAN ID format: (slave_id << 16) | (master_id << 8) | payload_len
    
    This function matches the Rust implementation for CANFD receive:
    - Extract expected_slave_id and expected_master_id from expected_can_id
    - Match received frames by slave_id and master_id (not exact CAN ID match)
    
    Args:
        expected_can_id: Expected CAN ID containing slave_id and master_id
        expected_frames: Expected frame count (hint from SDK)
        max_retries: Maximum retry attempts (default: 100 for ~200ms timeout)
        
    Returns:
        (can_id, data, frame_count) tuple or None if timeout
        
    Note:
        Sleep timing (0.1ms) optimized for Linux (Ubuntu).
        Windows not tested - may need adjustment if performance issues occur.
    """
    # CANFD CAN ID format: (slave_id << 16) | (master_id << 8) | payload_len
    expected_slave_id = (expected_can_id >> 16) & 0xFF
    expected_master_id = (expected_can_id >> 8) & 0xFF
    
    logger.debug(f"CANFD RX: waiting, expected_can_id=0x{expected_can_id:08X}, "
                 f"slave=0x{expected_slave_id:02X}, master=0x{expected_master_id:02X}")
    
    for attempt in range(max_retries):
        try:
            can_msgs = canfd_receive(DEVICE_TYPE, DEVICE_INDEX, 0)
            if can_msgs is None:
                # Only sleep on empty receive, minimal delay
                time.sleep(0.0001)  # 0.1ms
                continue
            
            for msg in can_msgs:
                can_id = msg.hdr.id
                
                # Match slave_id and master_id from CAN ID
                resp_slave_id = (can_id >> 16) & 0xFF
                resp_master_id = (can_id >> 8) & 0xFF
                
                logger.debug(f"CANFD RX: received - can_id=0x{can_id:08X}, "
                             f"slave=0x{resp_slave_id:02X}, master=0x{resp_master_id:02X}")
                
                if resp_slave_id == expected_slave_id and resp_master_id == expected_master_id:
                    data = [msg.dat[i] for i in range(msg.hdr.len)]
                    return (can_id, data, 1)
                    
        except Exception as e:
            logger.error(f"CANFD receive filtered exception: {e}")
    
    logger.debug(f"CANFD receive timeout after {max_retries} attempts")
    return None


def _zlgcan_read_messages() -> Optional[tuple[int, list[int]]]:
    """
    Read messages from ZLG CAN device, support multi-frame response concatenation

    Returns:
        (can_id, data) tuple, where data may contain multi-frame concatenated data
    """
    try:
        # Receive messages
        can_msgs = canfd_receive(DEVICE_TYPE, DEVICE_INDEX, 0)

        if can_msgs is None:
            return None

        # Process the first received message
        first_msg = can_msgs[0]
        can_id = first_msg.hdr.id
        data = [first_msg.dat[i] for i in range(first_msg.hdr.len)]
        return (can_id, data)

    except Exception as e:
        logger.error(f"Failed to read message: {e}")
        import traceback
        logger.error(traceback.format_exc())

    return None
