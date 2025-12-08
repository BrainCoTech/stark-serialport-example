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
