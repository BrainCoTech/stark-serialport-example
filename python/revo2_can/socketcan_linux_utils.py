import os
import socket
import struct
import time
from typing import Optional, Tuple

from can_utils import logger

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF
CAN_RAW = 1

CAN_MTU = 16
CAN_FRAME_FMT = "=IB3x8s"

_sock = None


def _get_iface() -> str:
    return os.getenv("STARK_SOCKETCAN_IFACE", "can0")


def socketcan_open(iface: Optional[str] = None) -> None:
    global _sock
    if _sock is not None:
        return

    if iface is None:
        iface = _get_iface()

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.bind((iface,))
    sock.settimeout(0.5)
    _sock = sock
    logger.info(f"SocketCAN opened on {iface}")


def socketcan_close() -> None:
    global _sock
    if _sock is None:
        return
    _sock.close()
    _sock = None
    logger.info("SocketCAN closed")


def socketcan_send_message(can_id: int, data: bytes) -> bool:
    if _sock is None:
        logger.error("SocketCAN not initialized")
        return False

    payload = data[:8]
    if can_id > CAN_SFF_MASK:
        can_id_flag = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG
    else:
        can_id_flag = can_id & CAN_SFF_MASK
    frame = struct.pack(
        CAN_FRAME_FMT,
        can_id_flag,
        len(payload),
        payload.ljust(8, b"\x00"),
    )
    try:
        _sock.send(frame)
        return True
    except OSError as exc:
        logger.error(f"SocketCAN send failed: {exc}")
        return False


def socketcan_receive_message(
    quick_retries: int = 5, dely_retries: int = 2
) -> Optional[Tuple[int, bytes]]:
    """Receive single CAN message (legacy interface)"""
    if _sock is None:
        logger.error("SocketCAN not initialized")
        return None

    for _ in range(quick_retries):
        msg = _socketcan_read_message()
        if msg is not None:
            return msg
        time.sleep(0.01)

    logger.warning("SocketCAN quick receive timeout")

    for _ in range(dely_retries):
        time.sleep(2.0)
        msg = _socketcan_read_message()
        if msg is not None:
            return msg

    if dely_retries > 0:
        logger.error("SocketCAN slow receive timeout")
    return None


def socketcan_receive_filtered(
    expected_can_id: int, expected_frames: int = 1, max_retries: int = 10
) -> Optional[Tuple[int, bytes, int]]:
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
    if _sock is None:
        logger.error("SocketCAN not initialized")
        return None
    
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
            msg = _socketcan_read_message()
            if msg is None:
                continue
            
            frame_id, frame_data = msg
            can_dlc = len(frame_data)
            
            # Check if this frame matches expected CAN ID
            if frame_id != expected_can_id:
                logger.debug(f"Skipping frame with different CAN ID: 0x{frame_id:x} (expected 0x{expected_can_id:x})")
                continue
            
            frame_bytes = list(frame_data)
            
            # Check for multi-frame protocol format
            if is_multi_frame_cmd and can_dlc > 0:
                frame_header = frame_bytes[0]
                
                if cmd == 0x0B:
                    # MultiRead format: [addr, len|flag, data...]
                    if can_dlc >= 2:
                        len_and_flag = frame_bytes[1]
                        is_last = (len_and_flag & 0x80) != 0
                        
                        # Append entire frame (8 bytes)
                        all_data.extend(frame_bytes)
                        frame_count += 1
                        
                        # Check if this is the last frame
                        if is_last:
                            return (expected_can_id, bytes(all_data), frame_count)
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
                        all_data.extend(frame_bytes)
                        frame_count += 1
                        
                        # Check if all frames received
                        if frame_count >= total_frames:
                            return (expected_can_id, bytes(all_data), frame_count)
                        continue
            
            # Standard single-frame or non-protocol data
            all_data.extend(frame_bytes)
            frame_count += 1
            
            # For single frame request, return immediately
            if expected_frames <= 1 and not is_multi_frame_cmd:
                return (expected_can_id, bytes(all_data), frame_count)
            
            # Check if we have enough frames (for non-protocol multi-frame)
            if expected_frames > 1 and frame_count >= expected_frames:
                return (expected_can_id, bytes(all_data), frame_count)
                
        except Exception as e:
            logger.error(f"SocketCAN receive filtered exception: {e}")
    
    # Timeout - return whatever we have
    if all_data:
        return (expected_can_id, bytes(all_data), frame_count)
    
    return None


def _socketcan_read_message() -> Optional[Tuple[int, bytes]]:
    if _sock is None:
        return None
    try:
        data = _sock.recv(CAN_MTU)
        if len(data) != CAN_MTU:
            return None
        can_id, length, payload = struct.unpack(CAN_FRAME_FMT, data)
        can_id = can_id & CAN_EFF_MASK
        return can_id, payload[:length]
    except socket.timeout:
        return None
    except OSError as exc:
        logger.error(f"SocketCAN recv failed: {exc}")
        return None
