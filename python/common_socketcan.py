"""
Common SocketCAN utilities for Linux

Supports both CAN 2.0 (8 bytes) and CANFD (64 bytes) modes.
"""

import os
import socket
import struct
import time
from typing import Optional, Tuple

from common_imports import logger

# CAN constants
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF
CAN_RAW = 1
CAN_RAW_FD_FRAMES = 5

# Frame sizes and formats
CAN_MTU = 16
CANFD_MTU = 72
CAN_FRAME_FMT = "=IB3x8s"
CANFD_FRAME_FMT = "=IBBBx64s"

_sock = None
_is_canfd = False


def _get_iface() -> str:
    return os.getenv("STARK_SOCKETCAN_IFACE", "can0")


def socketcan_open(iface: Optional[str] = None, canfd: bool = False) -> None:
    """
    Open SocketCAN interface
    
    Args:
        iface: CAN interface name (default: from STARK_SOCKETCAN_IFACE env or "can0")
        canfd: Enable CANFD mode (64 bytes payload)
    """
    global _sock, _is_canfd
    if _sock is not None:
        return

    if iface is None:
        iface = _get_iface()

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
    if canfd:
        sock.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    sock.bind((iface,))
    sock.settimeout(0.5)
    _sock = sock
    _is_canfd = canfd
    mode_str = "CANFD" if canfd else "CAN"
    logger.info(f"SocketCAN ({mode_str}) opened on {iface}")


def socketcan_close() -> None:
    """Close SocketCAN interface"""
    global _sock, _is_canfd
    if _sock is None:
        return
    _sock.close()
    _sock = None
    _is_canfd = False
    logger.info("SocketCAN closed")


def socketcan_send_message(can_id: int, data: bytes) -> bool:
    """Send CAN/CANFD message"""
    if _sock is None:
        logger.error("SocketCAN not initialized")
        return False

    try:
        if _is_canfd:
            payload = data[:64]
            can_id_flag = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG
            frame = struct.pack(
                CANFD_FRAME_FMT,
                can_id_flag,
                len(payload),
                1,  # CANFD_BRS
                0,
                payload.ljust(64, b"\x00"),
            )
        else:
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
        _sock.send(frame)
        return True
    except OSError as exc:
        logger.error(f"SocketCAN send failed: {exc}")
        return False


def socketcan_receive_message(
    quick_retries: int = 5, dely_retries: int = 2
) -> Optional[Tuple[int, bytes]]:
    """Receive single CAN/CANFD message (legacy interface)"""
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
    Receive CAN message filtered by expected CAN ID with multi-frame protocol support.
    
    Filters received frames to only return those matching the expected CAN ID.
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
    
    # Determine retry strategy (aligned with Rust ZQWL):
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
    """Read single CAN/CANFD frame from socket"""
    if _sock is None:
        return None
    try:
        if _is_canfd:
            data = _sock.recv(CANFD_MTU)
            if len(data) == CANFD_MTU:
                can_id, length, _flags, _rsvd, payload = struct.unpack(CANFD_FRAME_FMT, data)
                can_id = can_id & CAN_EFF_MASK
                return can_id, payload[:length]
            if len(data) == CAN_MTU:
                can_id, length, payload = struct.unpack(CAN_FRAME_FMT, data)
                can_id = can_id & CAN_EFF_MASK
                return can_id, payload[:length]
        else:
            data = _sock.recv(CAN_MTU)
            if len(data) != CAN_MTU:
                return None
            can_id, length, payload = struct.unpack(CAN_FRAME_FMT, data)
            can_id = can_id & CAN_EFF_MASK
            return can_id, payload[:length]
        return None
    except socket.timeout:
        return None
    except OSError as exc:
        logger.error(f"SocketCAN recv failed: {exc}")
        return None


def socketcan_receive_canfd_filtered(
    expected_can_id: int, expected_frames: int = 1, max_retries: int = 2
) -> Optional[Tuple[int, bytes, int]]:
    """
    Receive CANFD message filtered by slave_id and master_id.
    
    CANFD CAN ID format: (slave_id << 16) | (master_id << 8) | payload_len
    
    This function matches the Rust implementation for CANFD receive:
    - Extract expected_slave_id and expected_master_id from expected_can_id
    - Match received frames by slave_id and master_id (not exact CAN ID match)
    
    Args:
        expected_can_id: Expected CAN ID containing slave_id and master_id
        expected_frames: Expected frame count (hint from SDK)
        max_retries: Maximum retry attempts (default: 2, aligned with Rust ZQWL)
        
    Returns:
        (can_id, data, frame_count) tuple or None if timeout
    """
    if _sock is None:
        logger.error("SocketCAN not initialized")
        return None
    
    # CANFD CAN ID format: (slave_id << 16) | (master_id << 8) | payload_len
    expected_slave_id = (expected_can_id >> 16) & 0xFF
    expected_master_id = (expected_can_id >> 8) & 0xFF
    
    logger.debug(f"CANFD RX: waiting, expected_can_id=0x{expected_can_id:08X}, "
                 f"slave=0x{expected_slave_id:02X}, master=0x{expected_master_id:02X}")
    
    for attempt in range(max_retries):
        wait_ms = 0.002 if attempt < 5 else 0.005
        time.sleep(wait_ms)
        
        try:
            msg = _socketcan_read_message()
            if msg is None:
                continue
            
            can_id, data = msg
            
            # Match slave_id and master_id from CAN ID
            resp_slave_id = (can_id >> 16) & 0xFF
            resp_master_id = (can_id >> 8) & 0xFF
            
            logger.debug(f"CANFD RX: received - can_id=0x{can_id:08X}, "
                         f"slave=0x{resp_slave_id:02X}, master=0x{resp_master_id:02X}")
            
            if resp_slave_id == expected_slave_id and resp_master_id == expected_master_id:
                return (can_id, data, 1)
                
        except Exception as e:
            logger.error(f"SocketCAN CANFD receive exception: {e}")
    
    logger.debug(f"SocketCAN CANFD receive timeout after {max_retries} attempts")
    return None
