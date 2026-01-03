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


def _socketcan_read_message() -> Optional[Tuple[int, bytes]]:
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
