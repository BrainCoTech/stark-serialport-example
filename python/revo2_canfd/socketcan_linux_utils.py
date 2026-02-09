"""SocketCAN utilities for Revo2 CANFD (64 bytes)"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common_socketcan import (
    socketcan_open as _socketcan_open,
    socketcan_close,
    socketcan_send_message,
    socketcan_receive_message,
    socketcan_receive_filtered,
    socketcan_receive_canfd_filtered,
)


def socketcan_open(iface=None):
    """Open SocketCAN in CANFD mode"""
    _socketcan_open(iface, canfd=True)
