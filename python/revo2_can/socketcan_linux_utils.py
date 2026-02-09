"""SocketCAN utilities for Revo2 CAN (CAN 2.0, 8 bytes)"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common_socketcan import (
    socketcan_open as _socketcan_open,
    socketcan_close,
    socketcan_send_message,
    socketcan_receive_message,
    socketcan_receive_filtered,
)


def socketcan_open(iface=None):
    """Open SocketCAN in CAN 2.0 mode"""
    _socketcan_open(iface, canfd=False)
