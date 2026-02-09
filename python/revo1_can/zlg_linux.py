"""
ZLG CAN Linux driver wrapper for revo1_can examples.

This module re-exports functions from the shared zlgcan_linux_utils module.
"""
import sys
from pathlib import Path

current_dir = Path(__file__).resolve().parent
parent_dir = current_dir.parent
sys.path.append(str(parent_dir))

from can_utils import logger
from dll.zlgcan.zlgcan_linux_utils import (
    set_logger,
    zlgcan_open,
    zlgcan_close,
    zlgcan_send_message,
    zlgcan_receive_message,
    zlgcan_receive_filtered,
)

# Initialize logger for the shared module
set_logger(logger)

__all__ = [
    "zlgcan_open",
    "zlgcan_close",
    "zlgcan_send_message",
    "zlgcan_receive_message",
    "zlgcan_receive_filtered",
]
