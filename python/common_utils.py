"""
Common utility functions shared across all example scripts

This module provides common utility functions used by various example scripts,
including signal handling and shutdown event management.
"""

import asyncio
import platform
import signal
import sys

from common_imports import logger


def setup_shutdown_event(log=None):
    """
    Set up shutdown event handler for graceful termination
    
    Creates an asyncio Event and registers signal handlers (SIGINT, SIGTERM)
    to allow graceful shutdown of async applications.
    
    Args:
        log: Optional logger instance for logging shutdown messages.
             If None, uses default logger.
    
    Returns:
        asyncio.Event: Event that will be set when shutdown signal is received
    
    Example:
        shutdown_event = setup_shutdown_event(logger)
        await shutdown_event.wait()  # Wait for shutdown signal
    """
    # Create an event for shutdown
    shutdown_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    _logger = log or logger

    def shutdown_handler():
        _logger.info("Shutdown signal received")
        shutdown_event.set()
        # sys.exit(0)

    if platform.system() != "Windows":
        # Register signal handler for Unix-like systems
        loop.add_signal_handler(signal.SIGINT, shutdown_handler)
        loop.add_signal_handler(signal.SIGTERM, shutdown_handler)
    else:
        # Use different way to handle signal in Windows
        signal.signal(signal.SIGINT, lambda s, f: shutdown_handler())
        signal.signal(signal.SIGTERM, lambda s, f: shutdown_handler())

    return shutdown_event


# Touch sensor print utilities

def format_direction(direction: int) -> str:
    """Format direction value, 65535 (0xFFFF) means invalid"""
    return "N/A" if direction == 65535 else f"{direction}°"


def print_finger_touch_data_revo1(finger, channel: int, finger_name: str):
    """Print touch data for Revo1 Touch API (with direction)"""
    print(f"\n--- {finger_name} (Channel {channel}) ---")

    if channel == 0:
        # Thumb: 2 force groups, 1 self-proximity
        print(f"  Force Group 1: Normal={finger.normal_force1}, Tangential={finger.tangential_force1}, Direction={format_direction(finger.tangential_direction1)}")
        print(f"  Force Group 2: Normal={finger.normal_force2}, Tangential={finger.tangential_force2}, Direction={format_direction(finger.tangential_direction2)}")
        print(f"  Self-proximity: {finger.self_proximity1}")
    elif channel in [1, 2, 3]:
        # Index/Middle/Ring: 3 force groups, 2 self-proximity, 1 mutual-proximity
        print(f"  Force Group 1: Normal={finger.normal_force1}, Tangential={finger.tangential_force1}, Direction={format_direction(finger.tangential_direction1)}")
        print(f"  Force Group 2: Normal={finger.normal_force2}, Tangential={finger.tangential_force2}, Direction={format_direction(finger.tangential_direction2)}")
        print(f"  Force Group 3: Normal={finger.normal_force3}, Tangential={finger.tangential_force3}, Direction={format_direction(finger.tangential_direction3)}")
        print(f"  Self-proximity 1: {finger.self_proximity1}, Self-proximity 2: {finger.self_proximity2}")
        print(f"  Mutual-proximity: {finger.mutual_proximity}")
    elif channel == 4:
        # Pinky: 2 force groups, 1 self-proximity, NO mutual-proximity
        print(f"  Force Group 1: Normal={finger.normal_force1}, Tangential={finger.tangential_force1}, Direction={format_direction(finger.tangential_direction1)}")
        print(f"  Force Group 2: Normal={finger.normal_force2}, Tangential={finger.tangential_force2}, Direction={format_direction(finger.tangential_direction2)}")
        print(f"  Self-proximity: {finger.self_proximity1}")

    status_map = {0: "Normal", 1: "Data Error", 2: "Communication Error"}
    status_str = status_map.get(finger.status, "Unknown")
    print(f"  Status: {finger.status} ({status_str})")


def print_finger_touch_data_revo2(finger, finger_name: str):
    """Print touch data for Revo2 Touch API (no direction)"""
    status_map = {0: "OK", 1: "DataErr", 2: "CommErr"}
    status_str = status_map.get(finger.status, "Unknown")
    print(
        f"  {finger_name}: Normal={finger.normal_force1:4}, "
        f"Tangential={finger.tangential_force1:4}, "
        f"Self-prox={finger.self_proximity1:8}, Status={status_str}"
    )


def print_finger_touch_data(finger, finger_name: str, hw_type, channel: int = None):
    """
    Print touch data for Revo1 or Revo2 based on hw_type
    
    Args:
        finger: Touch data object from either Revo1 or Revo2
        finger_name: Display name for the finger (e.g., "Thumb", "Index")
        hw_type: Hardware type enum (required, e.g., sdk.StarkHardwareType.Revo1Touch)
        channel: Optional channel number (0-4), only used for Revo1 detailed output.
                 If None, will try to infer from finger_name.
    """
    from common_imports import uses_revo1_touch_api
    
    if uses_revo1_touch_api(hw_type):
        # Infer channel from finger_name if not provided
        if channel is None:
            name_to_channel = {
                "thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4,
                "拇指": 0, "食指": 1, "中指": 2, "无名指": 3, "小指": 4
            }
            channel = name_to_channel.get(finger_name.lower(), 1)  # default to index
        print_finger_touch_data_revo1(finger, channel, finger_name)
    else:
        print_finger_touch_data_revo2(finger, finger_name)
