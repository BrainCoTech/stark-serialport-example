"""
Common utility functions shared across all example scripts

This module provides common utility functions used by various example scripts,
including signal handling and shutdown event management.
"""

import asyncio
import platform
import signal
import sys


def setup_shutdown_event(logger=None):
    """
    Set up shutdown event handler for graceful termination
    
    Creates an asyncio Event and registers signal handlers (SIGINT, SIGTERM)
    to allow graceful shutdown of async applications.
    
    Args:
        logger: Optional logger instance for logging shutdown messages.
                If None, prints to stdout.
    
    Returns:
        asyncio.Event: Event that will be set when shutdown signal is received
    
    Example:
        shutdown_event = setup_shutdown_event(logger)
        await shutdown_event.wait()  # Wait for shutdown signal
    """
    # Create an event for shutdown
    shutdown_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    def shutdown_handler():
        if logger is not None:
            logger.info("Shutdown signal received")
        else:
            print("Shutdown signal received")
        sys.exit(0)

    if platform.system() != "Windows":
        # Register signal handler for Unix-like systems
        loop.add_signal_handler(signal.SIGINT, shutdown_handler)
        loop.add_signal_handler(signal.SIGTERM, shutdown_handler)
    else:
        # Use different way to handle signal in Windows
        signal.signal(signal.SIGINT, lambda s, f: shutdown_handler())
        signal.signal(signal.SIGTERM, lambda s, f: shutdown_handler())

    return shutdown_event
