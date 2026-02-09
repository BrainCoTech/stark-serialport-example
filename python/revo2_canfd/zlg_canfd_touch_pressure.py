"""
Revo2 CANFD Touch Pressure Example - ZLG Adapter

Demonstrates pressure touch sensor reading via ZLG USB-CAN adapter.
Supports both Windows and Linux.

Usage:
    python zlg_canfd_touch_pressure.py
    STARK_SLAVE_ID=0x7f python zlg_canfd_touch_pressure.py
"""
import asyncio
import os
import platform
import sys

from canfd_utils import (
    logger, libstark, setup_shutdown_event,
    display_pressure_touch_status, setup_pressure_touch_sensors
)

# Import ZLG driver based on OS
if platform.system() == "Windows":
    from zlg_win import zlgcan_open, zlgcan_close, zlgcan_send_message, zlgcan_receive_message
elif platform.system() == "Linux":
    from zlg_linux import zlgcan_open, zlgcan_close, zlgcan_send_message, zlgcan_receive_canfd_filtered
else:
    raise NotImplementedError(f"Unsupported OS: {platform.system()}")


def canfd_send(_slave_id: int, can_id: int, data: list):
    if not zlgcan_send_message(can_id, bytes(data)):
        logger.error("Failed to send CANFD message")


def canfd_read(_slave_id: int, expected_can_id: int, expected_frames: int):
    """
    CANFD read callback
    
    Args:
        _slave_id: Slave ID (not used)
        expected_can_id: Expected CAN ID for filtering
        expected_frames: Expected frame count (hint from SDK)
    """
    result = zlgcan_receive_canfd_filtered(expected_can_id, expected_frames)
    if result is None:
        return 0, bytes([])
    can_id, data, _frame_count = result
    logger.debug(f"Received CANFD - ID: {can_id:02x}, Data: {bytes(data).hex()}")
    return can_id, bytes(data)


async def monitor_pressure_touch(client, slave_id: int, shutdown_event):
    """Monitor pressure touch data periodically"""
    logger.info(f"Pressure touch monitoring started for device 0x{slave_id:02x}")
    while not shutdown_event.is_set():
        try:
            await display_pressure_touch_status(client, slave_id)
            await asyncio.sleep(0.1)
        except Exception as e:
            logger.error(f"Monitor error: {e}")
            await asyncio.sleep(1)


async def main():
    # Config from env vars
    master_id = int(os.getenv("STARK_MASTER_ID", "1"), 0)
    slave_id = int(os.getenv("STARK_SLAVE_ID", "0x7e"), 0)

    # Initialize
    client = libstark.init_device_handler(libstark.StarkProtocolType.CanFd, master_id)
    zlgcan_open()
    libstark.set_can_tx_callback(canfd_send)
    libstark.set_can_rx_callback(canfd_read)

    # Get device info
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device: {device_info.description}")

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD Baudrate: {baudrate}")

    # Check hardware type
    if not client.uses_pressure_touch_api(slave_id):
        logger.error("This example requires Revo2 Touch Pressure hardware")
        zlgcan_close()
        sys.exit(1)

    # Setup touch sensors
    await setup_pressure_touch_sensors(client, slave_id)

    # Monitor
    shutdown_event = setup_shutdown_event(logger)
    monitor_task = asyncio.create_task(monitor_pressure_touch(client, slave_id, shutdown_event))

    await shutdown_event.wait()
    logger.info("Shutting down...")
    monitor_task.cancel()
    zlgcan_close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
