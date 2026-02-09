"""
Revo2 CANFD Touch Pressure Example - SocketCAN (SDK built-in)

Demonstrates pressure touch sensor reading via SDK's built-in SocketCAN (Linux only).

Usage:
    python socketcan_canfd_touch_pressure.py [iface] [slave_id]
    python socketcan_canfd_touch_pressure.py can1 0x7e
"""
import asyncio
import os
import platform
import sys

if platform.system() != "Linux":
    raise NotImplementedError("SocketCAN is only supported on Linux")

from canfd_utils import (
    logger, libstark, setup_shutdown_event,
    display_pressure_touch_status, setup_pressure_touch_sensors
)


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
    # Parse command line args: [iface] [slave_id]
    iface = sys.argv[1] if len(sys.argv) > 1 else os.getenv("STARK_SOCKETCAN_IFACE", "can0")
    slave_id = int(sys.argv[2], 0) if len(sys.argv) > 2 else int(os.getenv("STARK_SLAVE_ID", "0x7e"), 0)
    master_id = int(os.getenv("STARK_MASTER_ID", "1"), 0)

    # Use SDK built-in SocketCAN (more reliable than Python socket)
    libstark.init_socketcan_canfd(iface)
    client = libstark.init_device_handler(libstark.StarkProtocolType.CanFd, master_id)

    # Get device info
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device: {device_info.description}")

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD Baudrate: {baudrate}")

    # Check hardware type
    if not client.uses_pressure_touch_api(slave_id):
        logger.error("This example requires Revo2 Touch Pressure hardware")
        libstark.close_socketcan()
        sys.exit(1)

    # Setup touch sensors
    await setup_pressure_touch_sensors(client, slave_id)

    # Monitor
    shutdown_event = setup_shutdown_event(logger)
    monitor_task = asyncio.create_task(monitor_pressure_touch(client, slave_id, shutdown_event))

    await shutdown_event.wait()
    logger.info("Shutting down...")
    monitor_task.cancel()
    libstark.close_socketcan()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
