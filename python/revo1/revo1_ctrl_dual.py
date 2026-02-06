"""
Revo1 Dexterous Hand Dual Control Example

Connection method:
- Use two serial ports to connect left and right hands separately (can use the same device ID, default device ID is 1)
"""

import asyncio
import sys
import time
from revo1_utils import *
from utils import setup_shutdown_event

LEFT_PORT = "/dev/ttyUSB0" # Replace with actual left hand device port
RIGHT_PORT = "/dev/ttyUSB1" # Replace with actual right hand device port
LEFT_ID = 1 # Replace with actual left hand device ID
RIGHT_ID = 1 # Replace with actual right hand device ID
BAUDRATE = libstark.Baudrate.Baud115200

async def main():
    """
    Main function: Initialize device connection and control tasks
    """
    libstark.init_logging()
    shutdown_event = setup_shutdown_event(logger)

    # Open two devices
    client_left = await libstark.modbus_open(LEFT_PORT, BAUDRATE)
    client_right = await libstark.modbus_open(RIGHT_PORT, BAUDRATE)
    if not client_left or not client_right:
        logger.critical("Failed to open one or both serial ports.")
        sys.exit(1)

    # Get device information
    left_info = await client_left.get_device_info(LEFT_ID)
    right_info = await client_right.get_device_info(RIGHT_ID)
    if not left_info or not right_info:
        logger.critical("Failed to get device info for one or both devices.")
        sys.exit(1)
    logger.info(f"Left: {left_info.description}")
    logger.info(f"Right: {right_info.description}")

    # Start motor status monitoring task
    task = asyncio.create_task(
        monitor_and_control(client_left, client_right, LEFT_ID, RIGHT_ID)
    )

    # Wait for shutdown event
    await shutdown_event.wait()

    # Clean up resources
    task.cancel()
    await asyncio.gather(task, return_exceptions=True)
    await client_left.modbus_close()
    await client_right.modbus_close()
    logger.info("Modbus clients closed")
    sys.exit(0)

async def monitor_and_control(client_left, client_right, left_id, right_id):
    """
    Periodically get motor status and execute automatic control
    """
    logger.info(f"Motor status monitoring started for devices {left_id:02x} and {right_id:02x}")
    index = 0
    positions = [0] * 6
    period = 2.0

    while True:
        try:
            index += 1
            now_ms = int(time.time() * 1000)
            t = (now_ms / 1000.0) % period
            if t < period / 2:
                positions = [0, 0, 0, 0, 0, 0]
            else:
                positions = [0, 0, 1000, 1000, 1000, 1000]

            # Get motor status
            for client, dev_id, hand in [
                (client_left, left_id, "Left"),
                (client_right, right_id, "Right"),
            ]:
                start = time.perf_counter()
                try:
                    status = await client.get_motor_status(dev_id)
                    cost_ms = (time.perf_counter() - start) * 1000
                    logger.debug(
                        f"{hand} [{dev_id:02x}] [{index}] Motor status - cost: {cost_ms:.2f}ms, "
                        f"is_idle: {status.is_idle()}, is_closed: {status.is_closed()}, is_opened: {status.is_opened()}"
                    )
                    logger.debug(f"{hand} [{dev_id:02x}] [{index}] Motor status: {status.description}")
                except Exception as e:
                    logger.error(f"Error getting motor status for {hand} [{dev_id:02x}]: {e}")

            logger.info(f"Publishing positions: {positions}")
            await asyncio.gather(
                client_left.set_finger_positions(left_id, positions),
                client_right.set_finger_positions(right_id, positions),
            )
            await asyncio.sleep(0.01)
        except Exception as e:
            logger.error(f"Error in monitor_and_control loop: {e}")
            await asyncio.sleep(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)