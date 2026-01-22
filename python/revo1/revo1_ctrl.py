"""
Revo1 Dexterous Hand Automatic Control Example

This example demonstrates how to implement automatic control of the Revo1 dexterous hand, including:
- Initialization and connection of the dexterous hand
- Periodic monitoring of motor status
- Automatic grip and open actions based on status
- Precise control of individual fingers
- Creation and management of asynchronous tasks
"""

import asyncio
import sys
import time
from utils import setup_shutdown_event
from revo1_utils import *


async def get_motor_status_periodically(client, slave_id):
    """
    Periodically get motor status and execute automatic control

    This function continuously monitors motor status and executes automatic open/close actions based on finger position:
    - When fingers are open, execute grip action
    - When fingers are closed, execute open action
    - Record time cost and status information for each operation

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    logger.info("Motor status monitoring started")
    index = 0

    while True:
        try:
            # Get motor status
            logger.debug("get_motor_status")
            start = time.perf_counter()
            status: libstark.MotorStatusData = await client.get_motor_status(slave_id)
            cost_ms = (time.perf_counter() - start) * 1000

            # Log status information
            logger.info(
                f"[{index}] Motor status - cost: {cost_ms:.2f}ms, "
                f"is_idle: {status.is_idle()}, "
                f"is_closed: {status.is_closed()}, "
                f"is_opened: {status.is_opened()}"
            )
            logger.info(f"[{index}] Motor status: {status.description}")
            index += 1

            # Execute automatic control based on current status
            if status.is_idle():
                if status.is_opened():
                    # When fingers are open, execute grip action
                    # Position values: [Thumb, Thumb Auxiliary, Index, Middle, Ring, Pinky]
                    await client.set_finger_positions(
                        slave_id, [600, 600, 1000, 1000, 1000, 1000]
                    )
                elif status.is_closed():
                    # When fingers are closed, execute open action
                    # Set all fingers to minimum position value (0)
                    await client.set_finger_positions(slave_id, [0] * 6)

        except Exception as e:
            logger.error(f"Error in motor status monitoring: {e}")
            # Wait 1 second before retry when error occurs
            await asyncio.sleep(1)


async def main():
    """
    Main function: Initialize dexterous hand connection and start automatic control task
    """
    # Set up shutdown event listener
    shutdown_event = setup_shutdown_event(logger)

    # Detect baud rate and device ID of the dexterous hand, initialize client
    (client, slave_id) = await open_modbus_revo1(port_name=None, quick=True) # Replace with actual serial port name, passing None will attempt auto-detection

    # Execute initialization actions
    # Set single finger control: Pinky closes to 100% position
    await client.set_finger_position(slave_id, libstark.FingerId.Pinky, 1000)
    await asyncio.sleep(1)  # Wait 1 second

    # Execute grip action: Set all fingers to predetermined positions
    # Position parameters: [Thumb 60%, Thumb Auxiliary 60%, Index 100%, Middle 100%, Ring 100%, Pinky 100%]
    await client.set_finger_positions(slave_id, [600, 600, 1000, 1000, 1000, 1000])
    await asyncio.sleep(1)  # Wait 1 second

    # Execute OPEN action: Set all fingers to minimum position (0%)
    await client.set_finger_positions(slave_id, [0] * 6)
    await asyncio.sleep(1)  # Wait 1 second

    # Create and start motor status monitoring task
    asyncio.create_task(get_motor_status_periodically(client, slave_id))
    logger.info("Motor status monitoring task started")

    # Wait for shutdown event (Ctrl+C or other shutdown signals)
    await shutdown_event.wait()

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)