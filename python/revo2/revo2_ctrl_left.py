import asyncio
import sys
import time
from revo2_utils import *

SLAVE_ID = 0x7e  # Left hand
PORT = "/dev/ttyUSB0"  # Replace with actual serial port name
BAUDRATE = libstark.Baudrate.Baud460800

async def main():
    client: libstark.DeviceContext = await libstark.modbus_open(PORT, BAUDRATE)
    if not client:
        logger.critical(f"Failed to open serial port: {PORT}")
        sys.exit(1)
    info = await client.get_device_info(SLAVE_ID)
    if not info:
        logger.critical(f"Failed to get device info for left hand. Id: {SLAVE_ID}")
        sys.exit(1)
    logger.info(f"Left: {info.description}")
    await client.set_finger_unit_mode(SLAVE_ID, libstark.FingerUnitMode.Normalized)
    period = 2.0
    speeds = [1000] * 6
    try:
        while True:
            now_ms = int(time.time() * 1000)
            t = (now_ms / 1000.0) % period
            if t < period / 2:
                positions = [0, 0, 0, 0, 0, 0]
            else:
                positions = [400, 0, 1000, 1000, 1000, 1000]
            logger.info(f"Publishing positions: {positions}")
            await client.set_finger_positions_and_speeds(SLAVE_ID, positions, speeds)
            await asyncio.sleep(0.01)
    finally:
        libstark.modbus_close(client)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
