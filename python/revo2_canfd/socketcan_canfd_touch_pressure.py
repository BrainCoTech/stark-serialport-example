import asyncio
import os
import platform
import sys

from canfd_utils import *

if platform.system() != "Linux":
    raise NotImplementedError("SocketCAN is only supported on Linux.")

from socketcan_linux_utils import *


def canfd_send(_slave_id: int, can_id: int, data: list):
    if not socketcan_send_message(can_id, bytes(data)):
        logger.error("Failed to send CANFD message")
        return


def canfd_read(_slave_id: int):
    recv_msg = socketcan_receive_message()
    if recv_msg is None:
        logger.error("No message received")
        return 0, bytes([])

    (can_id, data) = recv_msg
    logger.debug(f"Received CANFD - ID: {can_id:02x}, Data: {bytes(data).hex()}")
    return can_id, data


async def get_and_display_motor_status(client, slave_id):
    logger.debug("get_motor_status")
    status: libstark.MotorStatusData = await client.get_motor_status(slave_id)
    logger.info(f"Finger status: {status.description}")

    thumb = await client.get_single_modulus_touch_summary(slave_id, 0)
    pinky = await client.get_single_modulus_touch_summary(slave_id, 4)
    palm = await client.get_single_modulus_touch_summary(slave_id, 5)
    logger.info(f"Thumb: {thumb}")
    logger.info(f"Pinky: {pinky}")
    logger.info(f"Palm: {palm}")

    touch_summary: list[int] = await client.get_modulus_touch_summary(slave_id)
    logger.info(f"Touch summary: {touch_summary}")
    touch_data: list[int] = await client.get_modulus_touch_data(slave_id)
    logger.info(f"Touch Data: {touch_data}")


async def get_motor_status_periodically(client, slave_id):
    logger.info(f"Motor status monitoring started for device {slave_id:02x}")
    while True:
        try:
            await get_and_display_motor_status(client, slave_id)
            await asyncio.sleep(0.001)
        except Exception as e:
            logger.error(
                f"Error in motor status monitoring for device {slave_id:02x}: {e}"
            )
            await asyncio.sleep(1)


async def setup_touch_sensors(client, slave_id):
    logger.debug("get_touch_sensor_enabled")
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x3F):06b}")

    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    data_type = await client.get_modulus_touch_data_type(slave_id)
    logger.info(f"Modulus Touch Data Type: {data_type}")


async def main():
    """
    Main function: initialize Revo2 dexterous hand and execute control example
    """
    master_id = int(os.getenv("STARK_MASTER_ID", "1"), 0)
    slave_id = int(os.getenv("STARK_SLAVE_ID", "0x7f"), 0)
    client = libstark.PyDeviceContext.init_canfd(master_id)

    socketcan_open()
    libstark.set_can_tx_callback(canfd_send)
    libstark.set_can_rx_callback(canfd_read)

    logger.debug("get_device_info")
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD, Baudrate: {baudrate}")

    if not client.is_touch_pressure():
        logger.error("This example is only for Revo2 Touch Pressure hardware")
        socketcan_close()
        sys.exit(1)

    await setup_touch_sensors(client, slave_id)

    shutdown_event = setup_shutdown_event(logger)
    reader_task = asyncio.create_task(get_motor_status_periodically(client, slave_id))

    await shutdown_event.wait()
    logger.info("Shutdown event received, stopping motor status monitoring...")
    reader_task.cancel()
    socketcan_close()


if __name__ == "__main__":
    asyncio.run(main())
