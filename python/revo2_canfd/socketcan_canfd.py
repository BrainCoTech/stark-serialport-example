import asyncio
import os
import platform
from canfd_utils import *

if platform.system() != "Linux":
    raise NotImplementedError("SocketCAN is only supported on Linux.")

from socketcan_linux_utils import *


def canfd_send(_slave_id: int, can_id: int, data: list):
    if not socketcan_send_message(can_id, bytes(data)):
        logger.error("Failed to send CANFD message")


def canfd_read(_slave_id: int):
    recv_msg = socketcan_receive_message(quick_retries=5, dely_retries=2)
    if recv_msg is None:
        logger.error("No message received")
        return 0, bytes([])

    can_id, data = recv_msg
    logger.debug(f"Received CANFD - ID: {can_id:02x}, Data: {data.hex()}")
    return can_id, data


async def configure_control_mode(client, slave_id):
    logger.debug("set_finger_unit_mode")
    await client.set_finger_unit_mode(
        slave_id, libstark.FingerUnitMode.Normalized
    )
    finger_unit_mode = await client.get_finger_unit_mode(slave_id)
    logger.info(f"Finger unit mode: {finger_unit_mode}")


async def configure_finger_parameters(client, slave_id):
    finger_id = libstark.FingerId.Middle
    _ = finger_id


async def execute_control_examples(client, slave_id):
    finger_id = libstark.FingerId.Index
    await single_finger_control_examples(client, slave_id, finger_id)
    await all_fingers_control_examples(client, slave_id)


async def single_finger_control_examples(client, slave_id, finger_id):
    await client.set_finger_pwm(slave_id, finger_id, -1000)
    await asyncio.sleep(1.0)
    await client.set_finger_position_with_millis(slave_id, finger_id, 1000, 300)
    await asyncio.sleep(1.0)


async def all_fingers_control_examples(client, slave_id):
    positions = [200, 200] + [0] * 4
    durations = [300] * 6
    await client.set_finger_positions_and_durations(slave_id, positions, durations)
    await asyncio.sleep(1.0)

    positions = [500, 500] + [1000] * 4
    speeds = [1000] * 6
    await client.set_finger_positions_and_speeds(slave_id, positions, speeds)
    await asyncio.sleep(1.0)


async def get_and_display_motor_status(client, slave_id):
    status: libstark.MotorStatusData = await client.get_motor_status(slave_id)
    logger.info(f"Finger status: {status.description}")


async def get_motor_status_periodically(client, slave_id):
    logger.info(f"Motor status monitoring started for device {slave_id:02x}")
    while True:
        try:
            await get_and_display_motor_status(client, slave_id)
            await asyncio.sleep(0.001)
        except Exception as e:
            logger.error(f"Error in motor status monitoring for device {slave_id:02x}: {e}")
            await asyncio.sleep(1)


async def main():
    """
    Main function: Initialize Revo2 dexterous hand and execute control examples
    """
    master_id = int(os.getenv("STARK_MASTER_ID", "1"), 0)
    slave_id = int(os.getenv("STARK_SLAVE_ID", "0x7f"), 0)
    client = libstark.init_device_handler(libstark.StarkProtocolType.CanFd, master_id)

    socketcan_open()
    libstark.set_can_tx_callback(canfd_send)
    libstark.set_can_rx_callback(canfd_read)

    logger.debug("get_device_info")
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD, Baudrate: {baudrate}")

    await configure_control_mode(client, slave_id)
    await configure_finger_parameters(client, slave_id)

    shutdown_event = setup_shutdown_event(logger)
    reader_task = asyncio.create_task(get_motor_status_periodically(client, slave_id))

    await execute_control_examples(client, slave_id)

    await shutdown_event.wait()
    logger.info("Shutdown event received, stopping motor status monitoring...")
    reader_task.cancel()
    socketcan_close()


if __name__ == "__main__":
    asyncio.run(main())
