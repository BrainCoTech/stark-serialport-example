import asyncio
import sys
import platform
from canfd_utils import *

# Import corresponding ZLG CAN driver module according to operating system
if platform.system() == "Windows":
    from zlg_win import (
        zlgcan_open,
        zlgcan_close,
        zlgcan_send_message,
        zlgcan_receive_message,
    )
elif platform.system() == "Linux":
    from zlg_linux import (
        zlgcan_open,
        zlgcan_close,
        zlgcan_send_message,
        zlgcan_receive_message,
    )
else:
    raise NotImplementedError(f"Unsupported operating system: {platform.system()}")

def canfd_send(_slave_id: int, can_id: int, data: list):
    # logger.debug(f"Sending CAN ID: {can_id}, Data: {data}, type: {type(data)}")
    if not zlgcan_send_message(can_id, bytes(data)):
        logger.error("Failed to send CANFD message")
        return

def canfd_read(_slave_id: int):
    recv_msg = zlgcan_receive_message()
    if recv_msg is None:
        logger.error("No message received")
        return 0, bytes([])

    (can_id, data) = recv_msg
    logger.debug(f"Received CANFD - ID: {can_id:02x}, Data: {bytes(data).hex()}")
    return can_id, data

async def get_and_display_motor_status(client, slave_id):
    """
    Get and display motor status information

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    logger.debug("get_motor_status")
    status: libstark.MotorStatusData = await client.get_motor_status(slave_id)

    # Display detailed status information
    # logger.info(f"positions: {list(status.positions)}")  # Position
    # logger.info(f"speeds: {list(status.speeds)}")        # Speed
    # logger.info(f"currents: {list(status.currents)}")    # Current
    # logger.info(f"states: {list(status.states)}")        # State
    logger.info(f"Finger status: {status.description}")

    # Get single finger touch sensor status (optional way)
    thumb = await client.get_single_modulus_touch_summary(slave_id, 0)
    pinky = await client.get_single_modulus_touch_summary(slave_id, 4)
    palm = await client.get_single_modulus_touch_summary(slave_id, 5)
    logger.info(f"Thumb: {thumb}")
    logger.info(f"Pinky: {pinky}")
    logger.info(f"Palm: {palm}")

    # Get all finger touch sensor status
    touch_summary: list[int] = await client.get_modulus_touch_summary(slave_id)
    logger.info(f"Touch summary: {touch_summary}")
    touch_data: list[int] = await client.get_modulus_touch_data(slave_id)
    logger.info(f"Touch Data: {touch_data}")

async def get_motor_status_periodically(client, slave_id):
    logger.info(f"Motor status monitoring started for device {slave_id:02x}")
    while True:
        try:
            await get_and_display_motor_status(client, slave_id)
            # Add delay to avoid too frequent queries
            await asyncio.sleep(0.001)
            # break

        except Exception as e:
            logger.error(
                f"Error in motor status monitoring for device {slave_id:02x}: {e}"
            )
            await asyncio.sleep(1)  # Wait longer before retrying when an error occurs

async def setup_touch_sensors(client, slave_id):
    """
    Configure and enable touch sensors

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # logger.debug("setup_touch_sensors")
    # bits = 0x3F  # 0x3F: Enable five fingers + palm
    # await client.touch_sensor_setup(slave_id, bits)
    # await asyncio.sleep(1)  # 等待触觉传感器准备就绪

    # Verify sensor enabled status
    logger.debug("get_touch_sensor_enabled")
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x3F):06b}")

    # Get touch sensor firmware version (can only be obtained after enabling)
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    # await client.get_modulus_touch_data_type(slave_id, libstark.ModulusTouchDataType.Calibrated)
    data_tpye = await client.get_modulus_touch_data_type(slave_id)
    logger.info(f"Modulus Touch Data Type: {data_tpye}")

async def main():
    # fmt: off
    """
    Main function: initialize Revo2 dexterous hand and execute control example
    """
    # Connect Revo2 device
    master_id = 1
    slave_id = 0x7e # Default left hand ID is 0x7e, right hand ID is 0x7f
    client = libstark.init_device_handler(libstark.StarkProtocolType.CanFd, master_id)

    zlgcan_open()
    libstark.set_can_tx_callback(canfd_send)
    libstark.set_can_rx_callback(canfd_read)

    logger.debug("get_device_info")  # Get device information
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD, Baudrate: {baudrate}")

    if not client.uses_pressure_touch_api(slave_id):
        logger.error("This example is only for Revo2 Touch Pressure hardware")
        zlgcan_close()
        sys.exit(1)

    # Configure touch sensors
    await setup_touch_sensors(client, slave_id)

    shutdown_event = setup_shutdown_event(logger)

    # Get motor status
    reader_task = asyncio.create_task(get_motor_status_periodically(client, slave_id))

    await shutdown_event.wait()  # Wait for shutdown event
    logger.info("Shutdown event received, stopping motor status monitoring...")
    reader_task.cancel()  # Cancel motor status monitoring task
    # Clean up resources
    zlgcan_close()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
