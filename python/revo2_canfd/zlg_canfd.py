import asyncio
import sys
import platform
from canfd_utils import *
from typing import Callable, Tuple, Optional, Any

# Import corresponding ZLG CAN driver module according to operating system
# These functions are imported from platform-specific modules
zlgcan_open: Callable[[], None]
zlgcan_close: Callable[[], None]
zlgcan_send_message: Callable[[int, bytes], bool]
zlgcan_receive_message: Callable[..., Optional[Tuple[int, bytes]]]

if platform.system() == "Windows":
    from zlg_win import zlgcan_open, zlgcan_close, zlgcan_send_message, zlgcan_receive_message  # type: ignore
elif platform.system() == "Linux":
    from zlg_linux import zlgcan_open, zlgcan_close, zlgcan_send_message, zlgcan_receive_message, zlgcan_receive_canfd_filtered  # type: ignore
else:
    raise NotImplementedError(f"Unsupported operating system: {platform.system()}")


class Revo2CanfdController:
    """Revo2 CANFD communication controller for use with common_init.py"""

    def __init__(self, master_id: int = 1, slave_id: int = 0x7e):
        self.master_id = master_id
        self.slave_id = slave_id
        self.client = None
        self.device_info = None

    async def initialize(self):
        """Initialize CANFD connection"""
        try:
            zlgcan_open()
            libstark.set_can_tx_callback(self._canfd_send)
            libstark.set_can_rx_callback(self._canfd_read)
            self.client = libstark.init_device_handler(libstark.StarkProtocolType.CanFd, self.master_id)
            logger.info(f"CANFD connection initialized - Master ID: {self.master_id}, Slave ID: {self.slave_id}")
            return True
        except Exception as e:
            logger.error(f"CANFD initialization failed: {e}")
            return False

    def _canfd_send(self, _slave_id: int, can_id: int, data: list) -> bool:
        if not zlgcan_send_message(can_id, bytes(data)):
            logger.error("Failed to send CANFD message")
            return False
        return True

    def _canfd_read(self, _slave_id: int, expected_can_id: int, expected_frames: int) -> tuple:
        result = zlgcan_receive_canfd_filtered(expected_can_id, expected_frames)
        if result is None:
            return 0, bytes([])
        can_id, data, _frame_count = result
        return can_id, bytes(data)

    async def get_device_info(self):
        """Get device information"""
        try:
            self.device_info = await self.client.get_device_info(self.slave_id)
            logger.info(f"Device: {self.device_info.description}")
            return self.device_info
        except Exception as e:
            logger.error(f"Failed to get device info: {e}")
            return None

    def cleanup(self):
        """Cleanup resources"""
        zlgcan_close()

    async def close(self):
        """Async close method for compatibility"""
        self.cleanup()


def canfd_send(_slave_id: int, can_id: int, data: list) -> None:
    # logger.debug(f"Sending CAN ID: {can_id}, Data: {data}, type: {type(data)}")
    if not zlgcan_send_message(can_id, bytes(data)):
        logger.error("Failed to send CANFD message")
        return


def canfd_read(_slave_id: int, expected_can_id: int, expected_frames: int):
    """
    CANFD message receiving with slave_id/master_id filtering
    
    CANFD CAN ID format: (slave_id << 16) | (master_id << 8) | payload_len
    Matches slave_id and master_id from CAN ID (not exact CAN ID match).
    
    Args:
        _slave_id: Slave ID (not used)
        expected_can_id: Expected CAN ID to filter responses
        expected_frames: Expected frame count (0=auto-detect, >0=specific count)
        
    Returns:
        tuple: (can_id, data)
    """
    result = zlgcan_receive_canfd_filtered(expected_can_id, expected_frames)
    if result is None:
        return 0, bytes([])

    can_id, data, _frame_count = result
    logger.debug(f"Received CANFD - ID: 0x{can_id:02x}, Data: {bytes(data).hex()}")
    return can_id, bytes(data)


async def configure_control_mode(client, slave_id):
    """
    Configure finger control mode

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Set finger control parameter unit mode
    logger.debug("set_finger_unit_mode")
    await client.set_finger_unit_mode(
        slave_id, libstark.FingerUnitMode.Normalized
    )  # Normalized mode
    # await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Physical)  # Physical mode

    # Get and verify finger control mode
    logger.debug("get_finger_unit_mode")
    finger_unit_mode = await client.get_finger_unit_mode(slave_id)
    logger.info(f"Finger unit mode: {finger_unit_mode}")

    # 参考文档：https://www.brainco-hz.com/docs/revolimb-hand/revo2/modbus_foundation.html#%E5%8D%95%E4%BD%8D%E6%A8%A1%E5%BC%8F%E8%AE%BE%E7%BD%AE-937


async def configure_finger_parameters(client, slave_id):
    """
    Configure finger parameters (optional)

    Set the maximum angle, minimum angle, maximum speed, maximum current, protected current, etc. of the finger.
    The range of each finger parameter is detailed in the official documentation.

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    finger_id = libstark.FingerId.Middle  # Select middle finger as example

    # Set minimum position (angle)
    # await client.set_finger_min_position(slave_id, finger_id, 0)
    # min_position = await client.get_finger_min_position(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] min position: {min_position}")

    # Set maximum position (angle)
    # await client.set_finger_max_position(slave_id, finger_id, 80)
    # max_position = await client.get_finger_max_position(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max position: {max_position}")

    # Set maximum speed
    # await client.set_finger_max_speed(slave_id, finger_id, 130)
    # max_speed = await client.get_finger_max_speed(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max speed: {max_speed}")

    # Set maximum current
    # await client.set_finger_max_current(slave_id, finger_id, 1000)
    # max_current = await client.get_finger_max_current(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max current: {max_current}")

    # Set protected current
    # await client.set_finger_protected_current(slave_id, finger_id, 500)
    # protected_current = await client.get_finger_protected_current(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] protected current: {protected_current}")


async def execute_control_examples(client, slave_id):
    """
    Execute various control examples

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    finger_id = libstark.FingerId.Index  # Select index finger as example

    # Single finger control example
    await single_finger_control_examples(client, slave_id, finger_id)

    # All fingers control example
    await all_fingers_control_examples(client, slave_id)


async def single_finger_control_examples(client, slave_id, finger_id):
    """
    Single finger control example

    Args:
        client: Modbus client instance
        slave_id: Device ID
        finger_id: Finger ID
    """
    # Speed Mode: positive value for closing direction, negative value for opening direction
    # await client.set_finger_speed(slave_id, finger_id, 500)  # Range: -1000 ~ 1000
    # await asyncio.sleep(1.0)  # Wait for the finger to reach the target position

    # Current Mode: positive value for closing direction, negative value for opening direction
    # await client.set_finger_current(slave_id, finger_id, -300)  # Range: -1000 ~ 1000
    # await asyncio.sleep(1.0)  # Wait for the finger to reach the target position

    # PWM Mode: positive value for closing direction, negative value for opening direction
    await client.set_finger_pwm(slave_id, finger_id, -1000)  # Range: -1000 ~ 1000
    await asyncio.sleep(1.0)  # Wait for the finger to reach the target position

    # Position Mode: target position + expected time
    await client.set_finger_position_with_millis(slave_id, finger_id, 1000, 300)
    await asyncio.sleep(1.0)  # Wait for the finger to reach the target position

    # Position Mode: target position + expected speed
    # await client.set_finger_position_with_speed(slave_id, finger_id, 1, 300)
    # await asyncio.sleep(1.0)  # Wait for the finger to reach the target position


async def all_fingers_control_examples(client, slave_id):
    """
    All fingers control example

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    # Speed Mode: positive value for closing direction, negative value for opening direction
    # await client.set_finger_speeds(slave_id, [500] * 6)
    # await asyncio.sleep(1.0)  # Wait for the finger to reach the target position

    # Current Mode: positive value for closing direction, negative value for opening direction
    # await client.set_finger_currents(slave_id, [-300] * 6)
    # await asyncio.sleep(1.0)  # Wait for the finger to reach the target position

    # PWM Mode: positive value for closing direction, negative value for opening direction
    # await client.set_finger_pwms(slave_id, [700] * 6)
    # await asyncio.sleep(1.0)  # Wait for the finger to reach the target position

    # Position Mode: target position + expected time
    positions = [200, 200] + [0] * 4
    durations = [300] * 6  # Expected time to reach the target position (milliseconds)
    await client.set_finger_positions_and_durations(slave_id, positions, durations)
    await asyncio.sleep(1.0)  # Wait for the finger to reach the target position

    # Position Mode: target position + expected speed
    positions = [500, 500] + [1000] * 4
    speeds = [1000] * 6  # Expected speed to reach the target position
    await client.set_finger_positions_and_speeds(slave_id, positions, speeds)
    await asyncio.sleep(1.0)  # Wait for the finger to reach the target position


async def get_and_display_motor_status(client, slave_id):
    """
    Get and display motor status information

    Args:
        client: Modbus client instance
        slave_id: Device ID
    """
    logger.debug("get_motor_status")
    status = await client.get_motor_status(slave_id)

    # Display detailed status information
    # logger.info(f"positions: {list(status.positions)}")  # Position
    # logger.info(f"speeds: {list(status.speeds)}")        # Speed
    # logger.info(f"currents: {list(status.currents)}")    # Current
    # logger.info(f"states: {list(status.states)}")        # State
    logger.info(f"Finger status: {status.description}")


async def get_motor_status_periodically(client, slave_id, shutdown_event=None):
    logger.info(f"Motor status monitoring started for device {slave_id:02x}")
    while True:
        if shutdown_event and shutdown_event.is_set():
            break
        try:
            await get_and_display_motor_status(client, slave_id)
            # Add delay to avoid too frequent queries
            await asyncio.sleep(0.01)

        except asyncio.CancelledError:
            break
        except Exception as e:
            logger.error(
                f"Error in motor status monitoring for device {slave_id:02x}: {e}"
            )
            await asyncio.sleep(1)  # When an error occurs, wait longer before retrying


async def main():
    # fmt: off
    """
    Main function: Initialize Revo2 dexterous hand and execute control examples
    """
    # NOTE: Run auto_detect.py first to find actual device ID.
    # Revo2 CANFD: Left hand default ID is 0x7e, right hand default ID is 0x7f.
    master_id = 1
    slave_id = 0x7e
    client = libstark.init_device_handler(libstark.StarkProtocolType.CanFd, master_id)

    zlgcan_open()
    libstark.set_can_tx_callback(canfd_send)
    libstark.set_can_rx_callback(canfd_read)

    logger.debug("get_device_info")  # Get device information
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD, Baudrate: {baudrate}")

    # Configure control mode
    await configure_control_mode(client, slave_id)

    # Configure finger parameters (optional)
    await configure_finger_parameters(client, slave_id)

    shutdown_event = setup_shutdown_event(logger)

    # Get motor status
    reader_task = asyncio.create_task(get_motor_status_periodically(client, slave_id, shutdown_event))

    # Execute control examples
    await execute_control_examples(client, slave_id)

    await shutdown_event.wait()  # Wait for shutdown event
    logger.info("Shutdown event received, stopping motor status monitoring...")
    reader_task.cancel()  # Cancel motor status monitoring task
    try:
        await asyncio.wait_for(reader_task, timeout=1.0)
    except (asyncio.CancelledError, asyncio.TimeoutError):
        pass
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
