import asyncio
import sys

from zlg_win import *
from can_utils import *


class Revo1CanController:
    """Revo1 CAN communication controller"""

    def __init__(self, master_id: int = 1, slave_id: int = 1):
        self.master_id = master_id
        self.slave_id = slave_id
        self.client = libstark.PyDeviceContext()

    async def initialize(self):
        """Initialize CAN connection"""
        try:
            # Initialize ZLG CAN device
            zlgcan_open()

            # Set callback functions
            libstark.set_can_tx_callback(self._can_send)
            libstark.set_can_rx_callback(self._can_read)

            logger.info(
                f"CAN connection initialized successfully - Master ID: {self.master_id}, Slave ID: {self.slave_id}"
            )
            return True

        except Exception as e:
            logger.error(f"CAN connection initialization failed: {e}")
            return False

    def _can_send(self, _slave_id: int, can_id: int, data: list) -> bool:
        """CAN message sending"""
        try:
            if not zlgcan_send_message(can_id, bytes(data)):
                logger.error("CAN message sending failed")
                return False
            return True
        except Exception as e:
            logger.error(f"CAN message sending failed: {e}")
            return False

    def _can_read(self, _slave_id: int) -> tuple:
        """CAN message receiving"""
        try:
            recv_msg = zlgcan_receive_message()
            if recv_msg is None:
                return 0, bytes([])

            can_id, data = recv_msg
            # Optional: Enable detailed debug logging
            # logger.debug(f"CAN - ID: {can_id:029b}, Data: {bytes(data).hex()}")
            return can_id, data

        except Exception as e:
            logger.error(f"CAN message receiving failed: {e}")
            return 0, bytes([])

    async def get_device_info(self):
        """Get device information"""
        try:
            device_info = await self.client.get_device_info(self.slave_id)
            logger.info(f"Device information: {device_info.description}")
            return device_info
        except Exception as e:
            logger.error(f"Failed to get device information: {e}")
            return None

    async def change_slave_id(self, new_slave_id: int):
        """Modify device slave ID (cautious use, device will restart)"""
        try:
            await self.client.set_slave_id(self.slave_id, new_slave_id)
            logger.info(f"Slave ID modified to {new_slave_id}, device will restart...")
            return True
        except Exception as e:
            logger.error(f"Failed to modify slave ID: {e}")
            return False

    async def configure_device(self):
        """Configure device parameters"""
        try:
            # Option: Modify slave ID (if needed, uncomment and set new ID)
            # WARNING: Device will restart, program will exit
            # if await self.change_slave_id(new_slave_id=2):
            #     sys.exit(0)

            # Disable auto calibration and perform manual calibration
            # await self.client.set_auto_calibration(self.slave_id, False) # Disable auto calibration on boot
            # await self.client.calibrate_position(self.slave_id)
            # await self.client.set_auto_calibration(self.slave_id, True)  # Enable auto calibration on boot
            # auto_calibration_enabled = await self.client.get_auto_calibration_enabled(self.slave_id)
            # logger.info(f"Auto calibration enabled: {auto_calibration_enabled}")

            # Configure: Turbo mode (optional, uncomment to enable)
            # await self.configure_turbo_mode()

            return True
        except Exception as e:
            logger.error(f"Failed to configure device: {e}")
            return False

    async def configure_turbo_mode(self):
        """Configure Turbo mode"""
        try:
            # Enable Turbo mode
            await self.client.set_turbo_mode_enabled(self.slave_id, True)

            # Set Turbo parameters
            turbo_interval = 200  # Grasp interval time (milliseconds)
            turbo_duration = 300  # Grasp duration time (milliseconds)
            turbo_conf = libstark.TurboConfig(turbo_interval, turbo_duration)
            await self.client.set_turbo_config(self.slave_id, turbo_conf)

            # Verify configuration
            turbo_mode_enabled = await self.client.get_turbo_mode_enabled(self.slave_id)
            turbo_config = await self.client.get_turbo_config(self.slave_id)

            logger.info(f"Turbo mode: {turbo_mode_enabled}")
            logger.info(
                f"Turbo configuration - Interval: {turbo_config.interval}ms, Duration: {turbo_config.duration}ms"
            )

        except Exception as e:
            logger.error(f"Failed to configure Turbo mode: {e}")

    async def finger_position_examples(self):
        """Finger position control example"""
        logger.info("=== Finger position control example ===")

        # Example 1: Each finger grip
        logger.info("Example 1: Each finger grip")
        positions = [
            [200, 0, 0, 0, 0, 0],  # Thumb
            [200, 300, 0, 0, 0, 0],  # Thumb + Index finger
            [200, 300, 500, 0, 0, 0],  # + Middle finger
            [200, 300, 500, 700, 0, 0],  # + Ring finger
            [200, 300, 500, 700, 800, 0],  # + Little finger
            [200, 300, 500, 700, 800, 900],  # + Wrist
        ]

        for i, pos in enumerate(positions):
            logger.info(f"Step {i+1}: {pos}")
            await self.client.set_finger_positions(self.slave_id, pos)
            await asyncio.sleep(0.8)

        await asyncio.sleep(1.0)

        # Example 2: Predefined gesture
        logger.info("Example 2: Predefined gestures")
        gestures = {
            "Open hand": [0, 0, 0, 0, 0, 0],
            "Point": [0, 300, 0, 0, 0, 0],
            "Victory gesture": [0, 300, 800, 0, 0, 0],
            "OK gesture": [500, 300, 800, 0, 0, 0],  # Thumb limited to 500 (2 joints)
            "Grip": [500, 300, 1000, 1000, 1000, 1000],  # Thumb limited to 500 (2 joints)
        }

        for gesture_name, positions in gestures.items():
            logger.info(f"Execute gesture: {gesture_name} - {positions}")
            await self.client.set_finger_positions(self.slave_id, positions)
            await asyncio.sleep(1.5)

        # Example 3: Grasp action simulation
        logger.info("Example 3: Grasp action simulation")
        grab_sequence = [
            ([0, 0, 0, 0, 0, 0], "Initial position"),
            ([300, 400, 600, 800, 800, 0], "Grasp preparation"),
            ([600, 700, 900, 1000, 1000, 0], "Grasp completion"),
        ]

        for positions, description in grab_sequence:
            logger.info(f"{description}: {positions}")
            await self.client.set_finger_positions(self.slave_id, positions)
            await asyncio.sleep(1.0)

    async def finger_speed_examples(self):
        """Finger speed control example"""
        logger.info("=== Finger speed control example ===")

        # Stop all movement
        await self.client.set_finger_speeds(self.slave_id, [0] * 6)
        logger.info("All fingers stopped movement")

    async def single_finger_control_example(self, finger_id: libstark.FingerId):
        """Single finger control example"""
        logger.info(f"=== Single finger control example - {finger_id} ===")

        # Position control
        logger.info("Position control test")
        await self.client.set_finger_position(self.slave_id, finger_id, 1000)  # Maximum position
        await asyncio.sleep(1.0)
        await self.client.set_finger_position(self.slave_id, finger_id, 0)  # Initial position
        await asyncio.sleep(1.0)

        # Speed control
        logger.info("Speed control test")
        await self.client.set_finger_speed(
            self.slave_id, finger_id, 1000
        )  # Maximum speed
        await asyncio.sleep(1.0)
        await self.client.set_finger_speed(
            self.slave_id, finger_id, -1000
        )  # Maximum speed
        await asyncio.sleep(1.0)
        await self.client.set_finger_speed(self.slave_id, finger_id, 0)  # Stop

    async def get_motor_status(self) -> libstark.MotorStatusData:
        """Get motor status"""
        try:
            status = await self.client.get_motor_status(self.slave_id)
            # Optional: Enable detailed status logging
            # logger.info(f"Motor status: {status.description}")
            return status
        except Exception as e:
            logger.error(f"Failed to get motor status: {e}")
            return None  # type: ignore

    async def monitor_motor_status(self, interval: float = 0.001):
        """Continuous monitoring of motor status"""
        logger.info(f"Start monitoring motor status of device {self.slave_id:02x}")

        while True:
            try:
                await self.get_motor_status()
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                logger.info("Motor status monitoring cancelled")
                break
            except Exception as e:
                logger.error(f"Motor status monitoring exception: {e}")
                await asyncio.sleep(1)

    async def demo_task(self):
        """Demo task"""
        # Get device information
        await self.get_device_info()

        # Configure device parameters, enable as needed
        # await self.configure_device()

        # Enable the following demos as needed
        await self.finger_position_examples()
        await asyncio.sleep(1.0)

        await self.finger_speed_examples()
        await asyncio.sleep(1.0)

        await self.single_finger_control_example(libstark.FingerId.Pinky)

        # Set the final gesture (packing gesture)
        # await self.client.set_finger_positions(
        #     self.slave_id, [800, 300, 1000, 1000, 1000, 1000]
        # )

    def cleanup(self):
        """Clean up resources"""
        try:
            zlgcan_close()
            logger.info("CAN connection closed")
        except Exception as e:
            logger.error(f"Error cleaning up resources: {e}")


async def main():
    """Main function"""
    controller = Revo1CanController(master_id=1, slave_id=1)

    try:
        # Initialize connection
        if not await controller.initialize():
            logger.error("Initialization failed, program exiting")
            return

        # Set shutdown event listener
        shutdown_event = setup_shutdown_event(logger)

        # Create tasks
        tasks = []

        # Start demo task
        demo_task = asyncio.create_task(controller.demo_task())
        tasks.append(demo_task)

        # Optional: Start motor status monitoring
        monitor_task = asyncio.create_task(controller.monitor_motor_status())
        tasks.append(monitor_task)

        # Wait for shutdown signal
        await shutdown_event.wait()
        logger.info("Received shutdown signal, stopping all tasks...")

        # Cancel all tasks
        for task in tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    except Exception as e:
        logger.error(f"Program execution exception: {e}")

    finally:
        controller.cleanup()
        sys.exit(0)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
