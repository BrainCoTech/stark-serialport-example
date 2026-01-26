import asyncio
import sys

from zqwl_win import *
from can_utils import *


class Revo2CanController:
    """Revo2 CAN communication controller"""

    def __init__(self, master_id: int = 1, slave_id: int = 1):
        self.master_id = master_id
        self.slave_id = slave_id
        self.client = libstark.PyDeviceContext()
        self.device_info = None  # Initialize device info

    async def initialize(
        self, device_type: int = 42, channel: int = 0, baudrate: int = 1000000
    ):
        """Initialize CAN connection"""
        try:
            # Initialize ZCAN device
            zcan_open(device_type=device_type, channel=channel, baudrate=baudrate)

            # Set callback function
            libstark.set_can_tx_callback(self._can_send)
            libstark.set_can_rx_callback(self._can_read)

            logger.info(
                f"CAN connection initialized successfully - Master ID: {self.master_id}, Slave ID: {self.slave_id}"
            )
            return True

        except Exception as e:
            logger.error(f"CAN connection initialization failed: {e}")
            return False

    def _can_send(self, slave_id: int, can_id: int, data: list) -> bool:
        """CAN message send"""
        try:
            if not zcan_send_message(slave_id, can_id, bytes(data)):
                logger.error("Send CAN message failed")
                return False
            return True
        except Exception as e:
            logger.error(f"CAN send exception: {e}")
            return False

    def _can_read(self, slave_id: int) -> tuple:
        """CAN message receive"""
        try:
            recv_msg = zqwl_can_receive_message()
            if recv_msg is None:
                return 0, bytes([])

            can_id, data = recv_msg
            # Optional: enable detailed debug log
            # logger.debug(f"接收CAN - ID: {can_id:029b}, Data: {bytes(data).hex()}")
            return can_id, data

        except Exception as e:
            logger.error(f"CAN receive exception: {e}")
            return 0, bytes([])

    async def get_device_info(self):
        """Get device information"""
        try:
            device_info = await self.client.get_device_info(self.slave_id)
            logger.info(f"Device information: {device_info.description}")
            self.device_info = device_info  # Store device info
            return device_info
        except Exception as e:
            logger.error(f"Get device information failed: {e}")
            return None
    
    def is_revo1_advanced_touch(self) -> bool:
        """Check if device is Revo1AdvancedTouch"""
        if not hasattr(self, 'device_info') or self.device_info is None:
            return False
        return self.device_info.hardware_type == libstark.StarkHardwareType.Revo1AdvancedTouch

    async def change_slave_id(self, new_slave_id: int):
        """Change device slave ID (use with caution, device will restart)"""
        try:
            await self.client.set_slave_id(self.slave_id, new_slave_id)
            logger.info(f"Slave ID has been changed to {new_slave_id}, device will restart...")
            return True
        except Exception as e:
            logger.error(f"Change slave ID failed: {e}")
            return False

    async def configure_device(self):
        """Configure device parameters"""
        try:
            # Option: change slave ID (if needed, uncomment and set new ID)
            # WARNING: Device will restart, program will exit
            # if await self.change_slave_id(new_slave_id=2):
            #     sys.exit(0)

            # Disable automatic calibration and perform manual calibration
            # await self.client.set_auto_calibration(self.slave_id, False) # Disable automatic calibration after power-on
            # await self.client.calibrate_position(self.slave_id)
            # await self.client.set_auto_calibration(self.slave_id, True)  # Enable automatic calibration after power-on
            # auto_calibration_enabled = await self.client.get_auto_calibration_enabled(self.slave_id)
            # logger.info(f"Automatic calibration after power-on: {auto_calibration_enabled}")

            # Configure: Turbo mode (optional, uncomment to enable)
            # await self.configure_turbo_mode()

            return True
        except Exception as e:
            logger.error(f"Configure device failed: {e}")
            return False

    async def configure_turbo_mode(self):
        """Configure Turbo mode"""
        try:
            # Enable Turbo mode
            await self.client.set_turbo_mode_enabled(self.slave_id, True)

            # Set Turbo parameters
            turbo_interval = 200  # Grip interval time (milliseconds)
            turbo_duration = 300  # Grip duration time (milliseconds)
            turbo_conf = libstark.TurboConfig(turbo_interval, turbo_duration)
            await self.client.set_turbo_config(self.slave_id, turbo_conf)

            # Verify configuration
            turbo_mode_enabled = await self.client.get_turbo_mode_enabled(self.slave_id)
            turbo_config = await self.client.get_turbo_config(self.slave_id)

            logger.info(f"Turbo mode: {turbo_mode_enabled}")
            logger.info(
                f"Turbo configuration - interval: {turbo_config.interval}ms, duration: {turbo_config.duration}ms"
            )

        except Exception as e:
            logger.error(f"Configure Turbo mode failed: {e}")

    async def finger_position_examples(self):
        """Finger position control example"""
        logger.info("=== Finger position control example ===")

        # Example 1: perform grip action one by one
        logger.info("Example 1: perform grip action one by one")
        positions = [
            [200, 0, 0, 0, 0, 0],  # Thumb
            [200, 300, 0, 0, 0, 0],  # Thumb + Index
            [200, 300, 500, 0, 0, 0],  # + Middle
            [200, 300, 500, 700, 0, 0],  # + Ring
            [200, 300, 500, 700, 800, 0],  # + Pinky
            [200, 300, 500, 700, 800, 900],  # + Wrist
        ]

        for i, pos in enumerate(positions):
            logger.info(f"步骤 {i+1}: {pos}")
            await self.client.set_finger_positions(self.slave_id, pos)
            await asyncio.sleep(0.8)

        await asyncio.sleep(1.0)

        # Example 2: perform gesture
        logger.info("Example 2: perform gesture")
        gestures = {
            "Open hand": [0, 0, 0, 0, 0, 0],
            "Point gesture": [0, 300, 0, 0, 0, 0],
            "Victory gesture": [0, 300, 800, 0, 0, 0],
            "OK gesture": [500, 300, 800, 0, 0, 0],
            "Grip gesture": [500, 300, 1000, 1000, 1000, 1000],
        }

        for gesture_name, positions in gestures.items():
            logger.info(f"Perform gesture: {gesture_name} - {positions}")
            await self.client.set_finger_positions(self.slave_id, positions)
            await asyncio.sleep(1.5)

        # Example 3: perform grab action simulation
        logger.info("Example 3: perform grab action simulation")
        grab_sequence = [
            ([0, 0, 0, 0, 0, 0], "Initial position"),
            ([300, 400, 600, 800, 800, 0], "Grab preparation"),
            ([500, 400, 900, 1000, 1000, 0], "Grab completed"),
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
        logger.info("All fingers stop movement")

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
        )  # Positive maximum speed
        await asyncio.sleep(1.0)
        await self.client.set_finger_speed(
            self.slave_id, finger_id, -1000
        )  # Negative maximum speed
        await asyncio.sleep(1.0)
        await self.client.set_finger_speed(self.slave_id, finger_id, 0)  # Stop

    async def get_motor_status(self) -> libstark.MotorStatusData:
        """Get motor status"""
        try:
            status = await self.client.get_motor_status(self.slave_id)
            # Optional: enable detailed status log
            # logger.info(f"Motor status: {status.description}")
            return status
        except Exception as e:
            logger.error(f"Get motor status failed: {e}")
            return None  # type: ignore

    async def monitor_motor_status(self, interval: float = 0.001):
        """Continuous monitoring of motor status"""
        logger.info(f"Start monitoring device {self.slave_id:02x} motor status")

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

    async def test_touch_sensor_control(self):
        """Test touch sensor control (Revo1AdvancedTouch)"""
        logger.info("=== Test touch sensor control ===")

        try:
            # 1. Enable touch sensors for all fingers
            logger.info("1. Enable touch sensors for all fingers...")
            await self.client.touch_sensor_setup(self.slave_id, 0xFF)
            await asyncio.sleep(0.5)

            # 2. Calibrate touch sensors
            logger.info("2. Calibrate touch sensors for all fingers...")
            await self.client.touch_sensor_calibrate(self.slave_id, 0xFF)
            await asyncio.sleep(2.0)

            # 3. Reset touch sensors
            logger.info("3. Reset touch sensors for all fingers...")
            await self.client.touch_sensor_reset(self.slave_id, 0xFF)
            await asyncio.sleep(0.5)

            logger.info("Touch sensor control test completed\n")
        except Exception as e:
            logger.error(f"Touch sensor control test failed: {e}")

    def print_finger_touch_data(self, finger, channel: int, finger_name: str):
        """Print touch data for a single finger"""
        logger.info(f"\n--- {finger_name} (Channel {channel}) ---")

        # Display different data groups based on finger type
        if channel == 0:
            # Thumb: 2 force groups, 1 self-proximity
            logger.info(f"  Force Group 1: Normal={finger.normal_force1}, Tangential={finger.tangential_force1}, Direction={finger.tangential_direction1}°")
            logger.info(f"  Force Group 2: Normal={finger.normal_force2}, Tangential={finger.tangential_force2}, Direction={finger.tangential_direction2}°")
            logger.info(f"  Self-proximity: {finger.self_proximity1}")
        elif channel in [1, 2, 3]:
            # Index/Middle/Ring: 3 force groups, 2 self-proximity, 1 mutual-proximity
            logger.info(f"  Force Group 1: Normal={finger.normal_force1}, Tangential={finger.tangential_force1}, Direction={finger.tangential_direction1}°")
            logger.info(f"  Force Group 2: Normal={finger.normal_force2}, Tangential={finger.tangential_force2}, Direction={finger.tangential_direction2}°")
            logger.info(f"  Force Group 3: Normal={finger.normal_force3}, Tangential={finger.tangential_force3}, Direction={finger.tangential_direction3}°")
            logger.info(f"  Self-proximity 1: {finger.self_proximity1}, Self-proximity 2: {finger.self_proximity2}")
            logger.info(f"  Mutual-proximity: {finger.mutual_proximity}")
        elif channel == 4:
            # Pinky: 2 force groups, 1 self-proximity, 1 mutual-proximity
            logger.info(f"  Force Group 1: Normal={finger.normal_force1}, Tangential={finger.tangential_force1}, Direction={finger.tangential_direction1}°")
            logger.info(f"  Force Group 2: Normal={finger.normal_force2}, Tangential={finger.tangential_force2}, Direction={finger.tangential_direction2}°")
            logger.info(f"  Self-proximity: {finger.self_proximity1}")
            logger.info(f"  Mutual-proximity: {finger.mutual_proximity}")

        # Display status
        status_map = {0: "Normal", 1: "Data Error", 2: "Communication Error"}
        status_str = status_map.get(finger.status, "Unknown")
        logger.info(f"  Status: {finger.status} ({status_str})")

    async def test_touch_sensor_reading(self):
        """Test touch sensor data reading (Revo1AdvancedTouch)"""
        logger.info("\n=== Test touch sensor data reading ===")

        try:
            touch_data = await self.client.get_touch_sensor_status(self.slave_id)
            logger.info(f"Successfully read touch data for {len(touch_data)} fingers")

            finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

            for i, finger in enumerate(touch_data):
                if i < len(finger_names):
                    self.print_finger_touch_data(finger, i, finger_names[i])

            logger.info("\nTouch sensor data reading test completed\n")
        except Exception as e:
            logger.error(f"Touch sensor data reading failed: {e}")

    async def monitor_touch_sensor(self, iterations: int = 10, interval: float = 0.5):
        """Periodic touch sensor data monitoring (Revo1AdvancedTouch)"""
        logger.info(f"\n=== Periodic touch sensor monitoring ({iterations} iterations) ===")

        finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

        for i in range(iterations):
            try:
                touch_data = await self.client.get_touch_sensor_status(self.slave_id)

                logger.info(f"\n[{i + 1}] Touch data snapshot:")
                for idx, finger in enumerate(touch_data):
                    if idx < len(finger_names):
                        logger.info(
                            f"  {finger_names[idx]}: Normal={finger.normal_force1:4}, "
                            f"Tangential={finger.tangential_force1:4}, "
                            f"Self-prox={finger.self_proximity1:8}, Status={finger.status}"
                        )

                await asyncio.sleep(interval)
            except Exception as e:
                logger.error(f"[{i + 1}] Reading failed: {e}")

        logger.info("\nPeriodic monitoring completed\n")

    async def demo_task(self):
        """Demo task"""
        # Get device information first
        await self.get_device_info()

        # Configure device parameters, enable as needed
        # await self.configure_device()

        # Check if device is Revo1AdvancedTouch
        if self.is_revo1_advanced_touch():
            logger.info("Detected Revo1AdvancedTouch device, running touch sensor demos...")
            # await self.test_touch_sensor_control()
            await self.test_touch_sensor_reading()
            await self.monitor_touch_sensor(iterations=10, interval=0.5)
        else:
            # Enable the following demos for other devices
            await self.finger_position_examples()
            await asyncio.sleep(1.0)

            await self.finger_speed_examples()
            await asyncio.sleep(1.0)

            await self.single_finger_control_example(libstark.FingerId.Pinky)

    def cleanup(self):
        """Clean up resources"""
        try:
            zcan_close()
            logger.info("CAN connection closed")
        except Exception as e:
            logger.error(f"Error cleaning up resources: {e}")


async def main():
    """Main function"""
    # Default left hand is 1, right hand is 2
    controller = Revo2CanController(master_id=1, slave_id=2)

    try:
        # Initialize connection
        if not await controller.initialize():
            logger.error("Initialization failed, program exit")
            return

        # Set shutdown event listener
        shutdown_event = setup_shutdown_event(logger)

        # Get device information
        await controller.get_device_info()

        # Create tasks
        tasks = []
        # Start demo task
        # demo_task = asyncio.create_task(controller.demo_task())
        # tasks.append(demo_task)

        # # Optional: start motor status monitoring
        # monitor_task = asyncio.create_task(controller.monitor_motor_status())
        # tasks.append(monitor_task)

        # Wait for shutdown signal
        await shutdown_event.wait()
        logger.info("Shutdown signal received, stop all tasks")

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
