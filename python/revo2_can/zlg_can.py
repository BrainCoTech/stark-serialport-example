import asyncio
import sys
import platform
from typing import Optional
from can_utils import (
    logger, libstark, setup_shutdown_event,
    print_finger_touch_data
)

# Import the corresponding ZLG CAN driver module according to the operating system
if platform.system() == "Windows":
    from zlg_win import *
elif platform.system() == "Linux":
    from zlg_linux import *
else:
    raise NotImplementedError(f"Unsupported operating system: {platform.system()}")

# Note: The following functions are imported from the zlg_win or zlg_linux module through the above conditions:
# - zlgcan_open()
# - zlgcan_close()
# - zlgcan_send_message(can_id, data)
# - zlgcan_receive_message(quick_retries, dely_retries)

class Revo2CanController:
    """Revo2 CAN communication controller"""

    def __init__(self, master_id: int = 1, slave_id: int = 2):
        self.master_id = master_id
        self.slave_id = slave_id
        self.client = None  # Will be initialized in initialize()
        self.device_info = None  # Initialize device info

    async def initialize(self):
        """Initialize CAN connection"""
        try:
            # Initialize ZLG CAN device
            zlgcan_open()  # type: ignore

            # Set callback functions
            libstark.set_can_tx_callback(self._can_send)
            libstark.set_can_rx_callback(self._can_read)

            # Initialize device handler for CAN protocol
            self.client = libstark.init_device_handler(libstark.StarkProtocolType.Can, self.master_id)

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
            if not zlgcan_send_message(can_id, bytes(data)):  # type: ignore
                logger.error("Failed to send CAN message")
                return False
            return True
        except Exception as e:
            logger.error(f"CAN sending exception: {e}")
            return False

    def _can_read(self, _slave_id: int, expected_can_id: int, expected_frames: int) -> tuple:
        """
        CAN message receiving with multi-frame protocol support
        
        SDK tells us how many frames to expect via expected_frames parameter.
        We need to collect all frames and return concatenated data.
        
        Supports multi-frame protocols:
        - MultiRead (0x0B): Detects is_last flag in byte[1] bit 7
        - TouchSensorRead (0x0D): Detects total/seq in byte[0] (total:4bit|seq:4bit)
        
        Args:
            _slave_id: Slave ID (not used)
            expected_can_id: Expected CAN ID (used for filtering)
            expected_frames: Expected frame count (0 = single frame, >0 = multi-frame)
            
        Returns:
            tuple: (can_id, data)
        """
        try:
            result = zlgcan_receive_filtered(expected_can_id, expected_frames)  # type: ignore
            if result is None:
                return 0, bytes([])
            
            can_id, data, frame_count = result
            return can_id, bytes(data)

        except Exception as e:
            logger.error(f"CAN message receiving failed: {e}")
            return 0, bytes([])

    async def get_device_info(self):
        """Get device information"""
        try:
            device_info = await self.client.get_device_info(self.slave_id)
            logger.info(f"Device information: {device_info.description}")
            self.device_info = device_info  # Store device info
            return device_info
        except Exception as e:
            logger.error(f"Failed to get device information: {e}")
            return None
    
    def uses_revo1_touch_api(self) -> bool:
        """Check if device uses Revo1 Touch API"""
        if self.device_info is None:
            return False
        return self.device_info.uses_revo1_touch_api()

    async def change_slave_id(self, new_slave_id: int):
        """Change device slave ID (use with caution, device will reboot)"""
        try:
            await self.client.set_slave_id(self.slave_id, new_slave_id)
            logger.info(f"Slave ID has been changed to {new_slave_id}, device will reboot...")
            return True
        except Exception as e:
            logger.error(f"Failed to change slave ID: {e}")
            return False

    async def configure_device(self):
        """Configure device parameters"""
        try:
            # Option: change slave ID (if needed, uncomment and set new ID)
            # WARNING: device will reboot, program will exit
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
            logger.error(f"Failed to configure device: {e}")
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
            logger.error(f"Failed to configure Turbo mode: {e}")

    async def finger_position_examples(self):
        """Finger position control example"""
        logger.info("=== Finger position control example ===")

        # Example 1: Sequential finger grip
        logger.info("Example 1: Sequential finger grip")
        positions = [
            [200, 0, 0, 0, 0, 0],  # Thumb
            [200, 300, 0, 0, 0, 0],  # Thumb + Index
            [200, 300, 500, 0, 0, 0],  # + Middle
            [200, 300, 500, 700, 0, 0],  # + Ring
            [200, 300, 500, 700, 800, 0],  # + Pinky
            [200, 300, 500, 700, 800, 900],  # + Wrist
        ]

        for i, pos in enumerate(positions):
            logger.info(f"Step {i+1}: {pos}")
            await self.client.set_finger_positions(self.slave_id, pos)
            await asyncio.sleep(0.8)

        await asyncio.sleep(1.0)

        # Example 2: Preset gestures sequence
        logger.info("Example 2: Preset gestures")
        gestures = {
            "Open hand": [0, 0, 0, 0, 0, 0],
            "Point": [0, 300, 0, 0, 0, 0],
            "Victory gesture": [0, 300, 800, 0, 0, 0],
            "OK gesture": [500, 300, 800, 0, 0, 0],
            "Grip": [500, 300, 1000, 1000, 1000, 1000],
        }

        for gesture_name, positions in gestures.items():
            logger.info(f"Execute gesture: {gesture_name} - {positions}")
            await self.client.set_finger_positions(self.slave_id, positions)
            await asyncio.sleep(1.5)

        # Example 3: Grab action simulation
        logger.info("Example 3: Grab action simulation")
        grab_sequence = [
            ([0, 0, 0, 0, 0, 0], "Initial position"),
            ([300, 200, 600, 800, 800, 800], "Grab preparation"),
            ([500, 400, 900, 1000, 1000, 1000], "Grab completion"),
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
        logger.info("All fingers stopped")

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
        )  # Maximum forward speed
        await asyncio.sleep(1.0)
        await self.client.set_finger_speed(
            self.slave_id, finger_id, -1000
        )  # Maximum reverse speed
        await asyncio.sleep(1.0)
        await self.client.set_finger_speed(self.slave_id, finger_id, 0)  # Stop

    async def get_motor_status(self) -> libstark.MotorStatusData:
        """Get motor status"""
        try:
            status = await self.client.get_motor_status(self.slave_id)
            # Optional: enable detailed status logging
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
                status = await self.get_motor_status()
                logger.debug(f"Motor status: {status.description}")
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

            # # 2. Calibrate touch sensors
            # logger.info("2. Calibrate touch sensors for all fingers...")
            # await self.client.touch_sensor_calibrate(self.slave_id, 0xFF)
            # await asyncio.sleep(2.0)

            # # 3. Reset touch sensors
            # logger.info("3. Reset touch sensors for all fingers...")
            # await self.client.touch_sensor_reset(self.slave_id, 0xFF)
            # await asyncio.sleep(0.5)

            logger.info("Touch sensor control test completed\n")
        except Exception as e:
            logger.error(f"Touch sensor control test failed: {e}")

    async def test_touch_sensor_reading(self):
        """Test touch sensor data reading"""
        logger.info("\n=== Test touch sensor data reading ===")

        try:
            touch_data = await self.client.get_touch_sensor_status(self.slave_id)
            logger.info(f"Successfully read touch data for {len(touch_data)} fingers")

            finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
            uses_revo1 = self.uses_revo1_touch_api()

            for i, finger in enumerate(touch_data):
                if i < len(finger_names):
                    if uses_revo1:
                        print_finger_touch_data_revo1(finger, i, finger_names[i])
                    else:
                        print_finger_touch_data_revo2(finger, finger_names[i])

            logger.info("\nTouch sensor data reading test completed\n")
        except Exception as e:
            logger.error(f"Touch sensor data reading failed: {e}")

    async def monitor_touch_sensor(self, iterations: int = 1000, interval: float = 0.5):
        """Periodic touch sensor data monitoring"""
        logger.info(f"\n=== Periodic touch sensor monitoring ({iterations} iterations) ===")

        finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        uses_revo1 = self.uses_revo1_touch_api()

        for i in range(iterations):
            try:
                touch_data = await self.client.get_touch_sensor_status(self.slave_id)

                logger.info(f"\n[{i + 1}] Touch data snapshot:")
                for idx, finger in enumerate(touch_data):
                    if idx < len(finger_names):
                        if uses_revo1:
                            print_finger_touch_data_revo1(finger, idx, finger_names[idx])
                        else:
                            print_finger_touch_data_revo2(finger, finger_names[idx])

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

        # Check if device uses Revo1 Touch API
        if self.uses_revo1_touch_api():
            logger.info("Detected Revo1 Touch device, running touch sensor demos...")
            # await self.test_touch_sensor_control()
            await self.test_touch_sensor_reading()
            await self.monitor_touch_sensor(iterations=1000, interval=0.5)
        else:
            # Enable the following demos for other devices
            await self.finger_position_examples()
            await asyncio.sleep(1.0)

            # await self.finger_speed_examples()
            # await asyncio.sleep(1.0)

            # await self.single_finger_control_example(libstark.FingerId.Pinky)

    def cleanup(self):
        """Clean up resources"""
        try:
            zlgcan_close()  # type: ignore
            logger.info("CAN connection closed")
        except Exception as e:
            logger.error(f"Error cleaning up resources: {e}")

    async def close(self):
        """Async close method for compatibility"""
        self.cleanup()


async def main():
    """Main function"""
    # Default left hand is 1, right hand is 2
    controller = Revo2CanController(master_id=1, slave_id=2)

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

        # Optional: start motor status monitoring (disabled to avoid CAN bus conflicts)
        # monitor_task = asyncio.create_task(controller.monitor_motor_status())
        # tasks.append(monitor_task)

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
