"""
Revo1AdvancedTouch CAN Protocol + DataCollector High-Performance Data Collection Example
Supports both ZLG and ZQWL CAN adapters

Features:
1. High-performance data collection using DataCollector
2. Motor status data collection (recommended 100Hz)
3. Touch sensor data collection (recommended 10Hz, limited by CAN multi-frame transmission)
4. Batch reading from buffers
5. Support for multiple CAN adapter types (ZLG/ZQWL)

⚠️ CAN Protocol Limitations:
- Touch data collection requires 15 CAN communications (5 fingers × 3 data types)
- Single touch read takes approximately 45-90ms
- Recommended configuration: motor_frequency=100Hz, touch_frequency=10Hz

Hardware Support:
- ✅ Revo1AdvancedTouch (CAN 2.0)

Usage:
    # Use ZLG adapter (auto-detect OS)
    python revo1_touch_collector.py --adapter zlg

    # Use ZQWL adapter (Windows only)
    python revo1_touch_collector.py --adapter zqwl

    # Use ZQWL with custom settings
    python revo1_touch_collector.py --adapter zqwl --device-type 4 --channel 0 --baudrate 1000000
"""

import asyncio
import sys
import argparse
from pathlib import Path
from can_utils import *
from can_adapter import create_adapter, CANAdapter


class Revo1AdvancedTouchCanCollector:
    """Revo1AdvancedTouch CAN Communication Controller (with DataCollector)"""

    def __init__(
        self,
        adapter: CANAdapter,
        master_id: int = 1,
        slave_id: int = 2,
    ):
        """
        Initialize collector

        Args:
            adapter: CAN adapter instance (ZLG or ZQWL)
            master_id: Master device ID
            slave_id: Slave device ID
        """
        self.adapter = adapter
        self.master_id = master_id
        self.slave_id = slave_id
        self.client = libstark.PyDeviceContext()
        self.device_info = None

        # DataCollector related
        self.motor_buffer = None
        self.touch_buffer = None
        self.collector = None

    async def initialize(self):
        """Initialize CAN connection"""
        try:
            # Open CAN adapter
            if not self.adapter.open():
                logger.error("Failed to open CAN adapter")
                return False

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
            if not self.adapter.send_message(can_id, bytes(data)):
                logger.error("Failed to send CAN message")
                return False
            return True
        except Exception as e:
            logger.error(f"CAN send exception: {e}")
            return False

    def _can_read(self, _slave_id: int) -> tuple:
        """CAN message receiving"""
        try:
            recv_msg = self.adapter.receive_message()
            if recv_msg is None:
                return 0, bytes([])

            can_id, data = recv_msg
            return can_id, data

        except Exception as e:
            logger.error(f"CAN receive exception: {e}")
            return 0, bytes([])

    async def get_device_info(self):
        """Get device information"""
        try:
            device_info = await self.client.get_device_info(self.slave_id)
            logger.info(f"Device info: {device_info.description}")
            self.device_info = device_info
            return device_info
        except Exception as e:
            logger.error(f"Failed to get device info: {e}")
            return None

    def is_revo1_advanced_touch(self) -> bool:
        """Check if device is Revo1AdvancedTouch"""
        if not hasattr(self, "device_info") or self.device_info is None:
            return False
        return (
            self.device_info.hardware_type
            == libstark.StarkHardwareType.Revo1AdvancedTouch
        )

    async def initialize_touch_sensor(self):
        """Initialize touch sensor"""
        logger.info("\n=== Initialize Touch Sensor ===")

        try:
            # 1. Enable touch sensors
            logger.info("Enabling touch sensors for all fingers...")
            await self.client.touch_sensor_setup(self.slave_id, 0xFF)
            await asyncio.sleep(0.5)

            # 2. Calibrate touch sensors
            logger.info("Calibrating touch sensors for all fingers (takes 2 seconds)...")
            await self.client.touch_sensor_calibrate(self.slave_id, 0xFF)
            await asyncio.sleep(2.0)

            # 3. Reset touch sensors
            logger.info("Resetting touch sensors for all fingers...")
            await self.client.touch_sensor_reset(self.slave_id, 0xFF)
            await asyncio.sleep(0.5)

            logger.info("Touch sensor initialization completed\n")
            return True

        except Exception as e:
            logger.error(f"Touch sensor initialization failed: {e}")
            return False

    async def create_data_collector(self):
        """Create DataCollector"""
        logger.info("\n=== Create DataCollector ===")

        try:
            # Create shared buffers
            self.motor_buffer = libstark.MotorStatusBuffer(max_size=1000)
            self.touch_buffer = libstark.TouchStatusBuffer(max_size=1000)

            # Configure collector
            # ⚠️ CAN protocol limitation: touch data collection takes 45-90ms
            # Recommended configuration: motor_frequency=100Hz, touch_frequency=10Hz
            motor_frequency = 100  # 100Hz motor collection (10ms interval)
            touch_frequency = 10   # 10Hz touch collection (100ms interval)

            logger.info("DataCollector configuration:")
            logger.info(f"  - Motor collection frequency: {motor_frequency}Hz ({1000//motor_frequency}ms interval)")
            logger.info(f"  - Touch collection frequency: {touch_frequency}Hz ({1000//touch_frequency}ms interval)")
            logger.info(f"  - Touch collection ratio: 1 touch collection per {motor_frequency//touch_frequency} motor collections")
            logger.info(f"  - Performance stats: Enabled")

            # Create capacitive touch collector
            self.collector = libstark.DataCollector.new_capacitive(
                ctx=self.client.ctx,
                motor_buffer=self.motor_buffer,
                touch_buffer=self.touch_buffer,
                slave_id=self.slave_id,
                motor_frequency=motor_frequency,
                touch_frequency=touch_frequency,
                enable_stats=True,
            )

            logger.info("DataCollector created successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to create DataCollector: {e}")
            return False

    async def run_data_collection(self, duration: int = 30):
        """Run data collection"""
        logger.info(f"\nStarting DataCollector...")
        self.collector.start()
        logger.info("DataCollector started (running in background thread)")
        logger.info("Please touch the sensor to see touch data...\n")

        # Wait for data collection
        await asyncio.sleep(2)

        # Process data
        logger.info(f"=== Start Data Processing ({duration} seconds) ===\n")

        finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

        for i in range(duration):
            await asyncio.sleep(1)

            # Read motor data
            motor_data = self.motor_buffer.pop_all()

            # Read touch data (all fingers)
            touch_data_all = self.touch_buffer.pop_all()
            touch_counts = [len(data) for data in touch_data_all]

            logger.info(
                f"[{i+1:2d}s] Motor data: {len(motor_data)} items, Touch data: {touch_counts} items"
            )

            # Display touch data details (if available)
            has_touch_data = any(len(data) > 0 for data in touch_data_all)

            if has_touch_data:
                for finger_idx, finger_data in enumerate(touch_data_all):
                    if len(finger_data) > 0:
                        latest = finger_data[-1]

                        # Detect contact (normal force threshold > 50)
                        is_contact = latest.normal_force1 > 50

                        if is_contact:
                            logger.info(
                                f"  {finger_names[finger_idx]}: "
                                f"Normal={latest.normal_force1}, "
                                f"Tangential={latest.tangential_force1}, "
                                f"Self-prox={latest.self_proximity1} "
                                f"[CONTACT]"
                            )

        # Stop collection
        logger.info("\nStopping DataCollector...")
        self.collector.stop()
        logger.info("DataCollector stopped")

        # Display final statistics
        logger.info("\n=== Final Data Statistics ===")
        final_motor_count = self.motor_buffer.len()
        final_touch_counts = self.touch_buffer.len_all()

        logger.info("Remaining data in buffers:")
        logger.info(f"  - Motor data: {final_motor_count} items")
        logger.info(f"  - Touch data: {final_touch_counts} items")

    def cleanup(self):
        """Clean up resources"""
        try:
            if self.collector and self.collector.is_running():
                self.collector.stop()

            self.adapter.close()
            logger.info("Resources cleaned up")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Revo1AdvancedTouch CAN Touch Data Collector",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use ZLG adapter (auto-detect OS)
  python revo1_touch_collector.py --adapter zlg

  # Use ZQWL adapter (Windows only)
  python revo1_touch_collector.py --adapter zqwl

  # Use ZQWL with custom settings
  python revo1_touch_collector.py --adapter zqwl --device-type 4 --baudrate 1000000
        """,
    )

    parser.add_argument(
        "--adapter",
        type=str,
        choices=["zlg", "zqwl"],
        default="zlg",
        help="CAN adapter type (default: zlg)",
    )

    parser.add_argument(
        "--master-id",
        type=int,
        default=1,
        help="Master device ID (default: 1)",
    )

    parser.add_argument(
        "--slave-id",
        type=int,
        default=2,
        help="Slave device ID (default: 2)",
    )

    parser.add_argument(
        "--duration",
        type=int,
        default=30,
        help="Data collection duration in seconds (default: 30)",
    )

    # ZQWL specific options
    parser.add_argument(
        "--device-type",
        type=int,
        default=4,
        help="ZQWL device type (default: 4 for USBCAN-2E-U)",
    )

    parser.add_argument(
        "--channel",
        type=int,
        default=0,
        help="ZQWL channel number (default: 0)",
    )

    parser.add_argument(
        "--baudrate",
        type=int,
        default=1000000,
        help="ZQWL baud rate in bps (default: 1000000)",
    )

    return parser.parse_args()


async def main():
    """Main function"""
    args = parse_arguments()

    # Create CAN adapter
    try:
        if args.adapter == "zlg":
            adapter = create_adapter("zlg")
        else:  # zqwl
            adapter = create_adapter(
                "zqwl",
                device_type=args.device_type,
                channel=args.channel,
                baudrate=args.baudrate,
            )
    except Exception as e:
        logger.error(f"Failed to create adapter: {e}")
        return

    # Create controller
    controller = Revo1AdvancedTouchCanCollector(
        adapter=adapter,
        master_id=args.master_id,
        slave_id=args.slave_id,
    )

    try:
        # Initialize connection
        if not await controller.initialize():
            logger.error("Initialization failed, exiting program")
            return

        # Get device information
        device_info = await controller.get_device_info()
        if not device_info:
            logger.error("Failed to get device info, exiting program")
            return

        # Verify device type
        if not controller.is_revo1_advanced_touch():
            logger.error("Error: Current device is not Revo1AdvancedTouch")
            logger.error(f"Current device type: {device_info.hardware_type}")
            return

        # Initialize touch sensor
        if not await controller.initialize_touch_sensor():
            logger.error("Touch sensor initialization failed, exiting program")
            return

        # Create DataCollector
        if not await controller.create_data_collector():
            logger.error("Failed to create DataCollector, exiting program")
            return

        # Run data collection
        await controller.run_data_collection(duration=args.duration)

        logger.info("\nExample program completed")

    except KeyboardInterrupt:
        logger.info("\nUser interrupted")

    except Exception as e:
        logger.error(f"Program execution exception: {e}", exc_info=True)

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
