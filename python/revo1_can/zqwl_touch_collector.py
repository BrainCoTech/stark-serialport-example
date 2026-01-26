"""
Revo1AdvancedTouch CAN Protocol + DataCollector High-Performance Data Collection Example
ZQWL CAN Adapter Version

This is a convenience wrapper that uses ZQWL adapter by default.
For a unified version supporting both ZLG and ZQWL, use revo1_touch_collector.py

Features:
1. High-performance data collection using DataCollector
2. Motor status data collection (recommended 100Hz)
3. Touch sensor data collection (recommended 10Hz, limited by CAN multi-frame transmission)
4. Batch reading from buffers
5. ZQWL CAN adapter support (Windows only)

⚠️ CAN Protocol Limitations:
- Touch data collection requires 15 CAN communications (5 fingers × 3 data types)
- Single touch read takes approximately 45-90ms
- Recommended configuration: motor_frequency=100Hz, touch_frequency=10Hz

Hardware Support:
- ✅ Revo1AdvancedTouch (CAN 2.0)
- ✅ ZQWL CAN adapters (Windows only)

Usage:
    # Use default settings (device_type=4, channel=0, baudrate=1000000)
    python zqwl_touch_collector.py

    # Use custom settings
    python zqwl_touch_collector.py --device-type 4 --channel 0 --baudrate 1000000
"""

import asyncio
import sys
import argparse
from pathlib import Path
from can_utils import *
from can_adapter import create_adapter


# Import the unified collector class
from revo1_touch_collector import Revo1AdvancedTouchCanCollector


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Revo1AdvancedTouch CAN Touch Data Collector (ZQWL Adapter)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default settings
  python zqwl_touch_collector.py

  # Use custom device type and baudrate
  python zqwl_touch_collector.py --device-type 4 --baudrate 1000000

  # Specify slave ID and collection duration
  python zqwl_touch_collector.py --slave-id 2 --duration 60
        """,
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

    # Create ZQWL adapter
    try:
        adapter = create_adapter(
            "zqwl",
            device_type=args.device_type,
            channel=args.channel,
            baudrate=args.baudrate,
        )
        logger.info(f"Using ZQWL CAN adapter")
    except Exception as e:
        logger.error(f"Failed to create ZQWL adapter: {e}")
        logger.error("Note: ZQWL adapter only supports Windows")
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
