"""
Revo1AdvancedTouch CAN Protocol + DataCollector High-Performance Data Collection Example
ZLG CAN Adapter Version

This is a convenience wrapper that uses ZLG adapter by default.
For a unified version supporting both ZLG and ZQWL, use revo1_touch_collector.py

Features:
1. High-performance data collection using DataCollector
2. Motor status data collection (recommended 100Hz)
3. Touch sensor data collection (recommended 10Hz, limited by CAN multi-frame transmission)
4. Batch reading from buffers
5. ZLG CAN adapter support (Windows/Linux)

⚠️ CAN Protocol Limitations:
- Touch data collection requires 15 CAN communications (5 fingers × 3 data types)
- Single touch read takes approximately 45-90ms
- Recommended configuration: motor_frequency=100Hz, touch_frequency=10Hz

Hardware Support:
- ✅ Revo1AdvancedTouch (CAN 2.0)
- ✅ ZLG CAN adapters (Windows/Linux)

Usage:
    # Use default settings (auto-detect OS)
    python zlg_touch_collector.py

    # Specify slave ID and collection duration
    python zlg_touch_collector.py --slave-id 2 --duration 60
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
        description="Revo1AdvancedTouch CAN Touch Data Collector (ZLG Adapter)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default settings
  python zlg_touch_collector.py

  # Specify slave ID and collection duration
  python zlg_touch_collector.py --slave-id 2 --duration 60
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

    return parser.parse_args()


async def main():
    """Main function"""
    args = parse_arguments()

    # Create ZLG adapter (auto-detect OS)
    try:
        adapter = create_adapter("zlg")
        logger.info(f"Using ZLG CAN adapter")
    except Exception as e:
        logger.error(f"Failed to create ZLG adapter: {e}")
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
