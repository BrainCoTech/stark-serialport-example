#!/usr/bin/env python3
"""
Communication Frequency Test - ZLG CAN Version

Tests communication frequency and performance with ZLG USB-CANFD adapter.
Linux only.

Run:
    python comm_frequency_test_zlg.py              # Interactive menu, default slave_id=0x7E
    python comm_frequency_test_zlg.py 1            # Run specific test (1-4)
    python comm_frequency_test_zlg.py 0            # Run all tests
    python comm_frequency_test_zlg.py -s 0x7F      # Custom slave_id (right hand)
    python comm_frequency_test_zlg.py -h           # Show help

Test modes:
    1. get_motor_status read frequency
    2. set_finger_positions write frequency
    3. Mixed function frequency (read + control)
    4. Long-term stability test
"""

import asyncio
import sys
import os
import time
import statistics
import argparse
import platform
from dataclasses import dataclass
from typing import List, Optional, Dict, Any

# Setup path and imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_imports import sdk, check_sdk, logger

check_sdk()

# Check platform
if platform.system() != "Linux":
    print("Error: ZLG CAN is only supported on Linux")
    sys.exit(1)


@dataclass
class TestConfig:
    """Test configuration"""
    test_duration: float = 5.0
    max_test_count: int = 10000
    min_interval_ms: float = 0.0
    target_frequency: float = 1000.0
    stability_duration: float = 5.0
    sample_interval: float = 1.0


@dataclass
class TestResult:
    """Single test result"""
    success: bool
    elapsed_ms: float
    timestamp: float
    error_msg: Optional[str] = None


@dataclass
class ConnectionInfo:
    """Connection info for test report"""
    adapter_type: str
    protocol: str
    slave_id: int
    baudrate: int
    data_baudrate: int

    def print_header(self):
        print(f"Connection:       {self.protocol} via {self.adapter_type}")
        print(f"Slave ID:         0x{self.slave_id:02X} ({self.slave_id})")
        print(f"Baudrate:         {self.baudrate // 1_000_000} Mbps / {self.data_baudrate // 1_000_000} Mbps")


@dataclass
class FrequencyTestReport:
    """Frequency test report"""
    function_name: str
    connection: ConnectionInfo
    total_tests: int
    successful_tests: int
    failed_tests: int
    success_rate: float
    avg_latency_ms: float
    max_latency_ms: float
    min_latency_ms: float
    std_dev_ms: float
    actual_frequency_hz: float
    target_frequency_hz: float
    total_duration_secs: float

    def print_report(self):
        print(f"\n{'=' * 60}")
        print(f"📊 {self.function_name} frequency test report")
        print("=" * 60)
        self.connection.print_header()
        print()
        print(f"Total tests:      {self.total_tests}")
        print(f"Successful tests: {self.successful_tests}")
        print(f"Failed tests:     {self.failed_tests}")
        print(f"Success rate:     {self.success_rate:.1f}%")
        print()
        print(f"Average latency:  {self.avg_latency_ms:.2f} ms")
        print(f"Minimum latency:  {self.min_latency_ms:.2f} ms")
        print(f"Maximum latency:  {self.max_latency_ms:.2f} ms")
        print(f"Std deviation:    {self.std_dev_ms:.2f} ms")
        print()
        print(f"Actual frequency: {self.actual_frequency_hz:.1f} Hz")
        print(f"Target frequency: {self.target_frequency_hz:.1f} Hz")
        print(f"Achievement rate: {self.actual_frequency_hz / self.target_frequency_hz * 100:.1f}%")
        print(f"Test duration:    {self.total_duration_secs:.1f} s")


class CommFrequencyTester:
    """Communication frequency tester for ZLG"""

    def __init__(self, ctx, slave_id: int, connection_info: ConnectionInfo, config: TestConfig):
        self.ctx = ctx
        self.slave_id = slave_id
        self.connection_info = connection_info
        self.config = config

    async def test_get_motor_status_frequency(self) -> FrequencyTestReport:
        """Test 1: get_motor_status read frequency"""
        print("\n📊 Starting get_motor_status read frequency test...")

        results = []
        start_time = time.perf_counter()
        test_count = 0

        while (time.perf_counter() - start_time) < self.config.test_duration and test_count < self.config.max_test_count:
            test_start = time.perf_counter()

            try:
                await self.ctx.get_motor_status(self.slave_id)
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                results.append(TestResult(True, elapsed_ms, time.perf_counter() - start_time))
            except Exception as e:
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                results.append(TestResult(False, elapsed_ms, time.perf_counter() - start_time, str(e)))

            if test_count % 100 == 0:
                logger.info(f"  Test {test_count}, latency: {results[-1].elapsed_ms:.2f}ms")

            test_count += 1

            if self.config.min_interval_ms > 0:
                await asyncio.sleep(self.config.min_interval_ms / 1000)

        total_duration = time.perf_counter() - start_time
        return self._generate_report("get_motor_status", results, total_duration, self.config.target_frequency)

    async def test_set_finger_positions_frequency(self) -> FrequencyTestReport:
        """Test 2: set_finger_positions write frequency"""
        print("\n📊 Starting set_finger_positions write frequency test...")

        results = []
        start_time = time.perf_counter()
        test_count = 0

        position_sequences = [
            [400, 400, 1000, 1000, 1000, 1000],
            [400, 400, 0, 0, 0, 0],
            [400, 400, 500, 500, 500, 500],
            [400, 400, 0, 0, 0, 0],
        ]

        while (time.perf_counter() - start_time) < self.config.test_duration and test_count < self.config.max_test_count:
            test_start = time.perf_counter()
            positions = position_sequences[test_count % len(position_sequences)]

            try:
                await self.ctx.set_finger_positions(self.slave_id, positions)
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                results.append(TestResult(True, elapsed_ms, time.perf_counter() - start_time))
            except Exception as e:
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                results.append(TestResult(False, elapsed_ms, time.perf_counter() - start_time, str(e)))

            if test_count % 50 == 0:
                logger.info(f"  Test {test_count}, latency: {results[-1].elapsed_ms:.2f}ms")

            test_count += 1

            interval = max(self.config.min_interval_ms, 5.0)
            await asyncio.sleep(interval / 1000)

        total_duration = time.perf_counter() - start_time
        return self._generate_report("set_finger_positions", results, total_duration, 50.0)


    async def test_mixed_frequency(self) -> FrequencyTestReport:
        """Test 3: Mixed function frequency (read + control)"""
        print("\n📊 Starting mixed function frequency test...")

        results = []
        start_time = time.perf_counter()
        test_count = 0

        position_sequences = [
            [400, 400, 0, 0, 0, 0],
            [400, 400, 300, 300, 300, 300],
            [400, 400, 600, 600, 600, 600],
            [400, 400, 1000, 1000, 1000, 1000],
        ]

        while (time.perf_counter() - start_time) < self.config.test_duration and test_count < self.config.max_test_count:
            test_start = time.perf_counter()

            try:
                if test_count % 5 == 0:
                    positions = position_sequences[(test_count // 5) % len(position_sequences)]
                    await self.ctx.set_finger_positions(self.slave_id, positions)
                else:
                    await self.ctx.get_motor_status(self.slave_id)

                elapsed_ms = (time.perf_counter() - test_start) * 1000
                results.append(TestResult(True, elapsed_ms, time.perf_counter() - start_time))
            except Exception as e:
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                results.append(TestResult(False, elapsed_ms, time.perf_counter() - start_time, str(e)))

            if test_count % 100 == 0:
                action = "position" if test_count % 5 == 0 else "status"
                logger.info(f"  Test {test_count}, latency: {results[-1].elapsed_ms:.2f}ms, action: {action}")

            test_count += 1
            await asyncio.sleep(self.config.min_interval_ms / 1000)

        total_duration = time.perf_counter() - start_time
        return self._generate_report("mixed_functions", results, total_duration, self.config.target_frequency)

    async def test_stability(self) -> Dict[str, Any]:
        """Test 4: Long-term stability test"""
        print(f"\n📊 Starting stability test ({self.config.stability_duration:.0f}s)...")

        stability_results = {
            'samples': [],
            'avg_latencies': [],
            'error_rates': [],
            'timestamps': []
        }

        start_time = time.perf_counter()
        sample_count = 0

        while (time.perf_counter() - start_time) < self.config.stability_duration:
            sample_start = time.perf_counter()
            sample_results = []

            while (time.perf_counter() - sample_start) < self.config.sample_interval:
                test_start = time.perf_counter()

                try:
                    await self.ctx.get_motor_status(self.slave_id)
                    elapsed_ms = (time.perf_counter() - test_start) * 1000
                    sample_results.append((True, elapsed_ms))
                except Exception:
                    elapsed_ms = (time.perf_counter() - test_start) * 1000
                    sample_results.append((False, elapsed_ms))

                await asyncio.sleep(0.01)

            successful = [e for s, e in sample_results if s]
            total_count = len(sample_results)
            success_count = len(successful)

            avg_latency = statistics.mean(successful) if successful else 0
            error_rate = (total_count - success_count) / total_count * 100 if total_count > 0 else 0

            stability_results['samples'].append(total_count)
            stability_results['avg_latencies'].append(avg_latency)
            stability_results['error_rates'].append(error_rate)
            stability_results['timestamps'].append(time.perf_counter() - start_time)

            sample_count += 1
            if sample_count % 5 == 0:
                logger.info(f"  Sample {sample_count}, latency: {avg_latency:.2f}ms, error rate: {error_rate:.1f}%")

        return stability_results

    def _generate_report(self, function_name: str, results: List[TestResult],
                         total_duration: float, target_frequency: float) -> FrequencyTestReport:
        """Generate test report"""
        successful = [r for r in results if r.success]
        total_tests = len(results)
        successful_tests = len(successful)
        failed_tests = total_tests - successful_tests
        success_rate = (successful_tests / total_tests * 100) if total_tests > 0 else 0

        if successful:
            latencies = [r.elapsed_ms for r in successful]
            avg_latency = statistics.mean(latencies)
            max_latency = max(latencies)
            min_latency = min(latencies)
            std_dev = statistics.stdev(latencies) if len(latencies) > 1 else 0
        else:
            avg_latency = max_latency = min_latency = std_dev = 0

        actual_frequency = total_tests / total_duration if total_duration > 0 else 0

        return FrequencyTestReport(
            function_name=function_name,
            connection=self.connection_info,
            total_tests=total_tests,
            successful_tests=successful_tests,
            failed_tests=failed_tests,
            success_rate=success_rate,
            avg_latency_ms=avg_latency,
            max_latency_ms=max_latency,
            min_latency_ms=min_latency,
            std_dev_ms=std_dev,
            actual_frequency_hz=actual_frequency,
            target_frequency_hz=target_frequency,
            total_duration_secs=total_duration,
        )


    async def run_test(self, test_num: int):
        """Run specific test"""
        if test_num == 1:
            report = await self.test_get_motor_status_frequency()
            report.print_report()
        elif test_num == 2:
            report = await self.test_set_finger_positions_frequency()
            report.print_report()
        elif test_num == 3:
            report = await self.test_mixed_frequency()
            report.print_report()
        elif test_num == 4:
            stability = await self.test_stability()
            self._print_stability_results(stability)
        else:
            print(f"Invalid test number: {test_num}")

    async def run_all_tests(self):
        """Run all tests"""
        reports = []

        report1 = await self.test_get_motor_status_frequency()
        report1.print_report()
        reports.append(report1)

        report2 = await self.test_set_finger_positions_frequency()
        report2.print_report()
        reports.append(report2)

        report3 = await self.test_mixed_frequency()
        report3.print_report()
        reports.append(report3)

        stability = await self.test_stability()
        self._print_stability_results(stability)

        print(f"\n{'=' * 60}")
        print("📊 Test Summary")
        print("=" * 60)
        for report in reports:
            print(f"{report.function_name}: {report.actual_frequency_hz:.1f} Hz ({report.success_rate:.1f}% success)")

    def _print_stability_results(self, stability: Dict[str, Any]):
        """Print stability test results"""
        print(f"\n{'=' * 60}")
        print("📊 Stability Test Results")
        print("=" * 60)
        self.connection_info.print_header()
        print()

        if stability['avg_latencies']:
            overall_avg = statistics.mean(stability['avg_latencies'])
            overall_error = statistics.mean(stability['error_rates'])
            max_error = max(stability['error_rates'])
            print(f"Average latency:    {overall_avg:.2f} ms")
            print(f"Average error rate: {overall_error:.2f}%")
            print(f"Maximum error rate: {max_error:.2f}%")
            print(f"Sample count:       {len(stability['samples'])}")
        else:
            print("No stability data collected")


def show_menu() -> Optional[int]:
    """Show interactive menu"""
    print("\n=== ZLG CAN Frequency Test Menu ===")
    print("1. get_motor_status read frequency")
    print("2. set_finger_positions write frequency")
    print("3. Mixed function frequency")
    print("4. Stability test")
    print("0. Run all tests (1-4)")
    print("q. Quit")

    try:
        choice = input("\nSelect test: ").strip().lower()
        if choice == 'q':
            return None
        return int(choice)
    except ValueError:
        return -1


def prompt_slave_id() -> int:
    """Prompt for slave ID"""
    print("\nSelect slave ID:")
    print("  1. 0x7E (126) - Left hand")
    print("  2. 0x7F (127) - Right hand")
    print("  3. Custom")

    try:
        choice = input("Choice [1]: ").strip()
        if choice == "" or choice == "1":
            return 0x7E
        elif choice == "2":
            return 0x7F
        elif choice == "3":
            id_str = input("Enter slave ID (hex or decimal): ").strip()
            if id_str.startswith("0x") or id_str.startswith("0X"):
                return int(id_str, 16)
            return int(id_str)
        else:
            return 0x7E
    except ValueError:
        return 0x7E


async def init_zlg_canfd(slave_id: int):
    """Initialize ZLG CANFD device"""
    try:
        # Import ZLG CANFD adapter
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "revo2_canfd"))
        from zlg_canfd import Revo2CanfdController

        controller = Revo2CanfdController(master_id=1, slave_id=slave_id)
        if not await controller.initialize():
            logger.error("Failed to initialize ZLG CANFD adapter")
            return None, None

        # Verify device connection
        print("\nVerifying device connection...")
        info = await controller.get_device_info()
        if info:
            print(f"✅ Device connected: {info}")
        else:
            logger.error("❌ Failed to get device info")
            return None, None

        return controller.client, controller

    except ImportError as e:
        logger.error(f"Failed to import ZLG adapter: {e}")
        logger.error("Make sure zlg_canfd.py exists in examples/python/revo2_canfd/")
        return None, None
    except Exception as e:
        logger.error(f"ZLG CANFD init failed: {e}")
        return None, None


async def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="ZLG CAN Communication Frequency Test")
    parser.add_argument("test_num", nargs="?", type=int, help="Test number (0=all, 1-4=specific)")
    parser.add_argument("-s", "--slave", type=str, default=None, help="Slave ID (hex or decimal)")
    args = parser.parse_args()

    print("=== ZLG CAN Communication Frequency Test ===\n")

    # Get slave ID
    if args.slave:
        if args.slave.startswith("0x") or args.slave.startswith("0X"):
            slave_id = int(args.slave, 16)
        else:
            slave_id = int(args.slave)
    else:
        slave_id = prompt_slave_id()

    print(f"\nUsing slave ID: 0x{slave_id:02X} ({slave_id})")

    # Initialize ZLG CANFD
    print("\nInitializing ZLG CANFD device...")
    ctx, controller = await init_zlg_canfd(slave_id)
    if ctx is None:
        print("\nTroubleshooting:")
        print("  - Check ZLG USB-CAN device is connected")
        print("  - Check CAN cable connection (CAN_H, CAN_L, GND)")
        print("  - Verify device is powered on")
        print("  - Check slave ID setting")
        return

    # Create connection info
    connection_info = ConnectionInfo(
        adapter_type="ZLG USB-CANFD",
        protocol="CANFD",
        slave_id=slave_id,
        baudrate=1_000_000,
        data_baudrate=5_000_000,
    )

    config = TestConfig()
    tester = CommFrequencyTester(ctx, slave_id, connection_info, config)

    try:
        if args.test_num is not None:
            if args.test_num == 0:
                await tester.run_all_tests()
            else:
                await tester.run_test(args.test_num)
        else:
            # Interactive menu
            while True:
                choice = show_menu()
                if choice is None:
                    break
                elif choice == 0:
                    await tester.run_all_tests()
                elif 1 <= choice <= 4:
                    await tester.run_test(choice)
                else:
                    print("Invalid choice")
    finally:
        if controller:
            await controller.close()
        logger.info("Device connection closed")

    print("\n=== Test completed ===")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nUser interrupted")
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
