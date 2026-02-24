#!/usr/bin/env python3
"""
Communication Frequency Test - Python Version

Tests communication frequency and performance with auto-detection support.
Supports: Modbus (RS485), CAN 2.0, CANFD, SocketCAN

Run:
    python comm_frequency_test.py              # Auto-detect, interactive menu
    python comm_frequency_test.py 1            # Run specific test (1-4)
    python comm_frequency_test.py 0            # Run all tests
    python comm_frequency_test.py -h           # Show help

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
from dataclasses import dataclass
from typing import List, Optional, Dict, Any

# Setup path and imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_imports import sdk, check_sdk, get_hw_type_name, get_protocol_display_name, logger, baudrate_to_int

check_sdk()


@dataclass
class TestConfig:
    """Test configuration"""
    test_duration: float = 5.0       # Single test duration (seconds)
    max_test_count: int = 10000      # Maximum test count
    min_interval_ms: float = 0.0     # Minimum interval (milliseconds)
    target_frequency: float = 1000.0 # Target frequency (Hz)
    stability_duration: float = 5.0  # Stability test duration (seconds)
    sample_interval: float = 1.0     # Sample interval (seconds)


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
    port_name: str
    slave_id: int
    baudrate: int
    data_baudrate: int = 0

    def print_header(self):
        print(f"Connection:       {self.protocol} via {self.adapter_type}")
        print(f"Port:             {self.port_name}")
        print(f"Slave ID:         0x{self.slave_id:02X} ({self.slave_id})")
        # Convert baudrate to actual bps value
        # Note: Baudrate enum's int() returns index (0-6), not bps value
        baud = baudrate_to_int(self.baudrate) if hasattr(self.baudrate, 'int_value') else self.baudrate
        try:
            data_baud = int(self.data_baudrate)
        except (TypeError, ValueError):
            data_baud = 0
        if data_baud > 0:
            print(f"Baudrate:         {baud / 1_000_000:.0f} Mbps / {data_baud / 1_000_000:.0f} Mbps")
        elif baud >= 1_000_000:
            print(f"Baudrate:         {baud / 1_000_000:.0f} Mbps")
        elif baud > 0:
            print(f"Baudrate:         {baud} bps")
        else:
            print(f"Baudrate:         N/A")


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
        import platform
        print(f"\n{'=' * 60}")
        print(f"📊 {self.function_name} frequency test report")
        print("=" * 60)
        print(f"System:           {platform.system()} {platform.release()}")
        print(f"Language:         Python {platform.python_version()}")
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


def detect_adapter_type(port_name: str, protocol_type) -> str:
    """Detect adapter type from port name and protocol"""
    # SocketCAN interface
    if port_name.startswith("can") or port_name.startswith("vcan"):
        return "SocketCAN"
    
    # Check protocol type
    if protocol_type == sdk.StarkProtocolType.Modbus:
        return "USB-RS485"
    elif protocol_type in [sdk.StarkProtocolType.Can, sdk.StarkProtocolType.CanFd]:
        if "ttyUSB" in port_name or "ttyACM" in port_name or "COM" in port_name:
            return "ZQWL USB-CAN"
        return "CAN Adapter"
    return "Unknown"


class CommFrequencyTester:
    """Communication frequency tester"""

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
    print("\n=== Frequency Test Menu ===")
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


async def auto_detect_and_init():
    """Auto-detect device and initialize"""
    logger.info("Auto-detecting devices...")

    devices = await sdk.auto_detect(scan_all=True)

    if not devices:
        logger.error("No devices found")
        return None, None, None

    logger.info(f"Found {len(devices)} device(s):")
    for i, dev in enumerate(devices):
        hw_name = get_hw_type_name(dev.hardware_type) if dev.hardware_type else "Unknown"
        print(f"\n[{i + 1}] {hw_name}")
        print(f"    Protocol: {get_protocol_display_name(dev.protocol_type)}")
        print(f"    Port: {dev.port_name}")
        print(f"    Slave ID: 0x{dev.slave_id:02X} ({dev.slave_id})")

    # Select device
    if len(devices) > 1:
        try:
            choice = int(input(f"\nSelect device [1-{len(devices)}]: ").strip())
            if choice < 1 or choice > len(devices):
                logger.error("Invalid selection")
                return None, None, None
            device = devices[choice - 1]
        except ValueError:
            device = devices[0]
    else:
        logger.info("Using the only available device")
        device = devices[0]

    # Initialize context
    ctx = await sdk.init_from_detected(device)

    # Build connection info
    connection_info = ConnectionInfo(
        adapter_type=detect_adapter_type(device.port_name, device.protocol_type),
        protocol=get_protocol_display_name(device.protocol_type),
        port_name=device.port_name,
        slave_id=device.slave_id,
        baudrate=device.baudrate,
        data_baudrate=device.data_baudrate if hasattr(device, 'data_baudrate') else 0,
    )

    return ctx, device.slave_id, connection_info


async def main():
    """Main function"""
    import argparse

    parser = argparse.ArgumentParser(description="Communication Frequency Test")
    parser.add_argument("test_num", nargs="?", type=int, help="Test number (0=all, 1-4=specific)")
    args = parser.parse_args()

    print("=== Communication Frequency Test ===\n")

    ctx, slave_id, connection_info = await auto_detect_and_init()
    if ctx is None:
        return

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
        await sdk.close_device_handler(ctx)
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
