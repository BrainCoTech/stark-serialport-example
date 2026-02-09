"""
Revo2 communication frequency test example

This test is used to evaluate the communication frequency and performance of Revo2 dexterous hand in actual usage scenarios, including:
- get_motor_status read frequency test
- set_finger_positions set frequency test
- Communication function latency analysis
- High-frequency continuous communication stability test
- Communication error rate statistics

Test features:
1. Single function frequency test: continuously call the same API to measure the maximum frequency
2. Mixed function test: simulate read + control calls in actual usage scenarios
3. Long-term stability test: evaluate stability over time
4. Data integrity verification: ensure data accuracy under high-frequency communication

Usage scenarios:
- Real-time control system performance evaluation
- Finger position control frequency test
- Coordination test between status monitoring and control
- System integration test
"""

import asyncio
import time
import statistics
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from revo2_utils import *
import matplotlib.pyplot as plt
import json
import sys

# Test configuration parameters
@dataclass
class TestConfig:
    """Test configuration class"""
    # Basic test parameters
    test_duration: float = 10.0      # Single test duration (seconds)
    max_test_count: int = 1000       # Maximum test count
    timeout_ms: float = 100.0        # Timeout threshold (milliseconds)

    # Frequency test parameters
    target_frequency: int = 500      # Target frequency (Hz)
    min_interval_ms: float = 1.0     # Minimum interval (milliseconds)

    # Stability test parameters
    stability_duration: float = 60.0 # Stability test duration (seconds)
    sample_interval: float = 1.0     # Sampling interval (seconds)

@dataclass
class TestResult:
    """Single test result"""
    success: bool
    elapsed_ms: float
    timestamp: float
    data_size: int = 0
    error_msg: str = ""

@dataclass
class FrequencyTestReport:
    """Frequency test report"""
    function_name: str
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
    total_duration_s: float

class Revo2CommTester:
    """Revo2 communication frequency tester"""

    def __init__(self, config: TestConfig):
        self.config = config
        self.client: Optional[libstark.DeviceContext] = None
        self.slave_id: Optional[int] = None
        self.test_results: Dict[str, List[TestResult]] = {}

    async def initialize(self):
        """Initialize connection"""
        try:
            logger.info("🔌 Connecting to Revo2 device...")
            self.client, self.slave_id = await open_modbus_revo2()
            await self.client.set_finger_unit_mode(slave_id=self.slave_id, mode=libstark.FingerUnitMode.Normalized) # type: ignore
            logger.info(f"✅ Successfully connected to device, slave ID: {self.slave_id}")
            return True
        except Exception as e:
            logger.error(f"❌ Connection failed: {e}")
            return False

    async def cleanup(self):
        """Clean up resources"""
        if self.client:
            libstark.modbus_close(self.client)
            logger.info("🔌 Connection closed")

    async def test_get_motor_status_frequency(self) -> FrequencyTestReport:
        """Test the read frequency of get_motor_status"""
        logger.info("📊 Starting test for get_motor_status read frequency...")

        if not self.client or self.slave_id is None:
            raise RuntimeError("Device not initialized")

        function_name = "get_motor_status"
        results = []
        start_time = time.perf_counter()
        test_count = 0

        while (time.perf_counter() - start_time) < self.config.test_duration and test_count < self.config.max_test_count:
            test_start = time.perf_counter()

            try:
                # Call get_motor_status
                status: libstark.MotorStatusData = await self.client.get_motor_status(self.slave_id)

                elapsed_ms = (time.perf_counter() - test_start) * 1000

                # Record successful result
                result = TestResult(
                    success=True,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time,
                    data_size=len(status.positions) if hasattr(status, 'positions') else 0
                )
                results.append(result)

                if test_count % 100 == 0:
                    logger.info(f"  Test {test_count}, latency: {elapsed_ms:.2f}ms")

            except Exception as e:
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                result = TestResult(
                    success=False,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time,
                    error_msg=str(e)
                )
                results.append(result)
                logger.warning(f"  Test failed: {e}")

            test_count += 1

            # Control minimum interval
            if self.config.min_interval_ms > 0:
                await asyncio.sleep(self.config.min_interval_ms / 1000)

        total_duration = time.perf_counter() - start_time

        # Generate report
        report = self._generate_report(function_name, results, total_duration, self.config.target_frequency)
        self.test_results[function_name] = results

        return report

    async def test_set_finger_positions_frequency(self) -> FrequencyTestReport:
        """Test the set frequency of set_finger_positions"""
        logger.info("📊 Starting test for set_finger_positions set frequency...")

        if not self.client or self.slave_id is None:
            raise RuntimeError("Device not initialized")

        function_name = "set_finger_positions"
        results = []
        start_time = time.perf_counter()
        test_count = 0

        # Define test position sequence: handshake -> open -> half-握 -> open
        position_sequences = [
            [400, 400, 1000, 1000, 1000, 1000],  # Handshake
            [400, 400, 0, 0, 0, 0],              # Open
            [400, 400, 500, 500, 500, 500],      # Half-handshake
            [400, 400, 0, 0, 0, 0],              # Open
        ]

        while (time.perf_counter() - start_time) < self.config.test_duration and test_count < self.config.max_test_count:
            test_start = time.perf_counter()

            try:
                # Loop using different position settings
                positions = position_sequences[test_count % len(position_sequences)]

                # Call set_finger_positions
                await self.client.set_finger_positions(self.slave_id, positions)

                elapsed_ms = (time.perf_counter() - test_start) * 1000

                # Record successful result
                result = TestResult(
                    success=True,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time,
                    data_size=len(positions)
                )
                results.append(result)

                if test_count % 50 == 0:
                    logger.info(f"  Test {test_count}, latency: {elapsed_ms:.2f}ms, positions: {positions}")

            except Exception as e:
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                result = TestResult(
                    success=False,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time,
                    error_msg=str(e)
                )
                results.append(result)
                logger.warning(f"  Test failed: {e}")

            test_count += 1

            # Control interval, position setting does not need to be too frequent
            await asyncio.sleep(max(self.config.min_interval_ms / 1000, 0.005))  # Minimum 5ms interval

        total_duration = time.perf_counter() - start_time

        # Generate report
        report = self._generate_report(function_name, results, total_duration, 50)  # Position setting target frequency较低
        self.test_results[function_name] = results
        return report

    async def test_mixed_frequency(self) -> FrequencyTestReport:
        """Test mixed function call frequency (simulate actual usage scenarios: read status + control position)"""
        logger.info("📊 Starting test for mixed function call frequency...")

        if not self.client or self.slave_id is None:
            raise RuntimeError("Device not initialized")

        function_name = "mixed_functions"
        results = []
        start_time = time.perf_counter()
        test_count = 0

        # Define position sequence: simulate actual grasping action
        position_sequences = [
            [400, 400, 0, 0, 0, 0],              # Open
            [400, 400, 300, 300, 300, 300],      # Preposition position
            [400, 400, 600, 600, 600, 600],      # Contact
            [400, 400, 1000, 1000, 1000, 1000],  # Grip
        ]

        while (time.perf_counter() - start_time) < self.config.test_duration and test_count < self.config.max_test_count:
            test_start = time.perf_counter()

            try:
                # Simulate actual usage: mainly read status,配合位置控制
                if test_count % 5 == 0:
                    # Call position setting every 5 times
                    positions = position_sequences[test_count // 5 % len(position_sequences)]
                    await self.client.set_finger_positions(self.slave_id, positions) # type: ignore
                else:
                    # Most of the time read status
                    await self.client.get_motor_status(self.slave_id) # type: ignore

                elapsed_ms = (time.perf_counter() - test_start) * 1000

                result = TestResult(
                    success=True,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time
                )
                results.append(result)

                if test_count % 100 == 0:
                    action_type = "position setting" if test_count % 5 == 0 else "status read"
                    logger.info(f"  Test {test_count}, latency: {elapsed_ms:.2f}ms, action: {action_type}")

            except Exception as e:
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                result = TestResult(
                    success=False,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time,
                    error_msg=str(e)
                )
                results.append(result)
                logger.warning(f"  Test failed: {e}")

            test_count += 1
            await asyncio.sleep(self.config.min_interval_ms / 1000)

        total_duration = time.perf_counter() - start_time

        # Generate report
        report = self._generate_report(function_name, results, total_duration, self.config.target_frequency)
        self.test_results[function_name] = results

        return report

    async def test_stability(self) -> Dict[str, Any]:
        """Long-term stability test"""
        logger.info(f"📊 Starting long-term stability test ({self.config.stability_duration} seconds)...")

        if not self.client or self.slave_id is None:
            raise RuntimeError("Device not initialized")

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

            # Test multiple times within the sampling interval
            while (time.perf_counter() - sample_start) < self.config.sample_interval:
                test_start = time.perf_counter()

                try:
                    await self.client.get_motor_status(self.slave_id)
                    elapsed_ms = (time.perf_counter() - test_start) * 1000
                    sample_results.append(TestResult(True, elapsed_ms, time.perf_counter() - start_time))
                except Exception as e:
                    elapsed_ms = (time.perf_counter() - test_start) * 1000
                    sample_results.append(TestResult(False, elapsed_ms, time.perf_counter() - start_time, error_msg=str(e)))

                await asyncio.sleep(0.01)  # 10ms interval

            # Calculate the statistical data of this sample
            successful = [r for r in sample_results if r.success]
            total_count = len(sample_results)
            success_count = len(successful)

            avg_latency = statistics.mean([r.elapsed_ms for r in successful]) if successful else 0
            error_rate = (total_count - success_count) / total_count * 100 if total_count > 0 else 0

            stability_results['samples'].append(total_count)
            stability_results['avg_latencies'].append(avg_latency)
            stability_results['error_rates'].append(error_rate)
            stability_results['timestamps'].append(time.perf_counter() - start_time)

            sample_count += 1
            if sample_count % 10 == 0:
                logger.info(f"  Stability test progress: {sample_count} samples, current latency: {avg_latency:.2f}ms, error rate: {error_rate:.1f}%")

        return stability_results

    def _generate_report(self, function_name: str, results: List[TestResult],
                        total_duration: float, target_frequency: float) -> FrequencyTestReport:
        """Generate test report"""
        successful_results = [r for r in results if r.success]
        failed_results = [r for r in results if not r.success]

        total_tests = len(results)
        successful_tests = len(successful_results)
        failed_tests = len(failed_results)
        success_rate = (successful_tests / total_tests * 100) if total_tests > 0 else 0

        if successful_results:
            latencies = [r.elapsed_ms for r in successful_results]
            avg_latency = statistics.mean(latencies)
            max_latency = max(latencies)
            min_latency = min(latencies)
            std_dev = statistics.stdev(latencies) if len(latencies) > 1 else 0
        else:
            avg_latency = max_latency = min_latency = std_dev = 0

        actual_frequency = total_tests / total_duration if total_duration > 0 else 0

        return FrequencyTestReport(
            function_name=function_name,
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
            total_duration_s=total_duration
        )

    def print_report(self, report: FrequencyTestReport):
        """Print test report"""
        print(f"\n{'='*60}")
        print(f"📊 {report.function_name} frequency test report")
        print(f"{'='*60}")
        print(f"Total tests:     {report.total_tests}")
        print(f"Successful tests: {report.successful_tests}")
        print(f"Failed tests:    {report.failed_tests}")
        print(f"Success rate:    {report.success_rate:.1f}%")
        print(f"")
        print(f"Average latency: {report.avg_latency_ms:.2f} milliseconds")
        print(f"Minimum latency: {report.min_latency_ms:.2f} milliseconds")
        print(f"Maximum latency: {report.max_latency_ms:.2f} milliseconds")
        print(f"Delay standard deviation: {report.std_dev_ms:.2f} milliseconds")
        print(f"")
        print(f"Actual frequency: {report.actual_frequency_hz:.1f} Hz")
        print(f"Target frequency: {report.target_frequency_hz:.1f} Hz")
        print(f"Frequency achievement rate: {(report.actual_frequency_hz/report.target_frequency_hz*100):.1f}%")
        print(f"Test duration: {report.total_duration_s:.1f} seconds")

    def plot_results(self, reports: List[FrequencyTestReport], stability_data: Optional[Dict[str, Any]] = None):
        """Plot test results"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Revo2 Communication Frequency Test Results', fontsize=16)

        # 1. Frequency comparison chart
        ax1 = axes[0, 0]
        function_names = [r.function_name for r in reports]
        actual_frequencies = [r.actual_frequency_hz for r in reports]
        target_frequencies = [r.target_frequency_hz for r in reports]

        x = range(len(function_names))
        width = 0.35

        ax1.bar([i - width/2 for i in x], actual_frequencies, width, label='Actual Frequency', alpha=0.8)
        ax1.bar([i + width/2 for i in x], target_frequencies, width, label='Target Frequency', alpha=0.8)
        ax1.set_xlabel('Test Function')
        ax1.set_ylabel('Frequency (Hz)')
        ax1.set_title('Frequency Comparison')
        ax1.set_xticks(x)
        ax1.set_xticklabels(function_names, rotation=45)
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # 2. Delay distribution chart
        ax2 = axes[0, 1]
        avg_latencies = [r.avg_latency_ms for r in reports]
        std_devs = [r.std_dev_ms for r in reports]

        ax2.bar(function_names, avg_latencies, yerr=std_devs, capsize=5, alpha=0.8)
        ax2.set_xlabel('Test Function')
        ax2.set_ylabel('Latency (ms)')
        ax2.set_title('Average Latency with Standard Deviation')
        ax2.tick_params(axis='x', rotation=45)
        ax2.grid(True, alpha=0.3)

        # 3. Success rate chart
        ax3 = axes[1, 0]
        success_rates = [r.success_rate for r in reports]
        colors = ['green' if rate >= 95 else 'orange' if rate >= 90 else 'red' for rate in success_rates]

        bars = ax3.bar(function_names, success_rates, color=colors, alpha=0.8)
        ax3.set_xlabel('Test Function')
        ax3.set_ylabel('Success Rate (%)')
        ax3.set_title('Communication Success Rate')
        ax3.set_ylim(0, 105)
        ax3.tick_params(axis='x', rotation=45)
        ax3.grid(True, alpha=0.3)

        # Add value labels to the bar chart
        for bar, rate in zip(bars, success_rates):
            height = bar.get_height()
            ax3.text(bar.get_x() + bar.get_width()/2., height + 1,
                    f'{rate:.1f}%', ha='center', va='bottom')

        # 4. Stability test results
        ax4 = axes[1, 1]
        if stability_data:
            timestamps = stability_data['timestamps']
            avg_latencies = stability_data['avg_latencies']
            error_rates = stability_data['error_rates']

            # Display delay and error rate on two Y axes
            ax4_twin = ax4.twinx()

            line1 = ax4.plot(timestamps, avg_latencies, 'b-', label='Average Latency')
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Latency (ms)', color='b')
            ax4.tick_params(axis='y', labelcolor='b')

            line2 = ax4_twin.plot(timestamps, error_rates, 'r-', label='Error Rate')
            ax4_twin.set_ylabel('Error Rate (%)', color='r')
            ax4_twin.tick_params(axis='y', labelcolor='r')

            # Merge legend
            lines = line1 + line2
            labels = [l.get_label() for l in lines]
            ax4.legend(lines, labels, loc='upper left')
            ax4.set_title('Long-term Stability Test')
            ax4.grid(True, alpha=0.3)
        else:
            ax4.text(0.5, 0.5, 'No Stability Test Data', ha='center', va='center', transform=ax4.transAxes)
            ax4.set_title('Stability Test')

        plt.tight_layout()
        plt.savefig('revo2_comm_frequency_test_results.png', dpi=300, bbox_inches='tight')
        plt.show()

    def save_detailed_results(self, reports: List[FrequencyTestReport], stability_data: Optional[Dict[str, Any]] = None):
        """Save detailed test results to file"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")

        # Save test report
        report_data = {
            'timestamp': timestamp,
            'test_config': {
                'test_duration': self.config.test_duration,
                'target_frequency': self.config.target_frequency,
                'max_test_count': self.config.max_test_count,
                'min_interval_ms': self.config.min_interval_ms
            },
            'reports': []
        }

        for report in reports:
            report_data['reports'].append({
                'function_name': report.function_name,
                'total_tests': report.total_tests,
                'successful_tests': report.successful_tests,
                'failed_tests': report.failed_tests,
                'success_rate': report.success_rate,
                'avg_latency_ms': report.avg_latency_ms,
                'max_latency_ms': report.max_latency_ms,
                'min_latency_ms': report.min_latency_ms,
                'std_dev_ms': report.std_dev_ms,
                'actual_frequency_hz': report.actual_frequency_hz,
                'target_frequency_hz': report.target_frequency_hz,
                'total_duration_s': report.total_duration_s
            })

        if stability_data:
            report_data['stability_test'] = stability_data

        # Save as JSON file
        with open(f'revo2_comm_test_report_{timestamp}.json', 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)

        logger.info(f"📁 Detailed test results saved to: revo2_comm_test_report_{timestamp}.json")

async def main():
    """Main test function"""
    print("🤖 Revo2 dexterous hand communication frequency test")
    print("="*60)

    # Create test configuration
    config = TestConfig(
        test_duration=10.0,          # Each test duration 10 seconds
        target_frequency=500,        # Target frequency 500Hz
        min_interval_ms=1.0,         # Minimum 1ms interval
        stability_duration=10.0      # Stability test duration 10 seconds
    )

    # Create tester
    tester = Revo2CommTester(config)

    try:
        # Initialize connection
        if not await tester.initialize():
            return

        # Run各项测试
        reports = []

        # 1. Test get_motor_status frequency
        report1 = await tester.test_get_motor_status_frequency()
        reports.append(report1)
        tester.print_report(report1)

        # 2. Test set_finger_positions frequency
        report2 = await tester.test_set_finger_positions_frequency()
        reports.append(report2)
        tester.print_report(report2)

        # 3. Test mixed function frequency
        report3 = await tester.test_mixed_frequency()
        reports.append(report3)
        tester.print_report(report3)

        # 4. Long-term stability test
        print(f"\n{'='*60}")
        print("🔄 Starting long-term stability test...")
        stability_data = await tester.test_stability()

        # Print stability test results
        print(f"\n{'='*60}")
        print("📊 Stability test results")
        print(f"{'='*60}")
        if stability_data['avg_latencies']:
            overall_avg_latency = statistics.mean(stability_data['avg_latencies'])
            overall_error_rate = statistics.mean(stability_data['error_rates'])
            max_error_rate = max(stability_data['error_rates'])
            print(f"Average latency:       {overall_avg_latency:.2f} milliseconds")
            print(f"Average error rate:     {overall_error_rate:.2f}%")
            print(f"Maximum error rate:     {max_error_rate:.2f}%")
            print(f"Sample count:       {len(stability_data['samples'])}")

        # Generate charts and save results
        tester.plot_results(reports, stability_data)
        tester.save_detailed_results(reports, stability_data)

        print(f"\n{'='*60}")
        print("✅ All tests completed!")
        print("📊 Results charts saved to: revo2_comm_frequency_test_results.png")
        print(f"{'='*60}")

    except Exception as e:
        logger.error(f"❌ Error occurred during test: {e}")
    finally:
        await tester.cleanup()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
