"""
Revo1灵巧手通讯频率测试

本测试专门用于评估Revo1灵巧手的实际使用场景下的通讯频率和性能，包括：
- get_motor_status 读取频率测试（状态监控）
- set_finger_positions 设置频率测试（位置控制）
- 不同通讯功能的延迟分析
- 高频连续通讯的稳定性测试
- 通讯错误率统计

测试功能：
1. 单一功能频率测试：连续调用同一API测量最大频率
2. 混合功能测试：模拟实际使用场景的读取+控制调用
3. 长时间稳定性测试：评估长期运行的稳定性
4. 数据完整性验证：确保高频通讯下数据的准确性

应用场景：
- 实时控制系统性能评估
- 手指位置控制频率测试
- 状态监控与控制的协调性测试
- 系统集成测试
"""

import asyncio
import time
import statistics
import sys
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from revo1_utils import *
import matplotlib.pyplot as plt
import json

# 测试配置参数
@dataclass
class TestConfig:
    """测试配置类"""
    # 基础测试参数
    test_duration: float = 10.0      # 单个测试持续时间（秒）
    max_test_count: int = 1000       # 最大测试次数
    timeout_ms: float = 100.0        # 超时阈值（毫秒）

    # 频率测试参数
    target_frequency: int = 100      # 目标频率（Hz）
    min_interval_ms: float = 1.0     # 最小间隔（毫秒）

    # 稳定性测试参数
    stability_duration: float = 60.0 # 稳定性测试时长（秒）
    sample_interval: float = 1.0     # 采样间隔（秒）

@dataclass
class TestResult:
    """单次测试结果"""
    success: bool
    elapsed_ms: float
    timestamp: float
    data_size: int = 0
    error_msg: str = ""

@dataclass
class FrequencyTestReport:
    """频率测试报告"""
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

class Revo1CommTester:
    """Revo1通讯频率测试器"""

    def __init__(self, config: TestConfig):
        self.config = config
        self.client: Optional[libstark.PyDeviceContext] = None
        self.slave_id: Optional[int] = None
        self.test_results: Dict[str, List[TestResult]] = {}

    async def initialize(self):
        """初始化连接"""
        try:
            logger.info("🔌 正在连接Revo1设备...")
            self.client, self.slave_id = await open_modbus_revo1()
            logger.info(f"✅ 成功连接到设备，从站ID: {self.slave_id}")
            return True
        except Exception as e:
            logger.error(f"❌ 连接失败: {e}")
            return False

    async def cleanup(self):
        """清理资源"""
        if self.client:
            libstark.modbus_close(self.client)
            logger.info("🔌 连接已关闭")

    async def test_get_motor_status_frequency(self) -> FrequencyTestReport:
        """测试 get_motor_status 的读取频率"""
        logger.info("📊 开始测试 get_motor_status 读取频率...")

        if not self.client or self.slave_id is None:
            raise RuntimeError("设备未初始化")

        function_name = "get_motor_status"
        results = []
        start_time = time.perf_counter()
        test_count = 0

        while (time.perf_counter() - start_time) < self.config.test_duration and test_count < self.config.max_test_count:
            test_start = time.perf_counter()

            try:
                # 调用 get_motor_status
                status: libstark.MotorStatusData = await self.client.get_motor_status(self.slave_id)

                elapsed_ms = (time.perf_counter() - test_start) * 1000

                # 记录成功结果
                result = TestResult(
                    success=True,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time,
                    data_size=len(status.positions) if hasattr(status, 'positions') else 0
                )
                results.append(result)

                if test_count % 100 == 0:
                    logger.info(f"  第 {test_count} 次测试，延迟: {elapsed_ms:.2f}ms")

            except Exception as e:
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                result = TestResult(
                    success=False,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time,
                    error_msg=str(e)
                )
                results.append(result)
                logger.warning(f"  测试失败: {e}")

            test_count += 1

            # 控制最小间隔
            if self.config.min_interval_ms > 0:
                await asyncio.sleep(self.config.min_interval_ms / 1000)

        total_duration = time.perf_counter() - start_time

        # 生成报告
        report = self._generate_report(function_name, results, total_duration, self.config.target_frequency)
        self.test_results[function_name] = results

        return report

    async def test_set_finger_positions_frequency(self) -> FrequencyTestReport:
        """测试 set_finger_positions 的设置频率"""
        logger.info("📊 开始测试 set_finger_positions 设置频率...")

        if not self.client or self.slave_id is None:
            raise RuntimeError("设备未初始化")

        function_name = "set_finger_positions"
        results = []
        start_time = time.perf_counter()
        test_count = 0

        # 定义测试位置序列：握手 -> 张开 -> 半握 -> 张开
        position_sequences = [
            [60, 60, 100, 100, 100, 100],  # 握手
            [0, 0, 0, 0, 0, 0],            # 张开
            [30, 30, 50, 50, 50, 50],      # 半握
            [0, 0, 0, 0, 0, 0],            # 张开
        ]

        while (time.perf_counter() - start_time) < self.config.test_duration and test_count < self.config.max_test_count:
            test_start = time.perf_counter()

            try:
                # 循环使用不同的位置设置
                positions = position_sequences[test_count % len(position_sequences)]

                # 调用 set_finger_positions
                await self.client.set_finger_positions(self.slave_id, positions)

                elapsed_ms = (time.perf_counter() - test_start) * 1000

                # 记录成功结果
                result = TestResult(
                    success=True,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time,
                    data_size=len(positions)
                )
                results.append(result)

                if test_count % 50 == 0:
                    logger.info(f"  第 {test_count} 次测试，延迟: {elapsed_ms:.2f}ms，位置: {positions}")

            except Exception as e:
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                result = TestResult(
                    success=False,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time,
                    error_msg=str(e)
                )
                results.append(result)
                logger.warning(f"  测试失败: {e}")

            test_count += 1

            # 控制间隔，位置设置不需要太频繁
            await asyncio.sleep(max(self.config.min_interval_ms / 1000, 0.005))  # 最少5ms间隔

        total_duration = time.perf_counter() - start_time

        # 生成报告
        report = self._generate_report(function_name, results, total_duration, 50)  # 位置设置目标频率较低
        self.test_results[function_name] = results

        return report

    async def test_mixed_frequency(self) -> FrequencyTestReport:
        """测试混合功能调用频率（模拟实际使用场景：读取状态+控制位置）"""
        logger.info("📊 开始测试混合功能调用频率...")

        if not self.client or self.slave_id is None:
            raise RuntimeError("设备未初始化")

        function_name = "mixed_functions"
        results = []
        start_time = time.perf_counter()
        test_count = 0

        # 位置序列：模拟实际抓取动作
        position_sequences = [
            [0, 0, 0, 0, 0, 0],            # 张开
            [20, 20, 30, 30, 30, 30],      # 预备位置
            [40, 40, 60, 60, 60, 60],      # 接触
            [60, 60, 100, 100, 100, 100],  # 握紧
        ]

        while (time.perf_counter() - start_time) < self.config.test_duration and test_count < self.config.max_test_count:
            test_start = time.perf_counter()

            try:
                # 模拟实际使用：主要是状态读取，配合位置控制
                if test_count % 5 == 0:
                    # 每5次调用一次位置设置
                    positions = position_sequences[test_count // 5 % len(position_sequences)]
                    await self.client.set_finger_positions(self.slave_id, positions)
                else:
                    # 大部分时间读取状态
                    await self.client.get_motor_status(self.slave_id)

                elapsed_ms = (time.perf_counter() - test_start) * 1000

                result = TestResult(
                    success=True,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time
                )
                results.append(result)

                if test_count % 100 == 0:
                    action_type = "位置设置" if test_count % 5 == 0 else "状态读取"
                    logger.info(f"  第 {test_count} 次混合测试，延迟: {elapsed_ms:.2f}ms，动作: {action_type}")

            except Exception as e:
                elapsed_ms = (time.perf_counter() - test_start) * 1000
                result = TestResult(
                    success=False,
                    elapsed_ms=elapsed_ms,
                    timestamp=time.perf_counter() - start_time,
                    error_msg=str(e)
                )
                results.append(result)
                logger.warning(f"  混合测试失败: {e}")

            test_count += 1
            await asyncio.sleep(self.config.min_interval_ms / 1000)

        total_duration = time.perf_counter() - start_time

        # 生成报告
        report = self._generate_report(function_name, results, total_duration, self.config.target_frequency)
        self.test_results[function_name] = results

        return report

    async def test_stability(self) -> Dict[str, Any]:
        """长时间稳定性测试"""
        logger.info(f"📊 开始长时间稳定性测试 ({self.config.stability_duration}秒)...")

        if not self.client or self.slave_id is None:
            raise RuntimeError("设备未初始化")

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

            # 在采样间隔内进行多次测试
            while (time.perf_counter() - sample_start) < self.config.sample_interval:
                test_start = time.perf_counter()

                try:
                    await self.client.get_motor_status(self.slave_id)
                    elapsed_ms = (time.perf_counter() - test_start) * 1000
                    sample_results.append(TestResult(True, elapsed_ms, time.perf_counter() - start_time))
                except Exception as e:
                    elapsed_ms = (time.perf_counter() - test_start) * 1000
                    sample_results.append(TestResult(False, elapsed_ms, time.perf_counter() - start_time, error_msg=str(e)))

                await asyncio.sleep(0.01)  # 10ms间隔

            # 计算本次采样的统计数据
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
                logger.info(f"  稳定性测试进度: {sample_count} 个采样点，当前延迟: {avg_latency:.2f}ms，错误率: {error_rate:.1f}%")

        return stability_results

    def _generate_report(self, function_name: str, results: List[TestResult],
                        total_duration: float, target_frequency: float) -> FrequencyTestReport:
        """生成测试报告"""
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
        """打印测试报告"""
        print(f"\n{'='*60}")
        print(f"📊 {report.function_name} 频率测试报告")
        print(f"{'='*60}")
        print(f"总测试次数:     {report.total_tests}")
        print(f"成功次数:       {report.successful_tests}")
        print(f"失败次数:       {report.failed_tests}")
        print(f"成功率:         {report.success_rate:.1f}%")
        print(f"")
        print(f"平均延迟:       {report.avg_latency_ms:.2f} 毫秒")
        print(f"最小延迟:       {report.min_latency_ms:.2f} 毫秒")
        print(f"最大延迟:       {report.max_latency_ms:.2f} 毫秒")
        print(f"延迟标准差:     {report.std_dev_ms:.2f} 毫秒")
        print(f"")
        print(f"实际频率:       {report.actual_frequency_hz:.1f} Hz")
        print(f"目标频率:       {report.target_frequency_hz:.1f} Hz")
        print(f"频率达成率:     {(report.actual_frequency_hz/report.target_frequency_hz*100):.1f}%")
        print(f"测试时长:       {report.total_duration_s:.1f} 秒")

    def plot_results(self, reports: List[FrequencyTestReport], stability_data: Optional[Dict[str, Any]] = None):
        """绘制测试结果图表"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Revo1 Communication Frequency Test Results', fontsize=16)

        # 1. 频率对比图
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

        # 2. 延迟分布图
        ax2 = axes[0, 1]
        avg_latencies = [r.avg_latency_ms for r in reports]
        std_devs = [r.std_dev_ms for r in reports]

        ax2.bar(function_names, avg_latencies, yerr=std_devs, capsize=5, alpha=0.8)
        ax2.set_xlabel('Test Function')
        ax2.set_ylabel('Latency (ms)')
        ax2.set_title('Average Latency with Standard Deviation')
        ax2.tick_params(axis='x', rotation=45)
        ax2.grid(True, alpha=0.3)

        # 3. 成功率图
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

        # 在柱状图上添加数值标签
        for bar, rate in zip(bars, success_rates):
            height = bar.get_height()
            ax3.text(bar.get_x() + bar.get_width()/2., height + 1,
                    f'{rate:.1f}%', ha='center', va='bottom')

        # 4. 稳定性测试结果
        ax4 = axes[1, 1]
        if stability_data:
            timestamps = stability_data['timestamps']
            avg_latencies = stability_data['avg_latencies']
            error_rates = stability_data['error_rates']

            # 双Y轴显示延迟和错误率
            ax4_twin = ax4.twinx()

            line1 = ax4.plot(timestamps, avg_latencies, 'b-', label='Average Latency')
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Latency (ms)', color='b')
            ax4.tick_params(axis='y', labelcolor='b')

            line2 = ax4_twin.plot(timestamps, error_rates, 'r-', label='Error Rate')
            ax4_twin.set_ylabel('Error Rate (%)', color='r')
            ax4_twin.tick_params(axis='y', labelcolor='r')

            # 合并图例
            lines = line1 + line2
            labels = [l.get_label() for l in lines]
            ax4.legend(lines, labels, loc='upper left')
            ax4.set_title('Long-term Stability Test')
            ax4.grid(True, alpha=0.3)
        else:
            ax4.text(0.5, 0.5, 'No Stability Test Data', ha='center', va='center', transform=ax4.transAxes)
            ax4.set_title('Stability Test')

        plt.tight_layout()
        plt.savefig('revo1_comm_frequency_test_results.png', dpi=300, bbox_inches='tight')
        plt.show()

    def save_detailed_results(self, reports: List[FrequencyTestReport], stability_data: Optional[Dict[str, Any]] = None):
        """保存详细测试结果到文件"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")

        # 保存测试报告
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

        # 保存为JSON文件
        with open(f'revo1_comm_test_report_{timestamp}.json', 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)

        logger.info(f"📁 详细测试结果已保存到: revo1_comm_test_report_{timestamp}.json")

async def main():
    """主测试函数"""
    print("🤖 Revo1灵巧手通讯频率测试")
    print("="*60)

    # 创建测试配置
    config = TestConfig(
        test_duration=15.0,          # 每个测试15秒
        target_frequency=100,        # 目标100Hz
        min_interval_ms=1.0,         # 最小1ms间隔
        stability_duration=30.0      # 稳定性测试30秒
    )

    # 创建测试器
    tester = Revo1CommTester(config)

    try:
        # 初始化连接
        if not await tester.initialize():
            return

        # 运行各项测试
        reports = []

        # 1. 测试 get_motor_status 频率
        report1 = await tester.test_get_motor_status_frequency()
        reports.append(report1)
        tester.print_report(report1)

        # 2. 测试 set_finger_positions 频率
        report2 = await tester.test_set_finger_positions_frequency()
        reports.append(report2)
        tester.print_report(report2)

        # 3. 测试混合功能频率
        report3 = await tester.test_mixed_frequency()
        reports.append(report3)
        tester.print_report(report3)

        # 4. 长时间稳定性测试
        print(f"\n{'='*60}")
        print("🔄 开始长时间稳定性测试...")
        stability_data = await tester.test_stability()

        # 打印稳定性测试结果
        print(f"\n{'='*60}")
        print("📊 稳定性测试结果")
        print(f"{'='*60}")
        if stability_data['avg_latencies']:
            overall_avg_latency = statistics.mean(stability_data['avg_latencies'])
            overall_error_rate = statistics.mean(stability_data['error_rates'])
            max_error_rate = max(stability_data['error_rates'])
            print(f"平均延迟:       {overall_avg_latency:.2f} 毫秒")
            print(f"平均错误率:     {overall_error_rate:.2f}%")
            print(f"最大错误率:     {max_error_rate:.2f}%")
            print(f"采样点数:       {len(stability_data['samples'])}")

        # 生成图表和保存结果
        tester.plot_results(reports, stability_data)
        tester.save_detailed_results(reports, stability_data)

        print(f"\n{'='*60}")
        print("✅ 所有测试完成！")
        print("📊 结果图表已保存为: revo1_comm_frequency_test_results.png")
        print(f"{'='*60}")

    except KeyboardInterrupt:
        print("\n⚠️  测试被用户中断")
    except Exception as e:
        logger.error(f"❌ 测试过程中发生错误: {e}")
    finally:
        await tester.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
