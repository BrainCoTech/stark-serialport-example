"""
Revo1通讯频率测试 - 快速演示示例

这是一个简化版本的测试示例，用于快速验证Revo1的通讯性能。
"""

import asyncio
import time
import statistics
from revo1_utils import *

async def quick_frequency_test(duration=5.0, target_hz=50, test_type="read"):
    """
    快速频率测试

    Args:
        duration: 测试持续时间（秒）
        target_hz: 目标频率（Hz）
        test_type: 测试类型 "read"(状态读取) 或 "write"(位置设置) 或 "mixed"(混合)
    """
    test_name_map = {
        "read": "get_motor_status 状态读取",
        "write": "set_finger_positions 位置设置",
        "mixed": "混合功能（读取+设置）"
    }

    print(f"🚀 开始 {duration} 秒的 {test_name_map.get(test_type, test_type)} 频率测试")
    print(f"🎯 目标频率: {target_hz} Hz")

    # 连接设备
    try:
        client, slave_id = await open_modbus_revo1()
        logger.info(f"✅ 成功连接设备，从站ID: {slave_id}")
    except Exception as e:
        logger.error(f"❌ 连接失败: {e}")
        return

    # 测试数据记录
    results = []
    start_time = time.perf_counter()
    test_count = 0
    successful_count = 0

    # 位置序列（用于写测试）
    position_sequences = [
        [0, 0, 0, 0, 0, 0],            # 张开
        [30, 30, 50, 50, 50, 50],      # 半握
        [60, 60, 100, 100, 100, 100],  # 握紧
    ]

    try:
        while (time.perf_counter() - start_time) < duration:
            test_start = time.perf_counter()

            try:
                positions = None  # 初始化变量
                if test_type == "read":
                    # 状态读取测试
                    status = await client.get_motor_status(slave_id)
                elif test_type == "write":
                    # 位置设置测试
                    positions = position_sequences[test_count % len(position_sequences)]
                    await client.set_finger_positions(slave_id, positions)
                elif test_type == "mixed":
                    # 混合测试：主要读取，偶尔设置
                    if test_count % 5 == 0:
                        positions = position_sequences[test_count // 5 % len(position_sequences)]
                        await client.set_finger_positions(slave_id, positions)
                    else:
                        status = await client.get_motor_status(slave_id)
                else:
                    # 默认读取测试
                    status = await client.get_motor_status(slave_id)

                elapsed_ms = (time.perf_counter() - test_start) * 1000
                results.append(elapsed_ms)
                successful_count += 1

                if test_count % 50 == 0:
                    action_desc = ""
                    if test_type == "write" and positions:
                        action_desc = f" - 位置: {positions}"
                    elif test_type == "mixed":
                        action_desc = f" - {'位置设置' if test_count % 5 == 0 else '状态读取'}"
                    print(f"  第 {test_count:3d} 次测试 - 延迟: {elapsed_ms:6.2f}ms{action_desc}")

            except Exception as e:
                print(f"  测试失败: {e}")

            test_count += 1

            # 控制频率
            await asyncio.sleep(max(0, 1.0/target_hz - (time.perf_counter() - test_start)))

        total_duration = time.perf_counter() - start_time

        # 计算统计数据
        if results:
            avg_latency = statistics.mean(results)
            max_latency = max(results)
            min_latency = min(results)
            std_dev = statistics.stdev(results) if len(results) > 1 else 0
            actual_frequency = test_count / total_duration
            success_rate = successful_count / test_count * 100

            print(f"\n{'='*50}")
            print(f"📊 测试结果统计")
            print(f"{'='*50}")
            print(f"总测试次数:     {test_count}")
            print(f"成功次数:       {successful_count}")
            print(f"成功率:         {success_rate:.1f}%")
            print(f"平均延迟:       {avg_latency:.2f} ms")
            print(f"最小延迟:       {min_latency:.2f} ms")
            print(f"最大延迟:       {max_latency:.2f} ms")
            print(f"延迟标准差:     {std_dev:.2f} ms")
            print(f"实际频率:       {actual_frequency:.1f} Hz")
            print(f"目标频率:       {target_hz} Hz")
            print(f"频率达成率:     {(actual_frequency/target_hz*100):.1f}%")
            print(f"测试时长:       {total_duration:.1f} 秒")

            # 简单性能评估
            print(f"\n🔍 性能评估:")
            if avg_latency < 10:
                print("✅ 通讯延迟优秀 (< 10ms)")
            elif avg_latency < 20:
                print("🟡 通讯延迟良好 (< 20ms)")
            else:
                print("🔴 通讯延迟较高 (>= 20ms)")

            if success_rate >= 99:
                print("✅ 通讯稳定性优秀 (>= 99%)")
            elif success_rate >= 95:
                print("🟡 通讯稳定性良好 (>= 95%)")
            else:
                print("🔴 通讯稳定性需要改善 (< 95%)")

            if actual_frequency >= target_hz * 0.9:
                print("✅ 频率达成率优秀 (>= 90%)")
            elif actual_frequency >= target_hz * 0.8:
                print("🟡 频率达成率良好 (>= 80%)")
            else:
                print("🔴 频率达成率较低 (< 80%)")

        else:
            print("❌ 没有成功的测试结果")

    except KeyboardInterrupt:
        print("\n⚠️  测试被用户中断")

    finally:
        libstark.modbus_close(client)
        logger.info("🔌 连接已关闭")

async def main():
    """主函数：快速测试演示"""
    print("🤖 Revo1 通讯频率快速测试")
    print("="*50)

    # 运行不同类型和频率的测试
    test_configs = [
        # (时长, 频率, 类型, 描述)
        (5.0, 50, "read", "状态读取测试"),
        (5.0, 20, "write", "位置设置测试"),
        (5.0, 40, "mixed", "混合功能测试"),
    ]

    for duration, frequency, test_type, description in test_configs:
        print(f"\n📋 {description}")
        await quick_frequency_test(duration, frequency, test_type)
        print("\n" + "-"*50)
        await asyncio.sleep(1)  # 测试间隔

    print("\n🎉 所有测试完成！")
    print("\n💡 测试总结：")
    print("   • 状态读取 (get_motor_status): 用于实时监控手指位置和状态")
    print("   • 位置设置 (set_finger_positions): 用于控制手指运动到目标位置")
    print("   • 混合功能: 模拟实际应用中的读取+控制操作")

if __name__ == "__main__":
    asyncio.run(main())
