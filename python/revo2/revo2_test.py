"""
Revo2灵巧手性能测试示例

本示例演示如何对Revo2灵巧手进行性能测试和分析，包括：
- 高频控制测试（200Hz控制频率）
- 位置跟踪精度测试
- 电机响应性能分析
- 实时数据采集和记录
- 可视化分析和结果保存

测试特点：
- 使用余弦轨迹作为目标位置
- 控制频率：200Hz（5ms间隔）
- 数据采集频率：尽可能高频
- 测试时长：6秒
- 支持数据可视化和导出

应用场景：
- 电机响应性能评估
- 控制算法优化
- 系统稳定性测试
- 精度分析和校准
"""

import asyncio
import sys
import math
import time
from revo2_utils import libstark, logger, open_modbus_revo2
import matplotlib.pyplot as plt

# 测试参数配置
TRAJ_LEN = 200          # 轨迹点数（余弦波的采样点数）
CTRL_INTERVAL = 0.005   # 控制间隔 5ms，对应200Hz控制频率
TEST_DURATION = 6.0     # 测试时长 6秒
DATA_START_TIME = 0.8   # 数据记录开始时间，跳过初始稳定阶段


def init_trajectory():
    """
    初始化余弦轨迹

    生成一个完整周期的余弦波轨迹，用作目标位置序列。
    轨迹范围：0-100%，精度：0.1%

    Returns:
        list: 轨迹点列表，每个点的值范围[0, 1000]（0.1%精度）
    """
    traj = []
    for i in range(TRAJ_LEN):
        # 生成余弦波：范围[0, 100]，偏移+50，幅值50
        value = 50.0 + 50.0 * math.cos(2 * math.pi * i / TRAJ_LEN)
        traj.append(int(value * 10))  # 转换为0.1%精度 [0,1000]
    return traj


# 初始化图表
plt.figure(figsize=(12, 6))
plt.xlabel('Time (s)')
plt.ylabel('Position (%)')
plt.title('Motor Response Performance')
plt.grid(True)


async def main():
    """
    主函数：执行性能测试
    """
    # 连接Revo2设备
    (client, slave_id) = await open_modbus_revo2()

    # 配置控制模式
    await configure_control_mode(client, slave_id)

    # 初始化测试轨迹
    trajectory = init_trajectory()
    traj_len = len(trajectory)

    # 数据记录变量
    target_positions = []
    measured_positions = []
    timestamps = []

    # 控制变量
    index = 0
    start_time = time.perf_counter()
    stop_flag = False
    finger_id = libstark.FingerId.Ring  # 选择无名指进行测试

    # 启动并发任务
    await run_concurrent_tasks(
        client, slave_id, finger_id, trajectory, traj_len,
        target_positions, measured_positions, timestamps,
        start_time, stop_flag, index
    )

    # 分析和保存结果
    await analyze_and_save_results(target_positions, measured_positions, timestamps)

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def configure_control_mode(client, slave_id):
    """
    配置控制模式

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 设置手指控制参数的单位模式
    logger.debug("set_finger_unit_mode")
    await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized)  # 千分比模式
    # await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Physical)  # 物理量模式

    # 获取并验证手指控制模式
    logger.debug("get_finger_unit_mode")
    finger_unit_mode = await client.get_finger_unit_mode(slave_id)
    logger.info(f"Finger unit mode: {finger_unit_mode}")

    # 参考文档：https://www.brainco-hz.com/docs/revolimb-hand/revo2/modbus_foundation.html#%E5%8D%95%E4%BD%8D%E6%A8%A1%E5%BC%8F%E8%AE%BE%E7%BD%AE-937


async def run_concurrent_tasks(client, slave_id, finger_id, trajectory, traj_len,
                              target_positions, measured_positions, timestamps,
                              start_time, stop_flag, index):
    """
    运行并发测试任务

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
        finger_id: 测试手指ID
        trajectory: 轨迹数据
        traj_len: 轨迹长度
        target_positions: 目标位置记录列表
        measured_positions: 实际位置记录列表
        timestamps: 时间戳记录列表
        start_time: 测试开始时间
        stop_flag: 停止标志
        index: 轨迹索引
    """

    async def control_task():
        """
        控制任务：200Hz频率发送目标位置
        """
        nonlocal index, stop_flag
        while not stop_flag:
            task_start = time.perf_counter()

            # 循环遍历轨迹点
            index = (index + 1) % traj_len
            target = trajectory[index]

            # 发送位置控制指令，1ms表示以最快速度响应
            await client.set_finger_position_with_millis(slave_id, finger_id, target, 1)

            elapsed = (time.perf_counter() - task_start) * 1000
            logger.info(f"[{index}] Target: {target}, Control elapsed: {elapsed:.1f}ms")

            # 保持控制频率
            if elapsed < CTRL_INTERVAL:
                await asyncio.sleep(CTRL_INTERVAL - elapsed)

    async def read_task():
        """
        读取任务：尽可能高频读取当前位置
        """
        nonlocal stop_flag
        while not stop_flag:
            try:
                # 获取电机状态
                status: libstark.MotorStatusData = await client.get_motor_status(slave_id)
                pos = status.positions[4]  # 读取无名指位置（索引4）

                current_time = time.perf_counter() - start_time
                target = trajectory[index]
                measured = pos

                # 记录数据（跳过初始稳定阶段）
                if current_time > DATA_START_TIME:
                    target_positions.append(target)
                    measured_positions.append(measured)
                    timestamps.append(current_time)

                # logger.debug(f"Time: {current_time:.3f}s, Target: {target}, Measured: {measured}")

            except Exception as e:
                logger.error(f"Read error: {e}")

            await asyncio.sleep(0.001)  # 尽可能快，但防止过载

    # 启动并发任务
    control = asyncio.create_task(control_task())
    reader = asyncio.create_task(read_task())

    # 等待测试完成
    await asyncio.sleep(TEST_DURATION)
    stop_flag = True

    # 等待任务结束
    await control
    await reader


async def analyze_and_save_results(target_positions, measured_positions, timestamps):
    """
    分析和保存测试结果

    Args:
        target_positions: 目标位置列表
        measured_positions: 实际位置列表
        timestamps: 时间戳列表
    """
    # 绘制性能分析图表
    plt.plot(timestamps, target_positions, label='Target Position (0~1000)', linestyle='--')
    plt.plot(timestamps, measured_positions, label='Measured Position (0~1000)', alpha=0.7)
    plt.legend()
    plt.show()
    plt.savefig("motor_response.png")
    logger.info(f"Test completed, data points collected: {len(timestamps)}")

    # 保存数据到文件
    with open("motor_response_data.txt", "w") as f:
        f.write("Time (s), Target Position, Measured Position\n")
        for t, target, measured in zip(timestamps, target_positions, measured_positions):
            f.write(f"{t:.3f}, {target}, {measured}\n")

    logger.info("Test data saved to motor_response_data.txt")
    logger.info("Performance chart saved to motor_response.png")


if __name__ == "__main__":
    asyncio.run(main())
