"""
Revo1灵巧手动作序列控制示例

本示例演示如何使用Revo1灵巧手的动作序列功能，包括：
- 自定义动作序列的创建和配置
- 动作序列的上传和存储
- 内置动作序列的执行
- 自定义动作序列的执行和管理
- 动作序列数据的读取和验证
"""

import asyncio
import sys
from utils import setup_shutdown_event
from revo1_utils import *

# 一代灵巧手动作序列配置
# 注意：当前版本中只有positions参数实际生效，speeds和forces参数为预留字段
sample_action_sequences = [
    {
        "index": 0,                          # 动作序列索引
        "duration_ms": 2000,                 # 动作持续时间（毫秒）
        "positions": [0, 0, 100, 100, 100, 100],  # 手指位置：[拇指, 拇指辅助, 食指, 中指, 无名指, 小指]
        "speeds": [10, 20, 30, 40, 50, 60],      # 手指速度（预留参数）
        "forces": [5, 10, 15, 20, 25, 30],       # 手指力度（预留参数）
    },
    {
        "index": 1,
        "duration_ms": 2000,
        "positions": [100, 100, 0, 0, 0, 0],     # 拇指闭合，其他手指张开
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 2,
        "duration_ms": 2000,
        "positions": [0, 0, 100, 100, 100, 100], # 拇指张开，其他手指闭合
        "speeds": [10, 20, 30, 40, 50, 60],
        "forces": [5, 10, 15, 20, 25, 30],
    },
    {
        "index": 3,
        "duration_ms": 2000,
        "positions": [100, 100, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 4,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],         # 所有手指完全张开
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 5,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],         # 保持张开状态
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 6,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 7,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 8,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 9,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
]


def action_sequence_info_to_list(action):
    """
    将动作序列字典转换为列表格式

    将动作序列的配置信息转换为SDK所需的列表格式，用于上传到设备。

    Args:
        action (dict): 动作序列配置字典，包含index、duration_ms、positions、speeds、forces

    Returns:
        list: 转换后的列表格式 [index, duration_ms, positions..., speeds..., forces...]
    """
    return (
        [action["index"]]
        + [action["duration_ms"]]
        + action["positions"]
        + action["speeds"]
        + action["forces"]
    )


async def main():
    """
    主函数：初始化灵巧手并执行动作序列控制
    """
    # 设置关闭事件监听
    shutdown_event = setup_shutdown_event(logger)

    # 检测灵巧手的波特率和设备ID，初始化client
    (client, slave_id) = await open_modbus_revo1(port_name=None)  # 替换为实际的串口名称, 传None会尝试自动检测

    # 转换动作序列格式
    # sample_action_sequences = random_action_sequences()  # 可选：使用随机动作序列
    mapped_sequences = map(lambda action: action_sequence_info_to_list(action), sample_action_sequences)
    action_sequences = list(mapped_sequences)

    # 上传自定义动作序列到设备
    # 设备最多支持6个自定义动作序列（CustomGesture1 ~ CustomGesture6）
    custom_action_id = libstark.ActionSequenceId.CustomGesture1
    await client.transfer_action_sequence(slave_id, custom_action_id, action_sequences)
    logger.info(f"Custom action sequence uploaded to {custom_action_id}")

    # 读取并验证已上传的动作序列
    action_result: libstark.ActionSequenceItem = await client.get_action_sequence(slave_id, custom_action_id)
    logger.info(f"Action sequence: {action_result.description}")

    # 执行内置动作序列：握拳动作
    logger.info("Executing built-in fist gesture...")
    await client.run_action_sequence(slave_id, libstark.ActionSequenceId.DefaultGestureFist)
    await asyncio.sleep(3)  # 等待内置动作序列执行完成

    # 执行自定义动作序列
    logger.info("Executing custom action sequence...")
    await client.run_action_sequence(slave_id, custom_action_id)

    # 等待关闭事件（Ctrl+C 或其他关闭信号）
    await shutdown_event.wait()

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
