"""
Revo2灵巧手动作序列控制示例

本示例演示如何使用Revo2灵巧手的动作序列功能，包括：
- 自定义动作序列的创建和配置
- 多种控制模式的组合使用
- 动作序列的上传和存储
- 内置动作序列的执行
- 自定义动作序列的执行和管理
- 动作序列数据的读取和验证

注意事项：
- Revo2支持多种控制模式：位置时间控制、位置速度控制、电流控制、速度控制
- 位置值为65535 (0xFFFF) 时，表示手指保持原来角度
- 动作序列参数使用物理量（角度、速度、电流）
- 设备最多支持6个自定义动作序列
"""

import asyncio
import sys
from utils import setup_shutdown_event
from revo2_utils import *

# 二代灵巧手动作序列配置
# Revo2支持更复杂的动作序列，包含多种控制模式和详细的参数配置
sample_action_sequences = [
    {
        "index": 0,                          # 动作序列索引，用于标识该动作序列在队列中的位置
        "duration_ms": 1000,                 # 动作序列的执行时间（毫秒）
        "mode": 1,                           # 控制模式：1=位置时间控制，2=位置速度控制，3=电流控制，4=速度控制
        "positions": [24, 0, 0, 0, 0, 0],    # 手指位置（°）：[拇指, 食指, 中指, 无名指, 小指, 手腕]
        "durations": [500, 100, 100, 100, 100, 100],  # 各手指到达目标位置的时间（毫秒）
        "speeds": [0, 0, 0, 0, 0, 0],        # 手指速度（°/s）：仅在位置速度控制模式时生效
        "currents": [0, 0, 0, 0, 0, 0],      # 手指电流（mA）：仅在电流控制模式时生效
    },
    {
        "index": 1,
        "duration_ms": 2000,
        "mode": 1,                           # 位置时间控制
        "positions": [45, 44, 84, 84, 84, 84],  # 执行握拳动作：各手指闭合到指定角度
        "durations": [2000, 2000, 100, 100, 100, 100],  # 拇指和食指缓慢闭合，其他手指快速闭合
        "speeds": [0, 0, 0, 0, 0, 0],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 2,
        "duration_ms": 1000,
        "mode": 1,
        "positions": [24, 0, 0, 0, 0, 0],    # 拇指部分闭合，其他手指完全张开
        "durations": [500, 100, 100, 100, 100, 100],
        "speeds": [0, 0, 0, 0, 0, 0],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 3,
        "duration_ms": 2000,
        "mode": 1,
        "positions": [45, 44, 84, 84, 84, 84],  # 重复握拳动作
        "durations": [2000, 2000, 100, 100, 100, 100],
        "speeds": [0, 0, 0, 0, 0, 0],
        "currents": [0, 0, 0, 0, 0, 0],
    },
]


def action_sequence_info_to_list(action):
    """
    将动作序列字典转换为列表格式

    将动作序列的配置信息转换为SDK所需的列表格式，用于上传到设备。
    Revo2的动作序列格式比Revo1更复杂，包含更多的控制参数。

    Args:
        action (dict): 动作序列配置字典，包含index、duration_ms、mode、positions、durations、speeds、currents

    Returns:
        list: 转换后的列表格式 [index, duration_ms, mode, positions..., durations..., speeds..., currents...]
    """
    return (
        [action["index"], action["duration_ms"], action["mode"]]
        + action["positions"]
        + action["durations"]
        + action["speeds"]
        + action["currents"]
    )


async def main():
    """
    主函数：初始化灵巧手并执行动作序列控制
    """
    # 设置关闭事件监听
    shutdown_event = setup_shutdown_event(logger)

    # 连接Revo2设备
    (client, slave_id) = await open_modbus_revo2(port_name=None) # 替换为实际的串口名称, 传None会尝试自动检测

    # 转换动作序列格式
    mapped_sequences = map(lambda action: action_sequence_info_to_list(action), sample_action_sequences)
    action_sequences = list(mapped_sequences)

    # 上传自定义动作序列到设备
    # 设备最多支持6个自定义动作序列（CustomGesture1 ~ CustomGesture6）
    custom_action_id = libstark.ActionSequenceId.CustomGesture1
    await client.transfer_action_sequence(slave_id, custom_action_id, action_sequences)
    logger.info(f"Custom action sequence uploaded to {custom_action_id}")

    # 读取并验证已上传的动作序列
    # 可以读取6个内置动作序列和最多24个自定义动作序列
    action_result: libstark.ActionSequenceItem = await client.get_action_sequence(slave_id, custom_action_id)
    logger.info(f"Action sequence: {action_result.description}")

    # 执行内置动作序列示例（可选）
    # logger.info("Executing built-in fist gesture...")
    # await client.run_action_sequence(slave_id, libstark.ActionSequenceId.DefaultGestureFist)
    # await asyncio.sleep(3)  # 等待内置动作序列执行完成

    # 执行自定义动作序列
    logger.info("Executing custom action sequence...")
    await client.run_action_sequence(slave_id, custom_action_id)

    # logger.info("Clearing custom action sequence...")
    # await client.clear_action_sequence(slave_id, custom_action_id)
    # await client.run_action_sequence(slave_id, custom_action_id)

    # 等待关闭事件（Ctrl+C 或其他关闭信号）
    await shutdown_event.wait()

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
