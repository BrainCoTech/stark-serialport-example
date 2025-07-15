import asyncio
import sys
from utils import setup_shutdown_event
from revo2_utils import libstark, logger, open_modbus_revo2

# 二代灵巧手动作序列
# - 动作序列索引 (index)：动作序列的索引，用于标识该动作序列在队列中的位置
# - 持续时间 (duration_ms)：该动作序列的执行时间，单位为毫秒
# - 控制模式 (u16), 位置时间控制：1，位置速度控制：2，电流控制：3，速度控制：4
# - 手指位置 (u16): 6 个手指位置的物理量（°）, 值为65535 (0xFFFF)时, 表示手指保持原来角度
# - 手指速度 (u16): 6 个手指速度的物理量，值为手指转动速度（°/s）
# - 手指电流 (u16): 6 个手指电流的物理量，值为电流(mA)
sample_action_sequences = [
    {
        "index": 0,
        "duration_ms": 1000,
        "mode": 1,  # 位置时间控制
        "positions": [24, 0, 0, 0, 0, 0],
        "durations": [500, 100, 100, 100, 100, 100],
        "speeds": [0, 0, 0, 0, 0, 0],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 1,
        "duration_ms": 2000,
        "mode": 1,  # 位置时间控制
        "positions": [45, 44, 84, 84, 84, 84],
        "durations": [2000, 2000, 100, 100, 100, 100],
        "speeds": [0, 0, 0, 0, 0, 0],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 2,
        "duration_ms": 1000,
        "mode": 1,
        "positions": [24, 0, 0, 0, 0, 0],
        "durations": [500, 100, 100, 100, 100, 100],
        "speeds": [0, 0, 0, 0, 0, 0],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 3,
        "duration_ms": 2000,
        "mode": 1,
        "positions": [45, 44, 84, 84, 84, 84],
        "durations": [2000, 2000, 100, 100, 100, 100],
        "speeds": [0, 0, 0, 0, 0, 0],
        "currents": [0, 0, 0, 0, 0, 0],
    },
]


def action_sequence_info_to_list(action):
    return (
        [action["index"], action["duration_ms"], action["mode"]]
        + action["positions"]
        + action["durations"]
        + action["speeds"]
        + action["currents"]
    )


### main.py
async def main():
    shutdown_event = setup_shutdown_event(logger)
    (client, slave_id) = await open_modbus_revo2()

    # fmt: off
    mapped_sequences = map(lambda action: action_sequence_info_to_list(action), sample_action_sequences)
    action_sequences = list(mapped_sequences)

    # 写入自定义动作序列，最多支持6个自定义动作序列
    custom_action_id = libstark.ActionSequenceId.CustomGesture1
    await client.transfer_action_sequence(slave_id, custom_action_id, action_sequences)

    # 读取动作序列，包括6个内置动作序列和最多24个自定义动作序列
    action_result = await client.get_action_sequence(slave_id, custom_action_id)
    logger.info(f"Action sequence: {action_result.description}")

    # 执行内置动作序列
    # await client.run_action_sequence(slave_id, libstark.ActionSequenceId.DefaultGestureFist)
    # await asyncio.sleep(3) # 等待内置动作序列执行完成

    # 运行动作序列，可以是内置动作序列或自定义动作序列
    await client.run_action_sequence(slave_id, custom_action_id)

    # 等待关闭事件
    await shutdown_event.wait()

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
