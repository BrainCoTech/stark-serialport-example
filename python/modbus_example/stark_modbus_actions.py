import asyncio
import sys
from utils import setup_shutdown_event
from stark_utils import get_stark_port_name, libstark, logger

sample_action_sequences = [
    {
        "index": 0,
        "duration_ms": 2000,
        "positions": [0, 0, 100, 100, 100, 100],
        "speeds": [10, 20, 30, 40, 50, 60],
        "forces": [5, 10, 15, 20, 25, 30],
    },
    {
        "index": 1,
        "duration_ms": 2000,
        "positions": [100, 100, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 2,
        "duration_ms": 2000,
        "positions": [0, 0, 100, 100, 100, 100],
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
        "positions": [0, 0, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 5,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],
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
    return (
        [action["index"]]
        + [action["duration_ms"]]
        + action["positions"]
        + action["speeds"]
        + action["forces"]
    )


### main.py
async def main():
    libstark.init_config(libstark.StarkFirmwareType.V1Standard)
    shutdown_event = setup_shutdown_event(logger)

    port_name = get_stark_port_name()
    if port_name is None:
        return
    slave_id = 1
    client = await libstark.modbus_open(
        port_name, libstark.Baudrate.Baud115200, slave_id
    )

    logger.debug("get_device_info")
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.firmware_version}")

    # fmt: off
    # sample_action_sequences = random_action_sequences()
    mapped_sequences = map(lambda action: action_sequence_info_to_list(action), sample_action_sequences)
    action_sequences = list(mapped_sequences)

    # 写入自定义动作序列，最多支持6个自定义动作序列
    custom_action_id = libstark.ActionSequenceId.CustomGesture1
    await client.transfer_action_sequence(slave_id, custom_action_id, action_sequences)

    # 读取动作序列，包括内置动作序列和自定义动作序列1~6
    action_result = await client.get_action_sequence(slave_id, custom_action_id)
    logger.info(f"Action sequence: {action_result.description}")

    # 执行内置动作序列
    await client.run_action_sequence(slave_id, libstark.ActionSequenceId.DefaultGestureFist)
    await asyncio.sleep(3) # 等待内置动作序列执行完成

    # 运行动作序列，包括内置动作序列和自定义动作序列1~6
    await client.run_action_sequence(slave_id, custom_action_id)

    # 等待关闭事件
    await shutdown_event.wait()

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
