#!/usr/bin/env python
import os
import asyncio
from modbus_client_utils import *

filename = os.path.basename(__file__).split(".")[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f"{filename}.log")

# 动作序列定义
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
]


def action_sequence_info_to_list(action):
    return (
        [action["index"]]
        + [action["duration_ms"]]
        + action["positions"]
        + action["speeds"]
        + action["forces"]
    )


async def main():
    setup_shutdown_event()
    StarkSDK.set_error_callback(lambda error: SKLog.error(f"Error: {error.message}"))

    ports = serial_ports()
    SKLog.info(f"serial_ports: {ports}")
    if len(ports) == 0:
        SKLog.error("No serial ports found")
        return

    client = client_connect(port=serial_port_name, baudrate=BaudRate.baudrate_115200)
    if client is None:
        SKLog.error("Failed to open modbus serial port")
        return

    broadcast_enable = False
    # default slave_id is 1 in Modbus Firmware
    slave_id = 0 if broadcast_enable else 1

    # new modbus device instance
    device = StarkDevice.create_device(slave_id, f"{serial_port_name}_{slave_id}")

    # set modbus device write/read registers callbacks
    device.set_write_registers_callback(
        lambda register_address, values: client_write_registers(
            client, register_address, values=values, slave=slave_id
        )
    )
    device.set_read_holding_registers_callback(
        lambda register_address, count: client_read_holding_registers(
            client, register_address, count, slave=slave_id
        )
    )
    device.set_read_input_registers_callback(
        lambda register_address, count: client_read_input_registers(
            client, register_address, count, slave=slave_id
        )
    )

    global sample_action_sequences
    SKLog.info(f"Action Sequence: {len(sample_action_sequences)}")
    for action in sample_action_sequences:
        print(f"{action},")

    # fmt: off
    mapped_sequences = map(lambda action: action_sequence_info_to_list(action), sample_action_sequences)
    action_sequences = list(mapped_sequences)

    # 写入动作序列，自定义动作序列1~6
    action_id = ActionSequenceId.custom_gesture_1
    device.transfer_action_sequence(action_id, action_sequences=action_sequences)
    # 保存动作序列，自定义动作序列1~6
    device.save_action_sequence(action_id)

    # 读取动作序列，内置动作序列或自定义动作序列1~6
    device.get_action_sequence(action_id,cb=lambda action_sequences: SKLog.info(f"get Action Sequence: {action_sequences}"))
    # 运行动作序列，内置动作序列或自定义动作序列1~6
    device.run_action_sequence(action_id)

    # 等待关闭事件
    # await shutdown_event.wait()

    # 关闭资源
    client_close(client)
    SKLog.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
