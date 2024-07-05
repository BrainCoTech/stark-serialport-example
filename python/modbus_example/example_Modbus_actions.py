#!/usr/bin/env python
import os
from modbus_client_utils import *

filename = os.path.basename(__file__).split('.')[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f'{filename}.log', c_log_level=LogLevel.info)
pymodbus_apply_logging_config(level=logging.INFO)    

# 动作序列定义
sample_action_sequences = [
    {
        "index": 0,
        "duration_ms": 2000,
        "positions": [0, 0, 100, 100, 100, 100],
        "speeds": [10, 20, 30, 40, 50, 60],
        "forces": [5, 10, 15, 20, 25, 30]
    },
    {
        "index": 1,
        "duration_ms": 2000,
        "positions": [100, 100, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31]
    },
    {
        "index": 2,
        "duration_ms": 2000,
        "positions": [0, 0, 100, 100, 100, 100],
        "speeds": [10, 20, 30, 40, 50, 60],
        "forces": [5, 10, 15, 20, 25, 30]
    },
    {
        "index": 3,
        "duration_ms": 2000,
        "positions": [100, 100, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31]
    },
    {
        "index": 4,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31]
    },
]

def action_sequence_info_to_list(action):
    return [action['index']] + [action['duration_ms']] + action['positions'] + action['speeds'] + action['forces']

def main():
    StarkSDK.set_error_callback(lambda error: SKLog.error(f"Error: {error.message}"))
    SKLog.info(f"serial_ports: {serial_ports()}")

    client = client_connect(port=serial_port_name, baudrate=BaudRate.baudrate_115200)
    if client is None:
        SKLog.error("Failed to open modbus serial port")
        return
    
    # new modbus device instance
    slave_id = 1
    device = StarkDevice.create_device(slave_id, f"{serial_port_name}_{slave_id}")

    # set modbus device write/read registers callbacks
    device.set_write_registers_callback(lambda register_address, values: client_write_registers(client, register_address, values=values, slave=slave_id))
    device.set_read_holding_registers_callback(lambda register_address, count: client_read_holding_registers(client, register_address, count, slave=slave_id))
    device.set_read_input_registers_callback(lambda register_address, count: client_read_input_registers(client, register_address, count, slave=slave_id))

    SKLog.info(f"Action Sequence: {len(sample_action_sequences)}")
    for action in sample_action_sequences:
        print(f"{action},")

    mapped_sequences = map(lambda action: action_sequence_info_to_list(action), sample_action_sequences)
    action_sequences = list(mapped_sequences)

    action_id = ActionSequenceId.custom_gesture_1

    # 测试写入 action sequence
    device.transfer_action_sequence(action_id, action_sequences=action_sequences)
    # 测试保存 action sequence
    device.save_action_sequence(action_id)

    # 测试读取 action sequence
    device.get_action_sequence(action_id, cb=lambda action_sequences: SKLog.info(f"get Action Sequence: {action_sequences}"))
    # 测试运行 action sequence
    device.run_action_sequence(action_id)
    
    client_close(client) 

if __name__ == "__main__":
    main()      



