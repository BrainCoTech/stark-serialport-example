#!/usr/bin/env python
import os
import asyncio
from modbus_client_utils import *

# init logging config
filename = os.path.basename(__file__).split(".")[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f"{filename}.log")


def handle_finger_status(device, device_id, index, status):
    SKLog.debug(f"[{device_id}-{index}] Finger status")
    if status.is_idle:
        if status.is_opened:
            SKLog.info(f"[{device_id}-{index}] 握手")
            device.set_finger_positions([60, 60, 100, 100, 100, 100])  # 握手
        elif status.is_closed:
            SKLog.info(f"[{device_id}-{index}] 张开")
            device.set_finger_positions([0] * 6)  # 张开

devices = []
device_ids = [1, 2] # 灵巧手的设备ID

# 定期获取手指状态
async def get_finger_status_periodically(client):
    SKLog.info("get_finger_status_periodically start")
    index = 0
    # 等待设备空闲，需要串行读写寄存器
    while is_idle(client) :
        for i in range(len(devices)):
            device = devices[i]
            device_id = device_ids[i]
            try:
                # 获取手指状态
                SKLog.debug(f"get_finger_status {i+1}")
                device.get_finger_status(
                    lambda status: handle_finger_status(device, device_id, index, status)
                )
            except Exception as e:
                SKLog.error(f"Error getting finger status: {e}")
                break
        
        index += 1
        await asyncio.sleep(0.002)


async def main():
    shutdown_event = setup_shutdown_event()
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

    for slave_id in device_ids:  
        device = StarkDevice.create_device(slave_id, f"{serial_port_name}_{slave_id}")
        devices.append(device)
        # set modbus device write/read registers callbacks
        device.set_write_registers_callback(
            lambda register_address, values, slave=slave_id: client_write_registers(
                client, register_address, values=values, slave=slave
            )
        )
        device.set_read_holding_registers_callback(
            lambda register_address, count, slave=slave_id: client_read_holding_registers(
                client, register_address, count, slave=slave
            )
        )
        device.set_read_input_registers_callback(
            lambda register_address, count, slave=slave_id: client_read_input_registers(
                client, register_address, count, slave=slave
            )
        )
        device.get_motorboard_info(lambda info: SKLog.critical(f"device[{slave_id}] Motorboard info: {info}"))
        device.set_finger_positions([60, 60, 100, 100, 100, 100])  # 握手

    # 创建并启动异步任务
    asyncio.create_task(get_finger_status_periodically(client))
    SKLog.info("Status task started")

    # 等待关闭事件
    await shutdown_event.wait()

    # 关闭资源
    client_close(client)
    SKLog.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
