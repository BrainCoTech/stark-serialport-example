#!/usr/bin/env python
import os
import asyncio
from modbus_client_utils import *

# init logging config
filename = os.path.basename(__file__).split(".")[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f"{filename}.log")


def handle_finger_status(device, index, status):
    SKLog.info(f"[{index}] Finger status: {status}")
    if status.is_idle:
        if status.is_opened:
            device.set_finger_positions([60, 60, 100, 100, 100, 100])  # 握手
        elif status.is_closed:
            device.set_finger_positions([0] * 6)  # 张开


# 定期获取手指状态
async def get_finger_status_periodically(device, client):
    SKLog.info("get_finger_status_periodically start")
    index = 0
    # 等待设备空闲，需要串行读写寄存器
    while is_idle(client):
        # 获取手指状态
        try:
            SKLog.debug("get_finger_status")
            device.get_finger_status(
                lambda status: handle_finger_status(device, index, status)
            )
            index += 1
        except Exception as e:
            SKLog.error(f"Error getting finger status: {e}")
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

    # 获取固件版本号等信息
    SKLog.info("get_motorboard_info")
    device.get_motorboard_info(lambda info: SKLog.critical(f"Motorboard info: {info}"))

    # 设置手指位置
    device.set_finger_positions([60, 60, 100, 100, 100, 100])  # 握手

    # 创建并启动异步任务
    asyncio.create_task(get_finger_status_periodically(device, client))
    SKLog.info("Status task started")

    # 等待关闭事件
    await shutdown_event.wait()

    # 关闭资源
    client_close(client)
    SKLog.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
