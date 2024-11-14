#!/usr/bin/env python
import os
import asyncio
from serial_utils import *

# init logging config
filename = os.path.basename(__file__).split(".")[0]
StarkSDK.init(log_level=logging.INFO, log_file_name=f"{filename}.log")


def handle_finger_status(device, index, status):
    SKLog.info(f"[{index}] Finger status: {status}")
    if status.is_idle:
        if status.is_opened:
            device.set_finger_positions([60, 60, 100, 100, 100, 100])  # 握手
        elif status.is_closed:
            device.set_finger_positions([0] * 6)  # 张开


# 定期获取手指状态
async def get_finger_status_periodically(device, serial_port):
    SKLog.info("get_finger_status_periodically start")
    index = 0
    while not serial_in_waiting(serial_port):
        # 获取手指状态
        try:
            SKLog.debug("get_finger_status")
            device.get_finger_status(
                lambda status: handle_finger_status(device, index, status)
            )
            index += 1
        except Exception as e:
            SKLog.error(f"Error getting finger status: {e}")
        await asyncio.sleep(0.01)


async def main():
    shutdown_event = setup_shutdown_event()

    ports = serial_ports()
    SKLog.info(f"serial_ports: {ports}")
    if len(ports) == 0:
        return

    # open serial port 8N1
    serial_port = serial_open(serial_port_name, BaudRate.baudrate_115200.value)
    if serial_port is None:
        SKLog.error("Failed to open serial port")
        exit(1)

    # set serial port read/write callback
    # fmt: off
    StarkSDK.set_read_data_callback(lambda: serial_read_data(serial_port))
    StarkSDK.set_write_data_callback(lambda value: serial_write_data(serial_port, value))

    # new device instance
    device_id = 10  # default is 10 in Firmware V9.2.X
    device = StarkDevice.create_device(device_id, f"{serial_port_name}_{device_id}")

    SKLog.info("get_motorboard_info")
    device.get_motorboard_info(lambda info: SKLog.critical(f"Motorboard info: {info}"))
    SKLog.info("get_serialport_cfg")
    device.get_serialport_cfg(lambda cfg: SKLog.info(f"Serialport cfg: {cfg}"))

    device.set_finger_positions([60, 60, 100, 100, 100, 100])  # 握手

    # 创建并启动异步任务
    asyncio.create_task(get_finger_status_periodically(device, serial_port))
    SKLog.info("Status task started")

    # 等待关闭事件
    await shutdown_event.wait()

    # 关闭资源
    serial_close(serial_port)
    SKLog.info("Serial port closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
