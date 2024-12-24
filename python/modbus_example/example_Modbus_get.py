#!/usr/bin/env python
import os
import asyncio
from modbus_client_utils import *

filename = os.path.basename(__file__).split(".")[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f"{filename}.log")


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
    
    # default slave_id is 1 in Modbus Firmware
    slave_id = 1

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

    # ----------------- 以下为获取示例代码 -----------------
    # fmt: off
    SKLog.info("get_hand_type")  # 获取手类型
    device.get_hand_type(lambda hand_type: SKLog.info(f"Hand type: {hand_type.name}"))
    SKLog.info("get_serialport_cfg")  # 获取串口配置，波特率等
    device.get_serialport_cfg(lambda cfg: SKLog.info(f"Serialport cfg: {cfg}"))
    SKLog.info("get_motorboard_info")  # 获取固件版本号等信息
    device.get_motorboard_info(lambda info: SKLog.critical(f"Motorboard info: {info}"))
    SKLog.info("get_force_level")  # 获取力量等级，大-中-小
    device.get_force_level(lambda force_level: SKLog.info(f"Force level: {force_level.name}"))
    SKLog.info("get_voltage")  # 获取电量
    device.get_voltage(lambda voltage: SKLog.info(f"Voltage: {voltage:.1f} mV"))

    device.set_finger_positions([60, 60, 100, 100, 100, 100])  # 设置手指位置
    SKLog.info("get_finger_status")  # 获取手指状态
    device.get_finger_status(lambda status: SKLog.info(f"Finger status: {status}"))

    # 关闭资源
    client_close(client)
    SKLog.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
