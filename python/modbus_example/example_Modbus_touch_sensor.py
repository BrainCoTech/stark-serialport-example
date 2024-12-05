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
    
    SKLog.info("get_motorboard_info")  # 获取固件版本号等信息
    device.get_motorboard_info(lambda info: SKLog.critical(f"Motorboard info: {info}"))
    
    # 启用触觉传感器功能
    SKLog.info("set_touch_sensor_enabled")
    # device.set_touch_sensor_enabled(0b00000)  # 0b00000: 5个手指传感器全部禁用
    device.set_touch_sensor_enabled(0b11111)  # 0b11111: 5个手指传感器全部启用
    await asyncio.sleep(0.5)  # 等待触觉传感器初始化完成
    
    # 获取触觉传感器功能启用状态
    SKLog.info("get_touch_sensor_enabled")
    device.get_touch_sensor_enabled(
        lambda finger_bits: SKLog.info(f"Touch sensor enabled: {(finger_bits & 0x1F):05b}")
    )
    
    # 获取触觉传感器状态、三维力数值
    SKLog.info("get_touch_sensor_status")  
    device.get_touch_sensor_status(
        lambda status: SKLog.info(f"Touch sensor status: {status}")
    )
    
    # 获取触觉传感器raw数据
    SKLog.info("get_touch_sensor_raw_data")
    device.get_touch_sensor_raw_data(
        lambda raw_data: SKLog.info(f"Touch sensor raw data: {raw_data}")
    )

    # 触觉传感器复位，发送传感器采集通道复位指令。在执行该指令时，手指传感器尽量不要受力。
    # SKLog.info("touch_sensor_reset")
    # device.touch_sensor_reset(finger_bits=0b11111)  # 复位指定手指的传感器采集通道

    # 触觉传感器参数校准，参数校准指令。 当空闲状态下的三维力数值不为0时，可通过该寄存器进行校准。该指令的执行时间较长，期间的数据无法作为参考。建议忽略在该参数校准寄存器设置后十秒内的数据。在执行该指令时，手指传感器不能受力， 否则将导致校准后的传感器数据异常。
    # SKLog.info("touch_sensor_calibrate")
    # device.touch_sensor_calibrate(finger_bits=0b11111)  # 校准指定手指的传感器采集通道

    # 等待关闭事件
    # await shutdown_event.wait()

    # 关闭资源
    client_close(client)
    SKLog.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
