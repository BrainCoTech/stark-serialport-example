#!/usr/bin/env python
import os
import asyncio
from modbus_client_utils import *

filename = os.path.basename(__file__).split(".")[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f"{filename}.log")


def init_register_rw_callback(device, client, slave_id):
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


def create_device(client, slave_id):
    device = StarkDevice.create_device(slave_id, f"{serial_port_name}_{slave_id}")
    init_register_rw_callback(device, client, slave_id)
    return device


def test_update_slave_id(device, client, new_slave_id):
    # 1. before set_serial_device_id, get current serialport_cfg, should be old_slave_id
    device.get_serialport_cfg(lambda cfg: SKLog.info(f"serialport_cfg: {cfg}"))
    # 2. set serial device id, device will reboot after device id changed
    old_slave_id = device.slave_id
    device.set_serial_device_id(new_slave_id)
    # 3. re-create device and set modbus client read/write registers callbacks
    device = create_device(client, new_slave_id)
    # 4. wait 3s for device reboot
    time.sleep(3)
    # 5. get new device id, should be new_slave_id
    device.get_serialport_cfg(lambda cfg: SKLog.info(f"new serialport_cfg: {cfg}"))

    # test revert to old slave_id
    device.set_serial_device_id(old_slave_id)
    device = create_device(client, old_slave_id)
    time.sleep(3)
    device.get_serialport_cfg(lambda cfg: SKLog.info(f"new serialport_cfg: {cfg}"))


def test_update_baudrate(device, client, baudrate):
    # 1. before set_serial_baudrate, get current serialport_cfg, should be old baudrate
    device.get_serialport_cfg(lambda cfg: SKLog.info(f"serialport_cfg: {cfg}"))
    # 2. set serial baudrate, device will reboot after baudrate changed
    device.set_serial_baudrate(baudrate=baudrate)
    # 3.wait 5s for device reboot
    time.sleep(5)
    # 4. re-create modbus client with new baudrate
    client_close(client)
    client = client_connect(port=serial_port_name, baudrate=baudrate)
    # 5. re-create device and set modbus client read/write registers callbacks
    device = create_device(client, device.slave_id)
    # 6.get new baudrate, should be new baudrate
    device.get_serialport_cfg(lambda cfg: SKLog.info(f"new serialport_cfg: {cfg}"))


async def main():
    setup_shutdown_event()
    StarkSDK.set_error_callback(lambda error: SKLog.error(f"Error: {error.message}"))

    ports = serial_ports()
    SKLog.info(f"serial_ports: {ports}")
    if len(ports) == 0:
        SKLog.error("No serial ports found")
        return

    # cur_baudrate = BaudRate.baudrate_57600
    cur_baudrate = BaudRate.baudrate_115200
    client = client_connect(port=serial_port_name, baudrate=cur_baudrate)
    if client is None:
        SKLog.error("Failed to open modbus serial port")
        return

    # ------------------- test update slave id -------------------
    old_slave_id = 1  # default slave_id is 1 in Modbus Firmware
    new_slave_id = 2
    device = create_device(client, old_slave_id)
    test_update_slave_id(device, client, new_slave_id)
    exit(0)

    # ------------------- test update baudrate -------------------
    # update serial baudrate, device will reboot
    # new_baudrate = BaudRate.baudrate_115200
    # new_baudrate = BaudRate.baudrate_57600
    # test_update_baudrate(device, client, new_baudrate)
    # exit(0)

    # ------------------- 设置力量等级，大-中-小 -------------------
    SKLog.info(f"set_force_level")
    device.set_force_level(force_level=ForceLevel.full)

    # ------------------- 设置Turbo模式及参数 -------------------
    SKLog.info(f"set_turbo_mode")
    device.set_turbo_mode(True)
    device.get_turbo_mode(
        lambda turbo_mode: SKLog.info(f"get_turbo_mode, {turbo_mode}")
    )

    SKLog.info(f"set_turbo_conf")
    device.set_turbo_conf(turbo_interval=1000, turbo_duration=2000)
    device.get_turbo_conf(
        lambda turbo_interval, turbo_duration: SKLog.info(
            f"get_turbo_conf, turbo_interval: {turbo_interval}, duration: {turbo_duration}"
        )
    )

    # ------------------- 位置校准设置 -------------------
    # SKLog.info(f"set_auto_calibration") # 设置是否在上电后自动执行位置校准
    # device.set_auto_calibration(enabled=True)
    # device.get_auto_calibration(lambda enabled: SKLog.info(f"auto_calibration_enabled: {enabled}"))

    # SKLog.info(f"send_manual_calibration_cmd") # 位置校准
    # device.send_manual_calibration_cmd()

    # 等待关闭事件
    # await shutdown_event.wait()

    # 关闭资源
    client_close(client)
    SKLog.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
