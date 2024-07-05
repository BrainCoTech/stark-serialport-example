#!/usr/bin/env python
import os
from modbus_client_utils import *

filename = os.path.basename(__file__).split('.')[0]
StarkSDK.init(isModbus=True, log_level=logging.DEBUG, log_file_name=f'{filename}.log')
pymodbus_apply_logging_config(level=logging.INFO)   

def init_register_rw_callback(device, client, slave_id):
    # set modbus device write/read registers callbacks
    device.set_write_registers_callback(lambda register_address, values: client_write_registers(client, register_address, values=values, slave=slave_id))
    device.set_read_holding_registers_callback(lambda register_address, count: client_read_holding_registers(client, register_address, count, slave=slave_id))
    device.set_read_input_registers_callback(lambda register_address, count: client_read_input_registers(client, register_address, count, slave=slave_id))    

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


def main():
    StarkSDK.set_error_callback(lambda error: SKLog.error(f"Error: {error.message}"))
    SKLog.info(f"serial_ports: {serial_ports()}")

    # old_baudrate = BaudRate.baudrate_57600
    # new_baudrate = BaudRate.baudrate_115200
    old_baudrate = BaudRate.baudrate_115200
    new_baudrate = BaudRate.baudrate_57600
    client = client_connect(port=serial_port_name, baudrate=old_baudrate)
    if client is None:
        SKLog.error("Failed to open modbus serial port")
        return
    
    # new modbus device instance
    old_slave_id = 1 # default slave_id is 1 in Modbus Firmware
    new_slave_id = 2
    
    device = create_device(client, old_slave_id)
    test_update_slave_id(device, client, new_slave_id)
    exit(0)

    # update serial baudrate, device will reboot
    # test_update_baudrate(device, client, new_baudrate)
    # exit(0)

    SKLog.info(f"set_force_level")
    device.set_force_level(force_level=ForceLevel.full)

    SKLog.info(f"set_turbo_mode")
    device.set_turbo_mode(True)
    device.get_turbo_mode(lambda turbo_mode: SKLog.info(f"get_turbo_mode, {turbo_mode}"))

    SKLog.info(f"set_turbo_conf")
    device.set_turbo_conf(turbo_interval=1000, turbo_duration=2000)
    device.get_turbo_conf(lambda turbo_interval, turbo_duration: SKLog.info(f"get_turbo_conf, turbo_interval: {turbo_interval}, duration: {turbo_duration}"))
    
    SKLog.info(f"set_auto_calibration")
    device.set_auto_calibration(enabled=True)
    device.get_auto_calibration(lambda enabled: SKLog.info(f"auto_calibration_enabled: {enabled}"))

    # SKLog.info(f"send_manual_calibration_cmd")
    # device.send_manual_calibration_cmd()

    client_close(client) 

if __name__ == "__main__":
    main()      

