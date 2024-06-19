#!/usr/bin/env python
import os
from modbus_client_utils import *

filename = os.path.basename(__file__).split('.')[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f'{filename}.log')
pymodbus_apply_logging_config(level=logging.INFO)   

def main():
    StarkSDK.set_error_callback(lambda error: SKLog.error(f"Error: {error.message}"))
    SKLog.info(f"serial_ports: {serial_ports()}")

    client = client_connect(port=serial_port_name, baudrate=BaudRate.baudrate_115200)
    if client is None:
        SKLog.error("Failed to open modbus serial port")
        return
    
    # new modbus device instance
    slave_id = 1 # default slave_id is 1 in Modbus Firmware
    device = StarkDevice.create_device(slave_id, f"{serial_port_name}_{slave_id}")

    # set modbus device write/read registers callbacks
    device.set_write_registers_callback(lambda register_address, values: client_write_registers(client, register_address, values=values, slave=slave_id))
    device.set_read_holding_registers_callback(lambda register_address, count: client_read_holding_registers(client, register_address, count, slave=slave_id))
    device.set_read_input_registers_callback(lambda register_address, count: client_read_input_registers(client, register_address, count, slave=slave_id))

    # set force level 
    device.set_force_level(force_level=ForceLevel.full)

    # set serial device id, device will reboot after device id changed
    # device.set_serial_device_id(2)

    # set serial baudrate, device will reboot after baudrate changed
    # device.set_serial_baudrate(baudrate=BaudRate.baudrate_115200)
    
    client_close(client) 

if __name__ == "__main__":
    main()      

