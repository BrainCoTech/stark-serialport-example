#!/usr/bin/env python
import os
from modbus_client_utils import *

filename = os.path.basename(__file__).split('.')[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f'{filename}.log')
pymodbus_apply_logging_config(level=logging.INFO)    

def main():
    StarkSDK.set_error_callback(lambda error: SKLog.error(f"Error: {error.message}"))
    ports = serial_ports()
    SKLog.info(f"serial_ports: {ports}")
    if len(ports) == 0:
        return

    broadcast_enable = True

    client = client_connect(port=serial_port_name, baudrate=BaudRate.baudrate_115200, broadcast_enable=broadcast_enable)
    if client is None:
        SKLog.error("Failed to open modbus serial port")
        return

    # new modbus device instance
    slave_id = 0 if broadcast_enable else 1
    device = StarkDevice.create_device(slave_id, f"{serial_port_name}_{slave_id}")

    # set modbus device write/read registers callbacks
    device.set_write_registers_callback(lambda register_address, values: client_write_registers(client, register_address, values=values, slave=slave_id))
    device.set_read_holding_registers_callback(lambda register_address, count: client_read_holding_registers(client, register_address, count, slave=slave_id))
    device.set_read_input_registers_callback(lambda register_address, count: client_read_input_registers(client, register_address, count, slave=slave_id))

    # close fingers
    device.set_finger_positions([60, 60, 100, 100, 100, 100])
    # wait 1s for close action
    time.sleep(1)
    # open all fingers
    device.set_finger_position(0) 
    # or
    # device.set_finger_positions([0, 0, 0, 0, 0, 0])

    client_close(client) 

if __name__ == "__main__":
    main()      
