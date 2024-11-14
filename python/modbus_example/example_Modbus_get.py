#!/usr/bin/env python
import os
from modbus_client_utils import *

filename = os.path.basename(__file__).split('.')[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f"{filename}.log")
pymodbus_apply_logging_config(level=logging.INFO)

def main():
    StarkSDK.set_error_callback(lambda error: SKLog.error(f"Error: {error.message}"))
    ports = serial_ports()
    SKLog.info(f"serial_ports: {ports}")
    if len(ports) == 0: 
        return

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

    # test get info
    SKLog.info('get_hand_type')
    device.get_hand_type(lambda hand_type: SKLog.info(f"Hand type: {hand_type.name}"))
    SKLog.info('get_serialport_cfg')
    device.get_serialport_cfg(lambda cfg: SKLog.info(f"Serialport cfg: {cfg}"))
    SKLog.info('get_motorboard_info')
    device.get_motorboard_info(lambda info: SKLog.info(f"Motorboard info: {info}"))
    SKLog.info('get_force_level')
    device.get_force_level(lambda force_level: SKLog.info(f"Force level: {force_level.name}"))
    SKLog.info('get_finger_positions')
    device.get_finger_positions(lambda positions: SKLog.info(f"Finger positions: {positions}"))

    client_close(client) 

if __name__ == "__main__":
    main()      
