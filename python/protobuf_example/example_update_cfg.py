#!/usr/bin/env python
import os
from serial_utils import *

# init logging config
filename = os.path.basename(__file__).split('.')[0]
StarkSDK.init(log_level=logging.INFO, log_file_name=f'{filename}.log')

def main():
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
    StarkSDK.set_write_data_callback(lambda value: serial_write_data(serial_port, value))
    StarkSDK.set_read_data_callback(lambda: serial_read_data(serial_port))

    # new device instance
    device_id = 10 # serial_device_id is 10 default in Firmware V9.2.7
    device = StarkDevice.create_device(device_id, f"{serial_port_name}_{device_id}")

    # set force level
    device.set_force_level(force_level=ForceLevel.full)

    # set serial device id, device will reboot after device id changed
    # device.set_serial_device_id(10)

    # set serial baudrate, device will reboot after baudrate changed
    # device.set_serial_baudrate(baudrate=BaudRate.baudrate_115200)

    # close serial port
    serial_close(serial_port)

if __name__ == "__main__":
    main()
