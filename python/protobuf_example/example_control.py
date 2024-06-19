#!/usr/bin/env python
import os
from serial_utils import *

# init logging config
filename = os.path.basename(__file__).split('.')[0]
StarkSDK.init(log_level=logging.INFO, log_file_name=f'{filename}.log')
        
def main():
    SKLog.info(f"serial_ports: {serial_ports()}")

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

    # close fingers
    device.set_finger_positions([60, 60, 100, 100, 100, 100])
    # wait 1s for close action
    time.sleep(1)
    # open all fingers
    device.set_finger_position(0) 
    # or
    # device.set_finger_positions([0, 0, 0, 0, 0, 0])

    # close serial port
    serial_close(serial_port)

if __name__ == "__main__":
    main()
