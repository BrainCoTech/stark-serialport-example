#!/usr/bin/env python
import os
from serial_utils import *

# init logging config
filename = os.path.basename(__file__).split(".")[0]
StarkSDK.init(log_level=logging.INFO, log_file_name=f"{filename}.log")


def main():
    ports = serial_ports()
    SKLog.info(f"serial_ports: {ports}")
    if len(ports) == 0:
        return

    # open serial port 8N1
    SKLog.debug("serial_open")
    serial_port = serial_open(serial_port_name, BaudRate.baudrate_115200.value)
    if serial_port is None:
        SKLog.error("Failed to open serial port")
        exit(1)

    # set serial port read/write callback
    # fmt: off
    StarkSDK.set_write_data_callback(lambda value: serial_write_data(serial_port, value))
    StarkSDK.set_read_data_callback(lambda: serial_read_data(serial_port))

    # new device instance
    device_id = 10  # broadcast_id=254 in Firmware V9.2.7
    device = StarkDevice.create_device(device_id, f"{serial_port_name}_{device_id}")

    device.get_motorboard_info(lambda info: SKLog.critical(f"Motorboard info: {info}"))
    device.get_serialport_cfg(lambda cfg: SKLog.info(f"Serialport cfg: {cfg}"))
    device.get_force_level(lambda level: SKLog.info(f"Force level: {level.name}"))
    device.get_voltage(lambda voltage: SKLog.info(f"Voltage: {voltage}"))
    device.get_max_current(lambda current: SKLog.info(f"Max current: {current}"))
    device.get_finger_status(lambda status: SKLog.info(f"Finger status: {status}"))
    
    SKLog.debug("close serial port")
    serial_close(serial_port)


if __name__ == "__main__":
    main()
