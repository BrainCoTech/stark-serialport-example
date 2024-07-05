#!/usr/bin/env python
import logging
import os
import asyncio
import pathlib
from functools import partial
from serial_utils import *

filename = os.path.basename(__file__).split('.')[0]
StarkSDK.init(log_level=logging.INFO, log_file_name=f'{filename}.log')

dfu_state = StarkDfuState.idle
dfu_progress = 0 # 0 ~ 1.0
event_loop = None

def exit_loop():
    SKLog.info(f"exit application")
    if event_loop is not None:
        event_loop.stop() 

def on_dfu_read(serial_port, device: StarkDevice):
    data = serial_read_data(serial_port)
    if data is not None and len(data) > 0:    
        device.did_receive_data(data)

def on_dfu_state_response(state):
    global dfu_state
    dfu_state = state
    SKLog.info(f"OTA state: {StarkSDK.dfu_state_to_string(dfu_state.value)}")
    if dfu_state == StarkDfuState.idle or dfu_state == StarkDfuState.aborted:
        exit_loop()

def on_dfu_progress_response(progress):
    global dfu_progress
    dfu_progress = progress
    SKLog.info(f"OTA progress: {round(progress, 3)}")    

def main():
    StarkSDK.set_error_callback(lambda error: SKLog.error(f"Error: {error.message}"))
    SKLog.info(f"serial_ports: {serial_ports()}")
    
    # open serial port 8N1
    serial_port = serial_open(serial_port_name, BaudRate.baudrate_115200.value)
    if serial_port is None:
        SKLog.error("Failed to open serial port")
        return
    
    # new device instance
    device_id = 254 # broadcast_id=254 in Firmware V9.2.7
    # device_id = 2 # default_device_id=2 in Firmware V1.2.4
    device = StarkDevice.create_device(device_id, f"{serial_port_name}_{device_id}")

    # set ota callback
    device.set_dfu_read_callback(lambda: event_loop.run_in_executor(None, partial(on_dfu_read, serial_port, device)))
    device.set_dfu_write_callback(lambda value: serial_write_data(serial_port, value))
    device.set_dfu_state_callback(on_dfu_state_response)
    device.set_dfu_progress_callback(on_dfu_progress_response)

    current_dir = pathlib.Path(__file__).resolve()
    parent_dir = current_dir.parent.parent.parent
    ota_bin_path = os.path.join(parent_dir, 'ota_bin', 'FW_MotorController_Release_SecureOTA_modbus_0.1.6.ota') # Modbus固件
    # ota_bin_path = os.path.join(parent_dir, 'ota_bin', 'FW_MotorController_Release_SecureOTA_485_9.2.7.ota')  # 
    if not os.path.exists(ota_bin_path):
        SKLog.warning(f"OTA文件不存在: {ota_bin_path}")
    else:
        device.set_dfu_cfg(dfu_enabling_delay=4, dfu_enabling_interval=10, dfu_applying_delay=5)
        global dfu_state, dfu_progress
        dfu_state = StarkDfuState.idle
        dfu_progress = 0
        device.start_dfu(ota_bin_path)

    # keep the event loop running
    global event_loop
    event_loop = asyncio.get_event_loop()
    try:
        event_loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        event_loop.stop()
        if serial_port is not None:
            serial_close(serial_port)

if __name__ == "__main__":
    main()  

