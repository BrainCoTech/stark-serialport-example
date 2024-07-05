#!/usr/bin/env python
import asyncio
from modbus_client_utils import *

filename = os.path.basename(__file__).split('.')[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f'{filename}.log', c_log_level=LogLevel.warning)
pymodbus_apply_logging_config(level=logging.INFO) 

event_loop = None
dfu_state = StarkDfuState.idle
dfu_progress = 0 # 0 ~ 1.0

def on_dfu_read(client, device: StarkDevice):
    data = client.recv(size=None)
    SKLog.debug(f"read_ota_resp, len: {len(data)} bytes, {hexlify_packets(data)}")
    if len(data) > 0:
        device.did_receive_data(data)

def on_dfu_state_response(state):
    global dfu_state
    dfu_state = state
    SKLog.info(f"OTA state: {StarkSDK.dfu_state_to_string(dfu_state.value)}")
    if dfu_state == StarkDfuState.idle or dfu_state == StarkDfuState.aborted:
        SKLog.info(f"exit application")
        if event_loop is not None:
            event_loop.stop() 

def on_dfu_progress_response(progress):
    global dfu_progress
    dfu_progress = progress
    SKLog.info(f"OTA progress: {round(progress, 3)}")    
   
def main():
    StarkSDK.set_error_callback(lambda error: SKLog.error(f"Error: {error.message}"))
    SKLog.info(f"serial_ports: {serial_ports()}")

    broadcast_enable = True
    
    client = client_connect(port=serial_port_name, baudrate=BaudRate.baudrate_115200, broadcast_enable=broadcast_enable)
    if client is None:
        SKLog.error("Failed to open modbus serial port")
        return
    
    # new modbus device instance
    slave_id = 0 if broadcast_enable else 1
    device = StarkDevice.create_device(slave_id, f"{serial_port_name}_{slave_id}")

    # set modbus device write registers callback
    device.set_write_registers_callback(lambda register_address, values: client_write_registers(client, register_address, values=values, slave=slave_id))

    # set ota callback
    device.set_dfu_read_callback(lambda: event_loop.run_in_executor(None, partial(on_dfu_read, client, device)))
    device.set_dfu_write_callback(lambda value: client_write_data(client, bytes(value)))
    device.set_dfu_state_callback(on_dfu_state_response)
    device.set_dfu_progress_callback(on_dfu_progress_response)

    # device.abort_dfu()
    # exit(0)

    current_dir = pathlib.Path(__file__).resolve()
    parent_dir = current_dir.parent.parent.parent
    ota_bin_path = os.path.join(parent_dir, 'ota_bin', 'FW_MotorController_Release_SecureOTA_modbus_0.1.6.ota') # Modbus固件
    # ota_bin_path = os.path.join(parent_dir, 'ota_bin', 'FW_MotorController_Release_SecureOTA_485_9.2.7.ota')  # protobuf固件
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
        if client is not None:
            client_close(client)

if __name__ == "__main__":
    main()  

