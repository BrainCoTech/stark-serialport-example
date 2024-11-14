#!/usr/bin/env python
import asyncio
from modbus_client_utils import *

filename = os.path.basename(__file__).split(".")[0]
StarkSDK.init(isModbus=True, log_level=logging.INFO, log_file_name=f"{filename}.log")

# event_loop = None
dfu_state = StarkDfuState.idle
dfu_progress = 0  # 0 ~ 1.0


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
        SKLog.info(f"ota done")


def on_dfu_progress_response(progress):
    global dfu_progress
    dfu_progress = progress
    SKLog.info(f"OTA progress: {round(progress, 3)}")


async def main():
    shutdown_event = setup_shutdown_event()
    StarkSDK.set_error_callback(lambda error: SKLog.error(f"Error: {error.message}"))

    ports = serial_ports()
    SKLog.info(f"serial_ports: {ports}")
    if len(ports) == 0:
        SKLog.error("No serial ports found")
        return

    client = client_connect(port=serial_port_name, baudrate=BaudRate.baudrate_115200)
    if client is None:
        SKLog.error("Failed to open modbus serial port")
        return

    broadcast_enable = False
    # default slave_id is 1 in Modbus Firmware
    slave_id = 0 if broadcast_enable else 1

    # new modbus device instance
    device = StarkDevice.create_device(slave_id, f"{serial_port_name}_{slave_id}")

    # set modbus device write/read registers callbacks
    device.set_write_registers_callback(
        lambda register_address, values: client_write_registers(
            client, register_address, values=values, slave=slave_id
        )
    )

    # set ota callback
    # fmt: off
    loop = asyncio.get_event_loop()
    device.set_dfu_read_callback(lambda: loop.run_in_executor(None, partial(on_dfu_read, client, device)))
    device.set_dfu_write_callback(lambda value: client_write_data(client, bytes(value)))
    device.set_dfu_state_callback(on_dfu_state_response)
    device.set_dfu_progress_callback(on_dfu_progress_response)

    # 固件升级文件路径
    current_dir = pathlib.Path(__file__).resolve()
    parent_dir = current_dir.parent.parent.parent
    # fmt: off
    ota_bin_path = os.path.join(parent_dir,"ota_bin","modbus/FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota",)  # Modbus固件
    # ota_bin_path = os.path.join(parent_dir, 'ota_bin', 'protobuf/FW_MotorController_Release_SecureOTA_485_9.2.9.ota')  # protobuf固件
    # ota_bin_path = os.path.join(parent_dir,"ota_bin","modbus/FW_MotorController_Release_SecureOTA_V1.8.8F-OTA.ota",)  # NOTE: 触觉手固件, 触觉手不兼容先前的固件，请勿交叉升级
    if not os.path.exists(ota_bin_path):
        SKLog.warning(f"OTA文件不存在: {ota_bin_path}")
    else:
        device.set_dfu_cfg(dfu_enabling_delay=5, dfu_enabling_interval=10, dfu_applying_delay=5)
        global dfu_state, dfu_progress
        dfu_state = StarkDfuState.idle
        dfu_progress = 0
        device.start_dfu(ota_bin_path)

    # 等待关闭事件
    await shutdown_event.wait()

    # 关闭资源
    client_close(client)
    SKLog.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
