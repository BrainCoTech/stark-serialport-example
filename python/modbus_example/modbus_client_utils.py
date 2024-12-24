import sys
import pathlib
from pymodbus.utilities import hexlify_packets
from pymodbus.client.serial import ModbusSerialClient
from pymodbus.logging import pymodbus_apply_logging_config

current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
sys.path.append(str(parent_dir))
sys.path.append(parent_dir.joinpath("protobuf_example"))
from lib import * # import stark lib
from protobuf_example.serial_utils import *


def is_connected(client: ModbusSerialClient):
    if hasattr(client, "connected"):
        return getattr(client, "connected")
    else:
        return client.connect()


def is_idle(client: ModbusSerialClient):
    return is_connected(client) and not client._in_waiting()


def client_connect(
    port: str, baudrate: BaudRate
) -> ModbusSerialClient:
    client = ModbusSerialClient(port=port, baudrate=baudrate.value, timeout=3)
    # pymodbus==3.7.0, broadcast_enable parameters removed. (https://github.com/pymodbus-dev/pymodbus/pull/2272/files)
    if not is_connected(client):
        raise ValueError(
            f"Failed to connect to Modbus client on port {port} at baudrate {baudrate}"
        )
    return client


def client_close(client: ModbusSerialClient) -> None:
    if isinstance(client, ModbusSerialClient):
        try:
            client.close()
        except Exception as e:
            SKLog.info(f"Error closing client: {e}")


def client_write_data(client: ModbusSerialClient, data: bytes) -> int:
    SKLog.debug(f"client_write_data, {len(data)} bytes, {hexlify_packets(data)}")
    # SKLog.debug(f"client_write_data, {len(data)} bytes, {hexlify_packets_without_0x(data)}")

    if isinstance(client, ModbusSerialClient) and is_connected(client):
        try:
            ret = client.send(data)  # Cleanup recv buffer before send
            # ret = client.socket.write(data)
            SKLog.debug(f"client_write_data, ret: {ret}")
            return 0
        except Exception as e:
            SKLog.error(f"Error writing data: {e}")
    else:
        SKLog.error("client_write_data, client is not connected")
    return -1


def client_write_registers(
    client: ModbusSerialClient, address: int, values: list, slave: int
) -> int:
    SKLog.debug(
        f"client_write_registers, slave: {slave}, address: {address}, len={len(values)}, {hexlify_packets(values)}"
    )
    # SKLog.debug(f"client_write_registers, address: {address}, len={len(values)}, {hexlify_packets_without_0x(values)}")

    if isinstance(client, ModbusSerialClient) and is_connected(client):
        try:
            if isinstance(values, list):
                if len(values) == 1:
                    client.write_register(address=address, value=values[0], slave=slave)
                    # time.sleep(0.1)
                else:
                    client.write_registers(address=address, values=values, slave=slave)
            else:
                SKLog.warning(
                    f"client_write_registers, values type error: {type(values)}"
                )
            return 0
        except Exception as e:
            SKLog.error(f"Error writing registers: {e}")
    else:
        SKLog.error("client_write_registers, client is not connected")
    return -1


def client_read_holding_registers(
    client: ModbusSerialClient, register_address: int, count: int = 1, slave: int = 0
) -> list:
    SKLog.debug(
        f"read_holding_registers, register_address: {register_address}, count: {count}"
    )

    if isinstance(client, ModbusSerialClient) and is_connected(client):
        try:
            response = client.read_holding_registers(
                register_address, count=count, slave=slave
            )
            values = response.registers
            SKLog.debug(
                f"read_holding_registers, register_address: {register_address}, len={len(values)}, {hexlify_packets(values)}"
            )
            return values
        except Exception as e:
            SKLog.error(f"Error reading holding registers: {e}")
    else:
        SKLog.error("read_holding_registers, client is not connected")


def client_read_input_registers(
    client: ModbusSerialClient, register_address: int, count: int = 1, slave: int = 0
) -> list:
    SKLog.debug(
        f"read_input_registers, register_address: {register_address}, count: {count}"
    )

    if isinstance(client, ModbusSerialClient) and is_connected(client):
        try:
            response = client.read_input_registers(
                register_address, count=count, slave=slave
            )
            values = response.registers
            SKLog.debug(
                f"read_input_registers, register_address: {register_address}, len={len(values)}, {hexlify_packets(values)}"
            )
            return values
        except Exception as e:
            SKLog.error(f"Error reading input registers: {e}")
    else:
        SKLog.error("read_input_registers, client is not connected")
