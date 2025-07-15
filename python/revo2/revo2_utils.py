import json
import logging
from logger import getLogger

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

from bc_stark_sdk import main_mod
libstark = main_mod.stark

async def open_modbus_revo2():
    # port_name = '/dev/ttyUSB0'
    port_name = None
    (protocol, port_name, baudrate, slave_id) = await libstark.auto_detect_modbus_revo2(port_name)
    assert (protocol == libstark.StarkProtocolType.Modbus), "Only Modbus protocol is supported for Revo2"
    client = await libstark.modbus_open(port_name, baudrate)
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")
    return (client, slave_id)

def get_stark_port_name():
    ports = libstark.list_available_ports()
    logger.info(f"available_ports: {ports}")

    # 解码字节对象并解析 JSON
    ports_json = json.loads(ports.decode("utf-8"))
    if not ports_json:
        logger.error("parse ports failed")
        return

    if len(ports) == 0:
        logger.error("No ports found")
        return

    port_name = ports_json[0]["port_name"]
    logger.info(f"Using port: {port_name}")
    return port_name
