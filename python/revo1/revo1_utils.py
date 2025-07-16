import json
import logging
from logger import getLogger

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

from bc_stark_sdk import main_mod
libstark = main_mod.stark

async def open_modbus_revo1():
    port_name = None
    (protocol, port_name, baudrate, slave_id) = await libstark.auto_detect_modbus_revo1(port_name)
    assert (protocol == libstark.StarkProtocolType.Modbus), "Only Modbus protocol is supported for Revo1"
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

MAX_ANGLES = [55, 90, 70, 70, 70, 70]

def convert_to_position(angles):
    # 将角度映射到 [0, 100] 并限制边界
    mapped = [max(0, min(100, round(angle * 100.0 / max_angle))) for angle, max_angle in zip(angles, MAX_ANGLES)]
    # logger.info(f"Mapped angles: {mapped}")
    return mapped

def convert_to_angle(positions):
    # 将每个位置值限制在 0~100，并映射到角度
    mapped = [min(max(0, pos), 100) * max_angle / 100.0 for pos, max_angle in zip(positions, MAX_ANGLES)]
    # logger.info(f"Mapped positions: {mapped}")
    return mapped

def convert_to_mA(currents):
    # 将电流值转换为毫安
    return [int(current * 6) for current in currents] # 一代灵巧手


