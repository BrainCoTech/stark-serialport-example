import logging
from logger import getLogger
import bc_stark_sdk
import json

libstark = bc_stark_sdk.stark

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

# 打印属性，方法
def print_class():
  from utils import inspect_class
  inspect_class(libstark.DeviceInfo)
  logger.info(libstark.DfuState(1))
  logger.info(libstark.DfuState.Idle.int_value)
  inspect_class(libstark.DfuState)

def get_stark_port_name():
    ports = bc_stark_sdk.stark.list_available_ports()
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


