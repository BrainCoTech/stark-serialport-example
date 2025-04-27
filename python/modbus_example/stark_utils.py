import logging
from logger import getLogger
import bc_stark_sdk
import json

libstark = bc_stark_sdk.stark

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)


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
