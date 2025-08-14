"""
Education SDK Utilities

这个模块提供了与 bc-edu-sdk 交互的实用工具函数，
主要用于设备发现、端口管理和通用功能。
"""

import json
import logging
from typing import Optional, List, Dict, Any

from logger import getLogger
from bc_edu_sdk import main_mod

# 配置常量
VENDOR_ID = 21059  # BrainCo 厂商ID
GLOVE_PRODUCT_ID = 6  # 手套设备产品ID
ARMBAND_PRODUCT_ID_PRIMARY = 1  # 臂环设备主要产品ID
ARMBAND_PRODUCT_ID_SECONDARY = 5  # 臂环设备备用产品ID

# 初始化日志和SDK模块
# logger = getLogger(logging.DEBUG) # 可选：使用DEBUG级别日志
logger = getLogger(logging.INFO)  # 默认：使用INFO级别日志
libedu = main_mod

def get_usb_available_ports() -> None:
    """
    获取所有可用的USB端口信息

    这是一个便捷函数，用于显示当前系统中所有可用的USB端口。
    主要用于调试和端口发现。
    """
    libedu.get_usb_available_ports()


def _get_first_port_name(ports_data: bytes, device_type: str) -> Optional[str]:
    """
    从端口数据中提取第一个可用端口名称的通用函数

    该函数用于解析各种设备的端口扫描结果，提取第一个可用的端口名称。
    支持JSON格式的端口数据解析，包含错误处理和日志记录。

    Args:
        ports_data: 端口数据，JSON格式的字节串
            预期格式: [{"port_name": "COM1", ...}, {"port_name": "COM2", ...}]
        device_type: 设备类型名称，用于日志标识（如"Stark", "Glove", "Armband"）

    Returns:
        第一个可用端口的名称，解析失败时返回None
        成功时返回端口名称，如"/dev/ttyUSB0"、"COM3"等

    Note:
        - 该函数会优先选择列表中的第一个端口
        - 包含完整的错误处理，确保程序稳定性
        - 所有操作都会记录到日志中便于调试
    """
    logger.info(f"Available {device_type} ports: {ports_data}")

    try:
        # 解码字节对象并解析JSON格式数据
        ports_json: List[Dict[str, Any]] = json.loads(ports_data.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        logger.error(f"Failed to parse {device_type} ports data: {e}")
        return None

    # 检查解析结果是否为空列表
    if not ports_json:
        logger.warning(f"No {device_type} ports found in scan results")
        return None

    # 提取第一个端口的名称
    try:
        port_name = ports_json[0]["port_name"]
        logger.info(f"Using {device_type} port: {port_name}")
        return port_name
    except (KeyError, IndexError) as e:
        logger.error(f"Invalid port data structure for {device_type}: {e}")
        return None

def get_glove_port_name() -> Optional[str]:
    """
    获取第一个可用的手套设备端口名称

    通过USB VID/PID扫描手套设备，返回第一个检测到的端口名称。
    使用特定的厂商ID和产品ID来识别手套设备。

    Returns:
        第一个可用端口的名称，如果没有找到端口或解析失败则返回None
        成功时返回端口名称，如"/dev/ttyUSB0"、"COM3"等

    Note:
        - 使用VID=21059, PID=6识别手套设备
        - 该函数专门用于检测USB连接的手套设备
        - 如果系统中有多个手套设备，该函数只返回第一个检测到的端口

    Example:
        >>> port = get_glove_port_name()
        >>> if port:
        ...     print(f"Found glove device at: {port}")
        ... else:
        ...     print("No glove device found")
    """
    try:
        ports = libedu.available_usb_ports(VENDOR_ID, GLOVE_PRODUCT_ID)
        return _get_first_port_name(ports, "Glove")
    except Exception as e:
        logger.error(f"Error scanning for glove devices: {e}")
        return None


def get_armband_port_name() -> Optional[str]:
    """
    获取第一个可用的臂环设备端口名称

    通过USB VID/PID扫描臂环设备，返回第一个检测到的端口名称。
    支持多个产品ID，提高设备兼容性。

    Returns:
        第一个可用端口的名称，如果没有找到端口或解析失败则返回None
        成功时返回端口名称，如"/dev/ttyUSB0"、"COM3"等

    Note:
        - 首先尝试主要产品ID (PID=1)，然后尝试备用产品ID (PID=5)
        - 该函数专门用于检测USB连接的臂环设备
        - 如果系统中有多个臂环设备，该函数只返回第一个检测到的端口

    Example:
        >>> port = get_armband_port_name()
        >>> if port:
        ...     print(f"Found armband device at: {port}")
        ... else:
        ...     print("No armband device found")
    """
    try:
        # 首先尝试主要产品ID
        ports = libedu.available_usb_ports(VENDOR_ID, ARMBAND_PRODUCT_ID_PRIMARY)
        port_name = _get_first_port_name(ports, "Armband")

        if port_name:
            return port_name

        # 如果主要产品ID未找到设备，尝试备用产品ID
        logger.info("Primary armband product ID not found, trying secondary ID...")
        ports = libedu.available_usb_ports(VENDOR_ID, ARMBAND_PRODUCT_ID_SECONDARY)
        return _get_first_port_name(ports, "Armband")

    except Exception as e:
        logger.error(f"Error scanning for armband devices: {e}")
        return None


def get_all_device_ports() -> Dict[str, Optional[str]]:
    """
    获取所有支持设备的端口信息

    Returns:
        包含所有设备类型和对应端口的字典
        格式: {"glove": "COM3", "armband": "/dev/ttyUSB0"}
    """
    devices = {
        "glove": get_glove_port_name(),
        "armband": get_armband_port_name()
    }

    logger.info(f"Device scan results: {devices}")
    return devices


def is_device_connected(device_type: str) -> bool:
    """
    检查指定类型的设备是否已连接

    Args:
        device_type: 设备类型 ("glove" 或 "armband")

    Returns:
        如果设备已连接返回True，否则返回False
    """
    if device_type.lower() == "glove":
        return get_glove_port_name() is not None
    elif device_type.lower() == "armband":
        return get_armband_port_name() is not None
    else:
        logger.warning(f"Unknown device type: {device_type}")
        return False


def scan_and_report_devices() -> None:
    """
    扫描并报告所有连接的设备

    这是一个便捷函数，用于快速查看当前连接的所有设备状态。
    """
    logger.info("Scanning for connected devices...")

    devices = get_all_device_ports()
    connected_devices = [name for name, port in devices.items() if port is not None]

    if connected_devices:
        logger.info(f"Found {len(connected_devices)} connected device(s):")
        for device_name, port in devices.items():
            if port:
                logger.info(f"  - {device_name.capitalize()}: {port}")
    else:
        logger.warning("No devices found. Please check connections.")

    # 显示所有可用端口作为参考
    logger.info("All available USB ports:")
    get_usb_available_ports()


def print_afe_timestamps(logger, data) -> None:
    """
    打印AFE数据的时间戳信息

    Args:
        logger: 日志记录器
        data: AFE数据列表
    """
    if len(data) <= 6:
        for item in data:
            logger.info(f"{item}")
        return
    for item in data[:3]:
        logger.info(f"{item}")
    logger.info("...")
    for item in data[-3:]:
        logger.info(f"{item}")
