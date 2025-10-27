"""
Revo2灵巧手工具函数模块

本模块提供了Revo2灵巧手的常用工具函数，包括：
- Modbus连接的自动检测和建立
- 设备端口的扫描和选择
- 数据类型转换和处理功能
- 设备信息获取和验证
"""

import json
import sys
import logging
from logger import getLogger

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

from bc_stark_sdk import main_mod

libstark = main_mod
libstark.init_config(libstark.StarkProtocolType.Modbus)


async def open_modbus_revo2(port_name=None, quick=True):
    """
    自动检测并打开Revo2灵巧手的Modbus连接

    Revo2灵巧手支持Modbus、CANFD、EtherCAT三种通讯协议。该函数会自动检测端口、波特率和设备ID，
    并建立连接。

    Args:
        port_name (str, optional): 串口名称，默认为None表示自动检测第一个可用端口。
            多串口时可指定具体端口，例如: "/dev/ttyUSB0"
        quick (bool, optional): 快速检测模式配置，默认为True。
            True: 仅检测常用波特率和默认设备ID，速度快
            False: 检测设备ID范围1~247，覆盖更全面但耗时较长

    Returns:
        tuple: (client, slave_id) - Modbus客户端实例和设备从站ID

    Raises:
        AssertionError: 当检测到的协议不是Modbus时

    Example:
        client, slave_id = await open_modbus_revo2()
        # 或者手动指定参数:
        # client: libstark.PyDeviceContext = await libstark.modbus_open(port_name, baudrate)
    """
    try:
        # 自动检测第一个可用从机
        (protocol, detected_port_name, baudrate, slave_id) = (
            await libstark.auto_detect_modbus_revo2(port_name, quick)
        )
        # 验证检测到的协议类型
        assert (
            protocol == libstark.StarkProtocolType.Modbus
        ), "Only Modbus protocol is supported for Revo2"
    except Exception as e:
        logger.error(e)
        sys.exit(1)

    # set_latency_by_com_or_serial(detected_port_name)
    # 建立Modbus连接
    client: libstark.PyDeviceContext = await libstark.modbus_open(detected_port_name, baudrate)

    # 获取设备信息
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    if device_info.is_revo2():
        if device_info.is_revo2_touch():
            logger.info(f"触觉版")
        else:
            logger.info(f"标准版")

    return (client, slave_id)


def get_stark_port_name():
    """
    获取第一个可用的Stark设备端口名称

    扫描系统中所有可用的串口设备，返回第一个检测到的端口名称。

    Returns:
        str: 第一个可用端口的名称，如果没有找到端口或解析失败则返回None

    Note:
        该函数返回的端口名称可用于手动指定连接端口
    """
    # 获取系统中所有可用的端口列表
    ports = libstark.list_available_ports()
    logger.info(f"available_ports: {ports}")

    # 解码字节数据并解析JSON格式的端口信息
    ports_json = json.loads(ports.decode("utf-8"))
    if not ports_json:
        logger.error("parse ports failed")
        return

    if len(ports) == 0:
        logger.error("No ports found")
        return

    # 选择第一个可用端口
    port_name = ports_json[0]["port_name"]
    logger.info(f"Using port: {port_name}")
    return port_name

def is_positions_open(status: main_mod.MotorStatusData) -> bool:
    """
    判断手指是否处于张开状态

    Args:
        status (main_mod.MotorStatusData): 电机状态数据对象

    Returns:
        bool: 如果手指张开则返回True，否则返回False
    """
    return status.positions <= [400, 400, 0, 0, 0, 0]

def is_positions_closed(status: main_mod.MotorStatusData) -> bool:
    """
    判断手指是否处于闭合状态

    Args:
        status (main_mod.MotorStatusData): 电机状态数据对象

    Returns:
        bool: 如果手指闭合则返回True，否则返回False
    """

    return status.positions >= [399, 399, 950, 950, 950, 950]
