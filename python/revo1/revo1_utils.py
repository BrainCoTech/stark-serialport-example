import json
import logging
from logger import getLogger

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

from bc_stark_sdk import main_mod

libstark = main_mod


async def open_modbus_revo1(port_name = None, quick = True):
    """
    自动检测并打开Revo1灵巧手的Modbus连接

    Revo1灵巧手支持Modbus和Protobuf两种通讯协议，推荐使用Modbus协议。
    该函数会自动检测端口、波特率和设备ID，并建立连接。

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
        client, slave_id = await open_modbus_revo1()
        # 或者手动指定参数:
        # client: libstark.PyDeviceContext = await libstark.modbus_open(port_name, baudrate)
    """
    # 自动检测第一个可用从机
    (protocol, detected_port_name, baudrate, slave_id) = await libstark.auto_detect_modbus_revo1(
        port_name, quick
    )

    # 验证检测到的协议类型
    assert (
        protocol == libstark.StarkProtocolType.Modbus
    ), "Only Modbus protocol is supported for Revo1"

    # 建立Modbus连接
    client: libstark.PyDeviceContext = await libstark.modbus_open(detected_port_name, baudrate)

    # 获取设备信息
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    if device_info.is_revo1():
        if device_info.is_revo1_touch():
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


# 各关节最大角度限制 (单位: 度)
# 索引对应关节: [拇指, 食指, 中指, 无名指, 小指, 手腕]
MAX_ANGLES = [55, 90, 70, 70, 70, 70]


def convert_to_position(angles):
    """
    将角度值转换为位置百分比

    根据各关节的最大角度限制，将实际角度值映射到0-100的位置百分比范围。

    Args:
        angles (list): 各关节的角度值列表，长度应为6

    Returns:
        list: 对应的位置百分比列表，范围[0-100]

    Example:
        angles = [30, 45, 35, 35, 35, 35]
        positions = convert_to_position(angles)  # [55, 50, 50, 50, 50, 50]
    """
    # 将角度按比例映射到[0, 100]范围并限制边界
    mapped = [
        max(0, min(100, round(angle * 100.0 / max_angle)))
        for angle, max_angle in zip(angles, MAX_ANGLES)
    ]
    return mapped


def convert_to_angle(positions):
    """
    将位置百分比转换为角度值

    根据各关节的最大角度限制，将0-100的位置百分比映射回实际角度值。

    Args:
        positions (list): 各关节的位置百分比列表，范围[0-100]

    Returns:
        list: 对应的角度值列表

    Example:
        positions = [50, 50, 50, 50, 50, 50]
        angles = convert_to_angle(positions)  # [27.5, 45.0, 35.0, 35.0, 35.0, 35.0]
    """
    # 将位置值限制在[0, 100]范围内并按比例映射到角度
    mapped = [
        min(max(0, pos), 100) * max_angle / 100.0
        for pos, max_angle in zip(positions, MAX_ANGLES)
    ]
    return mapped


def convert_to_mA(currents):
    """
    将电流值转换为毫安单位

    根据一代灵巧手的硬件特性，将电流值转换为毫安单位。

    Args:
        currents (list): 原始电流值列表

    Returns:
        list: 毫安单位的电流值列表

    Note:
        转换系数为6，该系数专门适用于一代灵巧手硬件
    """
    return [int(current * 6) for current in currents]
