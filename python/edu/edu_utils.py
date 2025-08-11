import json
import logging
from logger import getLogger

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

from bc_stark_sdk import main_mod

libstark = main_mod.stark
libedu = main_mod.edu
lib = main_mod

async def open_modbus_device():
    """
    自动检测并打开Revo灵巧手的Modbus连接

    Revo灵巧手支持Modbus等通讯协议，推荐使用Modbus协议。
    该函数会自动检测端口、波特率和设备ID，并建立连接。

    Returns:
        tuple: (client, slave_id) - Modbus客户端实例和从站设备ID
            client: ModbusClient实例，用于与设备通信
            slave_id: int，Modbus从站ID（设备地址，范围1-247）

    Raises:
        AssertionError: 当检测不到设备或协议类型不是Modbus时抛出异常

    Example:
        >>> client, slave_id = await open_modbus_device()
        >>> # 或者手动指定参数:
        >>> # client: libstark.PyDeviceContext = await libstark.modbus_open(port_name, baudrate)
    """
    # 指定端口名称，None表示自动检测第一个可用端口
    # 多设备连接时可指定具体端口，例如: "/dev/ttyUSB0" (Linux/macOS) 或 "COM3" (Windows)
    port_name = None

    # 快速检测模式配置
    # True: 仅检测常用波特率(115200, 460800等)和默认设备ID，速度快
    # False: 检测设备ID范围1~247，覆盖更全面但耗时较长(约30秒)
    quick = True

    # 自动检测第一个可用从机设备
    # 返回: (协议类型, 端口名, 波特率, 从站ID)
    (protocol, port_name, baudrate, slave_id) = await libstark.auto_detect_device(
        port_name, quick
    )

    # 验证检测到的协议类型，确保是Modbus协议
    assert (
        protocol == libstark.StarkProtocolType.Modbus
    ), "Only Modbus protocol is supported"

    # 使用检测到的参数建立Modbus连接
    client: libstark.PyDeviceContext = await libstark.modbus_open(port_name, baudrate)

    # 获取并记录设备详细信息
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    # 根据设备型号记录具体版本信息
    if device_info.is_revo1():
        if device_info.is_revo1_touch():
            logger.info(f"Revo1-触觉版")  # 触觉版Revo1
        else:
            logger.info(f"Revo1-标准版")  # 标准版Revo1
    elif device_info.is_revo2():
        if device_info.is_revo2_touch():
            logger.info(f"Revo2-触觉版")  # 触觉版Revo2
        else:
            logger.info(f"Revo2-标准版")  # 标准版Revo2

    return (client, slave_id)


def _get_first_port_name(ports_data, device_type):
    """
    从端口数据中提取第一个可用端口名称的通用函数

    该函数用于解析各种设备的端口扫描结果，提取第一个可用的端口名称。
    支持JSON格式的端口数据解析，包含错误处理和日志记录。

    Args:
        ports_data (bytes): 端口数据，JSON格式的字节串
            预期格式: [{"port_name": "COM1", ...}, {"port_name": "COM2", ...}]
        device_type (str): 设备类型名称，用于日志标识（如"Stark", "Glove", "Armband"）

    Returns:
        str or None: 第一个可用端口的名称，解析失败时返回None
            成功时返回端口名称，如"/dev/ttyUSB0"、"COM3"等

    Note:
        - 该函数会优先选择列表中的第一个端口
        - 包含完整的错误处理，确保程序稳定性
        - 所有操作都会记录到日志中便于调试
    """
    logger.info(f"Available {device_type} ports: {ports_data}")

    # 解码字节对象并解析 JSON 格式数据
    try:
        ports_json = json.loads(ports_data.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        logger.error(f"Failed to parse {device_type} ports data: {e}")
        return None

    # 检查解析结果是否为空
    if not ports_json:
        logger.error(f"Parse {device_type} ports failed - empty result")
        return None

    # 检查是否找到任何端口
    if len(ports_json) == 0:
        logger.error(f"No {device_type} ports found")
        return None

    # 提取第一个端口的名称
    port_name = ports_json[0]["port_name"]
    logger.info(f"Using {device_type} port: {port_name}")
    return port_name


def get_stark_port_name():
    """
    获取第一个可用的Stark设备端口名称

    扫描系统中所有可用的串口设备，返回第一个检测到的Stark设备端口名称。
    该函数适用于Revo灵巧手等Stark系列设备的端口检测。

    Returns:
        str or None: 第一个可用端口的名称，如果没有找到端口或解析失败则返回None
            成功时返回端口名称，如"/dev/ttyUSB0"、"COM3"等

    Note:
        - 该函数返回的端口名称可用于手动指定连接端口
        - 建议在调用open_modbus_device()之前使用此函数确认端口可用性
        - 如果系统中有多个Stark设备，该函数只返回第一个检测到的端口

    Example:
        >>> port = get_stark_port_name()
        >>> if port:
        ...     print(f"Found Stark device at: {port}")
        ... else:
        ...     print("No Stark device found")
    """
    # 获取系统中所有可用的串口设备列表
    # 该函数会扫描所有串口并返回JSON格式的端口信息
    ports = libstark.list_available_ports()
    return _get_first_port_name(ports, "Stark")


def get_glove_port_name():
    """
    获取第一个可用的手套设备端口名称

    通过USB VID/PID扫描手套设备，返回第一个检测到的端口名称。
    使用特定的厂商ID(VID=21059)和产品ID(PID=6)来识别手套设备。

    Returns:
        str or None: 第一个可用端口的名称，如果没有找到端口或解析失败则返回None
            成功时返回端口名称，如"/dev/ttyUSB0"、"COM3"等

    Note:
        - VID=21059, PID=6 是手套设备的USB标识符
        - 该函数专门用于检测USB连接的手套设备
        - 如果系统中有多个手套设备，该函数只返回第一个检测到的端口

    Example:
        >>> port = get_glove_port_name()
        >>> if port:
        ...     print(f"Found glove device at: {port}")
        ... else:
        ...     print("No glove device found")
    """
    # 通过USB VID/PID扫描手套设备
    # VID=21059: 厂商标识符, PID=6: 手套设备产品标识符
    ports = main_mod.available_usb_ports(21059, 6)
    return _get_first_port_name(ports, "Glove")


def get_armband_port_name():
    """
    获取第一个可用的臂带设备端口名称

    通过USB VID/PID扫描臂带设备，返回第一个检测到的端口名称。
    使用特定的厂商ID(VID=21059)和产品ID(PID=1)来识别臂带设备。

    Returns:
        str or None: 第一个可用端口的名称，如果没有找到端口或解析失败则返回None
            成功时返回端口名称，如"/dev/ttyUSB0"、"COM3"等

    Note:
        - VID=21059, PID=1 是臂带设备的USB标识符
        - 该函数专门用于检测USB连接的臂带设备
        - 如果系统中有多个臂带设备，该函数只返回第一个检测到的端口

    Example:
        >>> port = get_armband_port_name()
        >>> if port:
        ...     print(f"Found armband device at: {port}")
        ... else:
        ...     print("No armband device found")
    """
    # 通过USB VID/PID扫描臂带设备
    # VID=21059: 厂商标识符, PID=1: 臂带设备产品标识符
    ports = main_mod.available_usb_ports(21059, 1)
    return _get_first_port_name(ports, "Armband")
