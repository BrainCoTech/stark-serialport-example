"""
Glove Data Collection Example

这个示例展示了如何使用 bc-edu-sdk 连接手套设备并收集传感器数据，
包括 Flex（弯曲传感器）、IMU（惯性测量单元）和磁力计数据。
"""

import asyncio
import numpy as np
from typing import Optional, List

from edu_utils import *
from model import FlexData, IMUData, MagData

# 配置常量
NUM_CHANNELS = 6  # Flex传感器通道数
FLEX_BUFFER_LENGTH = 1250  # Flex数据缓冲区长度（数据点数）
FETCH_DATA_COUNT = 1000  # 每次获取的数据点数
BAUDRATE = 115200  # 串口波特率
DATA_PRINT_INTERVAL = 0.5  # 数据打印间隔（秒）

# 传感器系数
ACC_COEFFICIENT = 0.0001220703125  # 加速度计系数 (1/8192)
GYRO_COEFFICIENT = 0.06103515625  # 陀螺仪系数 (1/16.4)
MAG_COEFFICIENT = 0.00152587890625  # 磁力计系数 (1/65536)

# 全局变量
flex_seq_num: Optional[int] = None  # Flex数据包序号
flex_values = np.zeros((NUM_CHANNELS, FLEX_BUFFER_LENGTH))  # Flex传感器数据缓冲区

def print_mag_data() -> None:
    """
    获取并打印磁力计数据
    """
    mag_buff = libedu.get_mag_buffer(FETCH_DATA_COUNT, clean=True)
    logger.info(f"Got mag buffer len={len(mag_buff)}")

    if len(mag_buff) == 0:
        return

    # 只打印第一条Mag数据作为示例
    mag_data = MagData.from_data(mag_buff[0])
    logger.info(f"MagData: {mag_data}")


def print_imu_data() -> None:
    """
    获取并打印IMU（惯性测量单元）数据
    """
    imu_buff = libedu.get_imu_buffer(FETCH_DATA_COUNT, clean=True)
    logger.info(f"Got IMU buffer len={len(imu_buff)}")

    if len(imu_buff) == 0:
        return

    # 只打印第一条IMU数据作为示例
    row = imu_buff[0]
    logger.info(f"IMU raw data: {row}")
    imu_data = IMUData.from_data(row)
    logger.info(f"IMUData: {imu_data}")


def update_flex_buffer(flex_data: FlexData) -> None:
    """
    更新Flex传感器数据缓冲区

    Args:
        flex_data: Flex传感器数据对象
    """
    global flex_seq_num

    seq_num = flex_data.seq_num

    # 检查数据包序号连续性
    if flex_seq_num is not None:
        # 处理正常递增和重复序号的情况
        if seq_num < flex_seq_num:
            logger.warning(f"Data sequence backward: expected >= {flex_seq_num}, got {seq_num}")
        elif seq_num > flex_seq_num + 1:
            logger.warning(f"Data sequence gap: expected {flex_seq_num + 1}, got {seq_num}")
        elif seq_num == flex_seq_num:
            logger.debug(f"Duplicate sequence number: {seq_num}")
            return  # 跳过重复的数据包

    flex_seq_num = seq_num

    # 将通道数据分割为各个通道
    channel_values = np.array_split(flex_data.channel_values, NUM_CHANNELS)

    # 更新每个通道的数据缓冲区
    for i in range(NUM_CHANNELS):
        flex_values[i] = np.roll(flex_values[i], -1)  # 数据向左滚动
        flex_values[i, -1] = channel_values[i][0]  # 添加最新数据


def print_flex_data() -> None:
    """
    获取并打印Flex传感器数据
    """
    flex_buff = libedu.get_flex_buffer(FETCH_DATA_COUNT, clean=True)
    logger.info(f"Got flex buffer len={len(flex_buff)}")

    if len(flex_buff) == 0:
        return

    flex_data_list = []
    for row in flex_buff:
        flex_data = FlexData.from_data(row)
        flex_data_list.append(flex_data)
        update_flex_buffer(flex_data)

    print_flex_timestamps(flex_data_list)

def print_flex_timestamps(data: List[FlexData]) -> None:
    """
    打印Flex数据的时间戳信息

    Args:
        data: Flex数据列表
    """
    if len(data) <= 6:
        for item in data:
            logger.info(f"{item}")
        return

    # 如果数据太多，只打印前3个和后3个
    for item in data[:3]:
        logger.info(f"{item}")
    logger.info("...")
    for item in data[-3:]:
        logger.info(f"{item}")


def print_all_sensor_data() -> None:
    """
    打印所有传感器数据（IMU、磁力计、Flex）
    """
    print_imu_data()
    print_mag_data()
    print_flex_data()


async def connect_device() -> bool:
    """
    连接手套设备并开始数据流

    Returns:
        bool: 连接成功返回True，失败返回False
    """
    port_name = get_glove_port_name()
    if port_name is None:
        logger.error("No glove device found")
        return False

    try:
        device = libedu.PyEduDevice(port_name, BAUDRATE)
        await device.start_data_stream(libedu.MessageParser("Glove-device", libedu.MsgType.Edu))
        logger.info("Listening for messages...")

        # 获取设备状态信息
        await device.get_dongle_pair_stat()
        await asyncio.sleep(0.5)

        # 可选：配置传感器采样率
        await device.set_flex_config(libedu.SamplingRate.SAMPLING_RATE_50)
        await asyncio.sleep(0.5)

        # 开始传感器数据流
        await device.start_sensor_data_stream()
        logger.info("Sensor data stream started successfully")
        return True

    except Exception as e:
        logger.error(f"Failed to connect device: {e}")
        return False


def initialize_configuration() -> None:
    """
    初始化SDK配置
    """
    logger.info("Initializing configuration...")
    libedu.set_flex_buffer_cfg(FLEX_BUFFER_LENGTH)
    libedu.set_msg_resp_callback(
        lambda device_id, msg: logger.debug(f"Message response from {device_id}: {msg}")
    )


async def main() -> None:
    """
    主函数：初始化配置，连接设备，开始数据收集循环
    """
    initialize_configuration()

    try:
        if await connect_device():
            logger.info("Device setup completed successfully")
        else:
            logger.error("Failed to setup device")
            return

        logger.info("Starting data collection loop...")

        while True:
            print_all_sensor_data()
            await asyncio.sleep(DATA_PRINT_INTERVAL)
    except KeyboardInterrupt:
        logger.info("Data collection stopped by user")
    except Exception as e:
        logger.error(f"Error in data collection loop: {e}")


if __name__ == "__main__":
    asyncio.run(main())
