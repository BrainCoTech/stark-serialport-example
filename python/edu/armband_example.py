"""
Armband EMG Data Collection Example

这个示例展示了如何使用 bc-edu-sdk 连接臂环设备并收集 EMG（肌电信号）数据，
同时也支持 IMU 和磁力计数据的收集。
"""

import asyncio
import numpy as np

from filters_sdk import *
from model import EMGData
from edu_utils import *

# 配置常量
SAMPLING_FREQUENCY = 250  # EMG采样频率 (Hz)
NUM_CHANNELS = 8  # EMG通道数
AFE_BUFFER_LENGTH = 1250  # AFE数据缓冲区长度（数据点数）
FETCH_DATA_COUNT = 100  # 每次获取的数据点数
BAUDRATE = 115200  # 串口波特率
DATA_PRINT_INTERVAL = 0.5  # 数据打印间隔（秒）

# 全局变量
afe_values = np.zeros((NUM_CHANNELS, AFE_BUFFER_LENGTH))  # EMG传感器数据缓冲区

# 滤波器
env_noise_50 = [BWBandStopFilter(4, sample_rate=SAMPLING_FREQUENCY, fl=49, fu=51) for _ in range(NUM_CHANNELS)]
env_noise_60 = [BWBandStopFilter(4, sample_rate=SAMPLING_FREQUENCY, fl=59, fu=61) for _ in range(NUM_CHANNELS)]
hp = [BWHighPassFilter(4, sample_rate=SAMPLING_FREQUENCY, f=10) for _ in range(NUM_CHANNELS)]


def update_afe_buffer(afe_data: EMGData) -> None:
    """
    更新AFE（EMG）传感器数据缓冲区

    Args:
        afe_data: EMG数据对象
    """
    # 将通道数据分割为各个通道
    channel_values = np.array_split(afe_data.channel_values, NUM_CHANNELS)

    # 更新每个通道的数据缓冲区
    for i in range(NUM_CHANNELS):
        afe_values[i] = np.roll(afe_values[i], -1)  # 数据向左滚动
        raw_value = channel_values[i][0]
        afe_values[i, -1] = raw_value  # 添加最新数据

        # filter_value = env_noise_50[i].filter(raw_value)
        # filter_value = env_noise_60[i].filter(filter_value)
        # filter_value = hp[i].filter(filter_value)
        # logger.info(f"Channel {i} raw value: {raw_value}, filtered value: {filter_value}")
        # afe_values[i, -1] = filter_value  # 添加最新数据


def print_afe_data() -> None:
    """
    获取并打印AFE（EMG）传感器数据

    支持获取以下类型的数据：
    - AFE数据（EMG肌电信号）
    - IMU原始数据或校准数据
    - 磁力计原始数据或校准数据
    """
    # 获取AFE数据
    afe_buff = libedu.get_afe_buffer(FETCH_DATA_COUNT, clean=True)

    # 可选：获取其他传感器数据
    # imu_buff = libedu.get_imu_buffer(FETCH_DATA_COUNT, clean=True)  # IMU原始数据
    # imu_calibration_buff = libedu.get_imu_calibration_buff(FETCH_DATA_COUNT, clean=True)  # IMU校准数据
    # mag_buff = libedu.get_mag_buffer(FETCH_DATA_COUNT, clean=True)  # 磁力计数据
    # mag_calibration_buff = libedu.get_mag_calibration_buff(FETCH_DATA_COUNT, clean=True)  # 磁力计校准数据

    logger.info(f"Got AFE buffer len={len(afe_buff)}")

    if len(afe_buff) == 0:
        return

    emg_data_list = []
    for row in afe_buff:
        afe_data = EMGData.from_data(row)
        emg_data_list.append(afe_data)
        update_afe_buffer(afe_data)

    # 打印数据时间戳信息
    print_afe_timestamps(logger, emg_data_list)


async def setup_armband_device() -> bool:
    """
    设置并连接臂环设备

    Returns:
        bool: 连接成功返回True，失败返回False
    """
    # 获取臂环设备端口（可以自动检测或手动指定）
    libedu.get_usb_available_ports()
    port_name = get_armband_port_name()

    # 如果自动检测失败，可以手动指定端口
    if port_name is None:
        logger.warning(f"Using manual port: {port_name}")

    if port_name is None:
        logger.error("No armband device found")
        return False

    try:
        device = libedu.PyEduDevice(port_name, BAUDRATE)

        # 启动数据流
        await device.start_data_stream(libedu.MessageParser("ARMBAND-device", libedu.MsgType.Edu))
        logger.info("Listening for messages...")

        # 获取Dongle与臂环的配对状态，成功配对时会返回Paired
        await device.get_dongle_pair_stat()
        await asyncio.sleep(0.5)

        # 配置传感器参数
        await configure_sensors(device)

        logger.info("Armband device setup completed successfully")
        return True

    except Exception as e:
        logger.error(f"Failed to setup armband device: {e}")
        return False


async def configure_sensors(device) -> None:
    """
    配置传感器采样率和数据类型

    Args:
        device: 设备对象
    """
    # 设置EMG采样率为250Hz，0xFF表示所有通道都开启
    await device.set_afe_config(libedu.AfeSampleRate.AFE_SR_250, 0xFF)

    # 配置IMU传感器 - 返回校准数据
    await device.set_imu_config(
        libedu.ImuSampleRate.IMU_SR_100,
        libedu.UploadDataType.CALIBRATED_DATA
    )

    # 配置磁力计传感器 - 返回校准数据
    await device.set_mag_config(
        libedu.MagSampleRate.MAG_SR_20,
        libedu.UploadDataType.CALIBRATED_DATA
    )

    # 可选：返回原始数据
    # await device.set_imu_config(libedu.ImuSampleRate.IMU_SR_100, libedu.UploadDataType.RAW_DATA)
    # await device.set_mag_config(libedu.MagSampleRate.MAG_SR_20, libedu.UploadDataType.RAW_DATA)

    logger.info("Sensor configuration completed")


def initialize_configuration() -> None:
    """
    初始化SDK配置
    """
    logger.info("Initializing AFE configuration...")
    libedu.set_afe_buffer_cfg(AFE_BUFFER_LENGTH)
    libedu.set_msg_resp_callback(
        lambda device_id, msg: logger.warning(f"Message response from {device_id}: {msg}")
    )


async def main() -> None:
    """
    主函数：初始化配置，连接设备，开始EMG数据收集循环
    """
    initialize_configuration()

    if await setup_armband_device():
        logger.info("Armband device setup completed successfully")
    else:
        logger.error("Failed to setup armband device")
        return

    logger.info("Starting EMG data collection loop...")
    try:
        while True:
            print_afe_data()
            await asyncio.sleep(DATA_PRINT_INTERVAL)
    except KeyboardInterrupt:
        logger.info("EMG data collection stopped by user")
    except Exception as e:
        logger.error(f"Error in data collection loop: {e}")


if __name__ == "__main__":
    asyncio.run(main())
