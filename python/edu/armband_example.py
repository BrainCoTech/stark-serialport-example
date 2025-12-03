"""
Armband EMG Data Collection Example

This example demonstrates how to use `bc-edu-sdk` to connect to an armband
device and collect EMG (electromyography) data.
It also supports collecting IMU and magnetometer data.
"""

import asyncio
import numpy as np

from filters_sdk import *
from model import EMGData
from edu_utils import *

# Configuration constants
SAMPLING_FREQUENCY = 250  # EMG sampling frequency (Hz)
NUM_CHANNELS = 8  # Number of EMG channels
AFE_BUFFER_LENGTH = 1250  # AFE data buffer length (number of data points)
FETCH_DATA_COUNT = 100  # Number of data points fetched each time
BAUDRATE = 115200  # Serial port baud rate
DATA_PRINT_INTERVAL = 0.5  # Data print interval (seconds)

# Global variables
afe_values = np.zeros((NUM_CHANNELS, AFE_BUFFER_LENGTH))  # EMG sensor data buffer

# Filters
env_noise_50 = [BWBandStopFilter(4, sample_rate=SAMPLING_FREQUENCY, fl=49, fu=51) for _ in range(NUM_CHANNELS)]
env_noise_60 = [BWBandStopFilter(4, sample_rate=SAMPLING_FREQUENCY, fl=59, fu=61) for _ in range(NUM_CHANNELS)]
hp = [BWHighPassFilter(4, sample_rate=SAMPLING_FREQUENCY, f=10) for _ in range(NUM_CHANNELS)]


def update_afe_buffer(afe_data: EMGData) -> None:
    """
    Update the AFE (EMG) sensor data buffer.

    Args:
        afe_data: EMG data object
    """
    # Split channel data into individual channels
    channel_values = np.array_split(afe_data.channel_values, NUM_CHANNELS)

    # Update the buffer of each channel
    for i in range(NUM_CHANNELS):
        afe_values[i] = np.roll(afe_values[i], -1)  # Shift data to the left
        raw_value = channel_values[i][0]
        afe_values[i, -1] = raw_value  # Append the latest data

        # filter_value = env_noise_50[i].filter(raw_value)
        # filter_value = env_noise_60[i].filter(filter_value)
        # filter_value = hp[i].filter(filter_value)
        # logger.info(f"Channel {i} raw value: {raw_value}, filtered value: {filter_value}")
        # afe_values[i, -1] = filter_value  # Append the latest filtered data


def print_afe_data() -> None:
    """
    Fetch and print AFE (EMG) sensor data.

    Supports retrieving the following types of data:
    - AFE data (EMG signals)
    - IMU raw or calibrated data
    - Magnetometer raw or calibrated data
    """
    # Fetch AFE data
    afe_buff = libedu.get_afe_buffer(FETCH_DATA_COUNT, clean=True)

    # Optional: fetch other sensor data
    # imu_buff = libedu.get_imu_buffer(FETCH_DATA_COUNT, clean=True)  # IMU raw data
    # imu_calibration_buff = libedu.get_imu_calibration_buff(FETCH_DATA_COUNT, clean=True)  # IMU calibrated data
    # mag_buff = libedu.get_mag_buffer(FETCH_DATA_COUNT, clean=True)  # Magnetometer data
    # mag_calibration_buff = libedu.get_mag_calibration_buff(FETCH_DATA_COUNT, clean=True)  # Magnetometer calibrated data

    logger.info(f"Got AFE buffer len={len(afe_buff)}")

    if len(afe_buff) == 0:
        return

    emg_data_list = []
    for row in afe_buff:
        afe_data = EMGData.from_data(row)
        emg_data_list.append(afe_data)
        update_afe_buffer(afe_data)

    # Print timestamp information of AFE data
    print_afe_timestamps(logger, emg_data_list)


async def setup_armband_device() -> bool:
    """
    Set up and connect to the armband device.

    Returns:
        bool: True if connection succeeded, False otherwise.
    """
    # Get armband device port (auto-detect or manual)
    libedu.get_usb_available_ports()
    port_name = get_armband_port_name()

    # If auto-detection fails, you can manually specify the port
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
    Configure sensor sampling rates and data types.

    Args:
        device: Device object
    """
    # Set EMG sample rate to 250 Hz, 0xFF means all channels are enabled
    await device.set_afe_config(libedu.AfeSampleRate.AFE_SR_250, 0xFF)

    # Configure IMU sensor - return calibrated data
    await device.set_imu_config(
        libedu.ImuSampleRate.IMU_SR_100,
        libedu.UploadDataType.CALIBRATED_DATA
    )

    # Configure magnetometer sensor - return calibrated data
    await device.set_mag_config(
        libedu.MagSampleRate.MAG_SR_20,
        libedu.UploadDataType.CALIBRATED_DATA
    )

    # Optional: return raw data instead of calibrated data
    # await device.set_imu_config(libedu.ImuSampleRate.IMU_SR_100, libedu.UploadDataType.RAW_DATA)
    # await device.set_mag_config(libedu.MagSampleRate.MAG_SR_20, libedu.UploadDataType.RAW_DATA)

    logger.info("Sensor configuration completed")


def initialize_configuration() -> None:
    """
    Initialize SDK configuration.
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
