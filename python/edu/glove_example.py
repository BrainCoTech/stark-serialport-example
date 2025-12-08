"""
Glove Data Collection Example

This example demonstrates how to connect the glove device and collect sensor data using bc-edu-sdk,
including Flex (bend sensor), IMU (Inertial Measurement Unit), and magnetometer data.
"""

import asyncio
import numpy as np
from typing import Optional, List

from edu_utils import *
from model import FlexData, IMUData, MagData

# Constants
NUM_CHANNELS = 6  # Flex sensor channel count
FLEX_BUFFER_LENGTH = 1250  # Flex data buffer length (data points)
FETCH_DATA_COUNT = 1000  # Number of data points to fetch
BAUDRATE = 115200  # Serial port baud rate
DATA_PRINT_INTERVAL = 0.5  # Data print interval (seconds)

# Sensor coefficients
ACC_COEFFICIENT = 0.0001220703125  # Accelerometer coefficient (1/8192)
GYRO_COEFFICIENT = 0.06103515625  # Gyroscope coefficient (1/16.4)
MAG_COEFFICIENT = 0.00152587890625  # Magnetometer coefficient (1/65536)

# Global variables
flex_seq_num: Optional[int] = None  # Flex sequence number
flex_values = np.zeros((NUM_CHANNELS, FLEX_BUFFER_LENGTH))  # Flex sensor data buffer

def print_mag_data() -> None:
    """
    Get and print magnetometer data
    """
    mag_buff = libedu.get_mag_buffer(FETCH_DATA_COUNT, clean=True)
    logger.info(f"Got mag buffer len={len(mag_buff)}")

    if len(mag_buff) == 0:
        return

    # Print the first magnetometer data as an example
    mag_data = MagData.from_data(mag_buff[0])
    logger.info(f"MagData: {mag_data}")


def print_imu_data() -> None:
    """
    Get and print IMU (Inertial Measurement Unit) data
    """
    imu_buff = libedu.get_imu_buffer(FETCH_DATA_COUNT, clean=True)
    logger.info(f"Got IMU buffer len={len(imu_buff)}")

    if len(imu_buff) == 0:
        return

    # Print the first IMU data as an example
    row = imu_buff[0]
    logger.info(f"IMU raw data: {row}")
    imu_data = IMUData.from_data(row)
    logger.info(f"IMUData: {imu_data}")


def update_flex_buffer(flex_data: FlexData) -> None:
    """
    Update Flex sensor data buffer

    Args:
        flex_data: Flex sensor data object
    """
    global flex_seq_num

    seq_num = flex_data.seq_num

    # Check data packet sequence continuity
    if flex_seq_num is not None:
        # Handle normal increment and duplicate sequence number cases
        if seq_num < flex_seq_num:
            logger.warning(f"Data sequence backward: expected >= {flex_seq_num}, got {seq_num}")
        elif seq_num > flex_seq_num + 1:
            logger.warning(f"Data sequence gap: expected {flex_seq_num + 1}, got {seq_num}")
        elif seq_num == flex_seq_num:
            logger.debug(f"Duplicate sequence number: {seq_num}")
            return  # Skip duplicate data packets

    flex_seq_num = seq_num

    # Split channel data into individual channels
    channel_values = np.array_split(flex_data.channel_values, NUM_CHANNELS)

    # Update each channel's data buffer
    for i in range(NUM_CHANNELS):
        flex_values[i] = np.roll(flex_values[i], -1)  # Data shift left
        flex_values[i, -1] = channel_values[i][0]  # Add latest data


def print_flex_data() -> None:
    """
    Get and print Flex sensor data
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
    Print Flex data timestamps

    Args:
        data: Flex data list
    """
    if len(data) <= 6:
        for item in data:
            logger.info(f"{item}")
        return

    # If there are too many data points, only print the first 3 and last 3
    for item in data[:3]:
        logger.info(f"{item}")
    logger.info("...")
    for item in data[-3:]:
        logger.info(f"{item}")


def print_all_sensor_data() -> None:
    """
    Print all sensor data (IMU, magnetometer, Flex)
    """
    print_imu_data()
    print_mag_data()
    print_flex_data()


async def connect_device() -> bool:
    """
    Connect to the glove device and start data stream

    Returns:
        bool: True if connection is successful, False otherwise
    """
    port_name = get_glove_port_name()
    if port_name is None:
        logger.error("No glove device found")
        return False

    try:
        device = libedu.PyEduDevice(port_name, BAUDRATE)
        await device.start_data_stream(libedu.MessageParser("Glove-device", libedu.MsgType.Edu))
        logger.info("Listening for messages...")

        # Get device status information
        await device.get_dongle_pair_stat()
        await asyncio.sleep(0.5)

        # Optional: Configure sensor sampling rate
        await device.set_flex_config(libedu.SamplingRate.SAMPLING_RATE_50)
        await asyncio.sleep(0.5)

        # Start sensor data stream
        await device.start_sensor_data_stream()
        logger.info("Sensor data stream started successfully")
        return True

    except Exception as e:
        logger.error(f"Failed to connect device: {e}")
        return False


def initialize_configuration() -> None:
    """
    Initialize SDK configuration
    """
    logger.info("Initializing configuration...")
    libedu.set_flex_buffer_cfg(FLEX_BUFFER_LENGTH)
    libedu.set_msg_resp_callback(
        lambda device_id, msg: logger.debug(f"Message response from {device_id}: {msg}")
    )


async def main() -> None:
    """
    Main function: initialize configuration, connect to device, and start data collection loop
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
