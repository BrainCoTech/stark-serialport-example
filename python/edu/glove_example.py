import asyncio
import numpy as np

from edu_utils import *
from model import FlexData, IMUData, MagData

# flex数据
num_channels = 6  # 通道数
flex_buffer_length = 1250  # 默认缓冲区长度, 1250个数据点
flex_seq_num = None  # flex数据包序号
flex_values = np.zeros((num_channels, flex_buffer_length))  # 8通道的flex数据

acc_coefficient = 0.0001220703125 # 加速度计系数, 1/8192
gyro_coefficient = 0.06103515625 # 陀螺仪系数, 1/16.4
mag_coefficient = 0.00152587890625 # 磁力计系数, 1/65536

def print_mag_data():
    # 获取mag数据
    fetch_num = 1000  # 每次获取的数据点数, 超过缓冲区长度时，返回缓冲区中的所有数据
    clean = True  # 是否清空缓冲区
    mag_buff = libedu.get_mag_buffer(fetch_num, clean)
    logger.warning(f"Got mag buffer len={len(mag_buff)}")
    if len(mag_buff) == 0:
        return

    for row in mag_buff:
        mag_data = MagData.from_data(row)
        logger.info(f"MagData: {mag_data}")
        break  # 只打印第一条Mag数据


def print_imu_data():
    # 获取IMU数据
    fetch_num = 1000  # 每次获取的数据点数, 超过缓冲区长度时，返回缓冲区中的所有数据
    clean = True  # 是否清空缓冲区
    imu_buff = libedu.get_imu_buffer(fetch_num, clean)
    logger.warning(f"Got IMU buffer len={len(imu_buff)}")
    if len(imu_buff) == 0:
        return

    for row in imu_buff:
        logger.info(f"IMU data: {row}")
        imu_data = IMUData.from_data(row)
        logger.info(f"IMUData: {imu_data}")
        break  # 只打印第一条IMU数据


def print_data():
    print_imu_data()
    print_mag_data()

    # 获取flex数据
    fetch_num = 1000  # 每次获取的数据点数, 超过缓冲区长度时，返回缓冲区中的所有数据
    clean = True  # 是否清空缓冲区
    flex_buff = libedu.get_flex_buffer(fetch_num, clean)
    logger.warning(f"Got flex buffer len={len(flex_buff)}")
    if len(flex_buff) == 0:
        return

    flex_data_arr = []
    for row in flex_buff:
        flex_data = FlexData.from_data(row)
        flex_data_arr.append(flex_data)

        # 检查数据包序号
        seq_num = flex_data.seq_num
        global flex_seq_num
        # logger.info(f"timestamp={timestamp}")
        if flex_seq_num is not None and seq_num != flex_seq_num + 1:
            logger.warning(f"flex_seq_num={flex_seq_num}, seq_num={seq_num}")
        if flex_seq_num is not None:
            flex_seq_num = seq_num

        # print(f"channel_values: {len(flex_data.channel_values)}")
        channel_values = np.array_split(flex_data.channel_values, num_channels)

        # 更新每个通道的数据
        for i in range(num_channels):
            flex_values[i] = np.roll(flex_values[i], -1)  # 数据向左滚动，腾出最后一个位置
            flex_values[i, -1] = channel_values[i][0]  # 更新最新的数据值

    # 打印数据
    print_flex_timestamps(flex_data_arr)

def print_flex_timestamps(data):
    if len(data) <= 6:
        for item in data:
            logger.info(f"{item}")
        return
    for item in data[:3]:
        logger.info(f"{item}")
    logger.info("...")
    for item in data[-3:]:
        logger.info(f"{item}")


async def connect():
    port_name = get_glove_port_name()
    if port_name is None:
        return False

    baudrate = 115200
    device = libedu.PyEduDevice(port_name, baudrate)

    await device.start_data_stream(lib.MessageParser("Glove-device", libstark.MsgType.Edu))
    logger.info("Listening for messages...")

    # 获取
    # await device.get_dongle_info()
    # await device.get_dongle_pair_cfg()
    await device.get_dongle_pair_stat()
    await asyncio.sleep(0.5)

    # await device.get_device_info()
    # await device.get_sensor_cfg()

    # await device.set_flex_config(libedu.SamplingRate.SAMPLING_RATE_50)
    # await asyncio.sleep(0.5)

    # 开始传感器数据流
    await device.start_sensor_data_stream()
    return True


def init_cfg():
    logger.info("Init cfg")
    libedu.edu_set_flex_buffer_cfg(flex_buffer_length)
    libedu.set_msg_resp_callback(
        lambda device_id, msg: logger.warning(f"Message response: {msg}")
    )


async def main():
    init_cfg()
    if await connect():
        logger.info("Device setup completed successfully")
    else:
        logger.error("Failed to setup device")
        return

    logger.info("Starting to print data...")
    while True:
        print_data()
        # print_imu_data()
        await asyncio.sleep(0.5)  # 500ms


if __name__ == "__main__":
    asyncio.run(main())
