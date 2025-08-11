import asyncio
import numpy as np
from model import EMGData
from edu_utils import get_armband_port_name, libedu, lib, logger, libstark
from utils import print_afe_timestamps

fs = 250  # 采样频率
num_channels = 8  # 通道数
afe_buffer_length = 1250  # 默认缓冲区长度, 1250个数据点
afe_values = np.zeros((num_channels, afe_buffer_length))  # 8通道的afe数据


def print_afe_data():
    # 获取afe数据
    fetch_num = 100  # 每次获取的数据点数, 超过缓冲区长度时，返回缓冲区中的所有数据
    clean = True  # 是否清空缓冲区
    afe_buff = libedu.get_afe_buffer(fetch_num, clean)
    logger.warning(f"Got afe buffer len={len(afe_buff)}")
    if len(afe_buff) == 0:
        return

    values = []
    for row in afe_buff:
        afe_data = EMGData.from_data(row)
        values.append(afe_data)

        channel_values = np.array_split(afe_data.channel_values, num_channels)
        # 更新每个通道的数据
        for i in range(num_channels):
            afe_values[i] = np.roll(afe_values[i], -1)  # 数据向左滚动，腾出最后一个位置
            afe_values[i, -1] = channel_values[i][0]  # 更新最新的数据值

    # 打印数据
    print_afe_timestamps(logger, values)


async def setup_device():
    # port_name = "COM8"  # 修改为实际端口号
    port_name = "/dev/tty.usbmodem212201"  # 修改为实际端口号
    # port_name = get_armband_port_name()
    # if port_name is None:
    #     return False

    # 波特率为115200
    device = libedu.PyEduDevice(port_name, 115200)

    # 启动数据流
    await device.start_data_stream(lib.MessageParser("ARMBAND-device", lib.MsgType.Edu))
    logger.info("Listening for messages...")

    # 获取Dongle与臂环的配对状态, 成功配对时，会返回Paired
    await device.get_dongle_pair_stat()
    await asyncio.sleep(0.5)

    # 设置EMG采样率，这里设置为250Hz, 0xFF表示所有通道都开启
    await device.set_afe_config(libedu.AfeSampleRate.AFE_SR_250, 0xFF)

    # 开始传感器数据流
    await device.start_sensor_data_stream()
    logger.info("Sensor data stream started")
    return True

def init_cfg():
    logger.info("Init cfg")
    libedu.edu_set_afe_buffer_cfg(afe_buffer_length)
    libedu.set_msg_resp_callback(
        lambda device_id, msg: logger.warning(f"Message response: {msg}")
    )


async def main():
    init_cfg()
    if await setup_device():
        logger.info("Device setup completed successfully")
    else:
        logger.error("Failed to setup device")
        return

    logger.info("Starting to print AFE data...")
    # 每500ms打印一次数据
    while True:
        print_afe_data()
        await asyncio.sleep(0.5)  # 500ms


if __name__ == "__main__":
    asyncio.run(main())
