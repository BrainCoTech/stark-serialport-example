import asyncio
import sys
from stark_utils import get_stark_port_name, libstark, logger

# Main
async def main():
    libstark.init_config(libstark.StarkFirmwareType.V1Touch)
    port_name = get_stark_port_name()
    if port_name is None:
        return
    slave_id = 1 # 默认设备ID
    # baudrate = libstark.Baudrate.Baud460800
    baudrate = libstark.Baudrate.Baud115200 # 默认波特率
    client = await libstark.modbus_open(port_name, baudrate)

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device Firmware: {device_info.firmware_version}")  # 固件版本
    logger.info(f"Device info: {device_info.description}")

    # 启用触觉传感器功能, 开机默认是关闭的
    bits = 0x1F  # 0x1f: 5个手指上的触觉传感器都使能
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)  # Wait for touch sensor to be ready
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x1F):05b}")
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)  # setup之后才能获取到版本号
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    # 获取触觉传感器状态、法向力、切向力数值、通道值（电容）
    # 通道布局详见指尖触觉传感器分布图
    # 法向力：垂直于接触表面的力，可简单理解为压力
    # 切向力：沿接触面切线方向的任何力，例如惯性力、弹性力、摩擦力
    # 自接近：自电容变化值
    # 互接近：互电容变化值
    # 触觉传感器状态，0表示正常，1表示数据异常，2表示与传感器通信异常
    while True:
        touch_status = await client.get_touch_sensor_status(slave_id) # 获取全部手指处的触觉传感器三维力数值
        thumb = touch_status[0]
        index = touch_status[1]
        # middle = touch_status[2]
        # ring = touch_status[3]
        # pinky = touch_status[4]
        logger.debug(f"Index Sensor Desc: {index.description}")
        logger.info(f"Index Sensor status: {index.status}") # 触觉传感器状态
        logger.info(f"Index normal_force1: {index.normal_force1}") # 法向力1
        logger.info(f"Index normal_force2: {index.normal_force2}") # 法向力2
        logger.info(f"Index normal_force3: {index.normal_force3}") # 法向力3
        logger.info(f"Index tangential_force1: {index.tangential_force1}") # 切向力1
        logger.info(f"Index tangential_force2: {index.tangential_force2}") # 切向力2
        logger.info(f"Index tangential_force3: {index.tangential_force3}") # 切向力3
        logger.info(f"Index tangential_direction1: {index.tangential_direction1}") # 切向力方向1
        logger.info(f"Index tangential_direction2: {index.tangential_direction2}") # 切向力方向2
        logger.info(f"Index tangential_direction3: {index.tangential_direction3}") # 切向力方向3
        logger.info(f"Index self_proximity1: {index.self_proximity1}") # 自接近1
        logger.info(f"Index self_proximity2: {index.self_proximity2}") # 自接近2
        logger.info(f"Index mutual_proximity: {index.mutual_proximity}") # 互接近

        thumb = await client.get_single_touch_sensor_status(slave_id, 0)  # 0~4 获取单个手指处的触觉传感器三维力数值
        # index = await client.get_single_touch_sensor_status(slave_id, 1)  # 0~4 获取单个手指处的触觉传感器三维力数值
        # middle = await client.get_single_touch_sensor_status(slave_id, 2)  # 0~4 获取单个手指处的触觉传感器三维力数值
        # ring = await client.get_single_touch_sensor_status(slave_id, 3)  # 0~4 获取单个手指处的触觉传感器三维力数值
        # pinky = await client.get_single_touch_sensor_status(slave_id, 4)  # 0~4 获取单个手指处的触觉传感器三维力数值

        logger.debug(f"Thumb Sensor Desc: {thumb.description}")
        logger.info(f"Thumb Sensor status: {thumb.status}")
        logger.info(f"Thumb normal_force1: {thumb.normal_force1}") # 法向力1
        logger.info(f"Thumb normal_force2: {thumb.normal_force2}") # 法向力2
        logger.info(f"Thumb tangential_force1: {thumb.tangential_force1}") # 切向力1
        logger.info(f"Thumb tangential_force2: {thumb.tangential_force2}") # 切向力2
        logger.info(f"Thumb tangential_direction1: {thumb.tangential_direction1}") # 切向力方向1
        logger.info(f"Thumb tangential_direction2: {thumb.tangential_direction2}") # 切向力方向2
        logger.info(f"Thumb self_proximity1: {thumb.self_proximity1}") # 自接近1

        # touch_raw_data = await client.get_touch_sensor_raw_data(slave_id) # 获取触觉传感器原始通道值
        # logger.info(f"Touch Sensor Raw Data: {touch_raw_data.description}")
        # logger.debug(f"Touch Sensor Raw Data Thumb: {touch_raw_data.thumb}")
        # logger.debug(f"Touch Sensor Raw Data Index: {touch_raw_data.index}")
        # logger.debug(f"Touch Sensor Raw Data Middle: {touch_raw_data.middle}")
        # logger.debug(f"Touch Sensor Raw Data Ring: {touch_raw_data.ring}")
        # logger.debug(f"Touch Sensor Raw Data Pinky: {touch_raw_data.pinky}")
        await asyncio.sleep(0.05)
        break

    # 触觉传感器复位，发送传感器采集通道复位指令。在执行该指令时，手指传感器尽量不要受力。
    # await client.touch_sensor_reset(slave_id, 0x1f)  # 复位指定手指的传感器采集通道

    # 触觉传感器参数校准，参数校准指令。 当空闲状态下的三维力数值不为0时，可通过该寄存器进行校准。该指令的执行时间较长，期间的数据无法作为参考。建议忽略在该参数校准寄存器设置后十秒内的数据。在执行该指令时，手指传感器不能受力，否则将导致校准后的传感器数据异常。
    # await client.touch_sensor_calibrate(slave_id, 0x1f)  # 校准指定手指的传感器采集通道

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
