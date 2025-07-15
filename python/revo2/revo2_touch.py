import asyncio
import sys
from revo2_utils import libstark, logger, open_modbus_revo2

# Main
async def main():
    (client, slave_id) = await open_modbus_revo2()
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    if not device_info.is_revo2_touch():
        logger.error("This example is only for Revo2 Touch hardware")
        sys.exit(1)

    # 启用触觉传感器功能, 开机默认是关闭的
    bits = 0x1F  # 0x1f: 5个手指上的触觉传感器都使能
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)  # Wait for touch sensor to be ready
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x1F):05b}")
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)  # Enabled之后才能获取到版本号
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    # 获取触觉传感器状态、法向力、切向力数值、通道值（电容）
    # 通道布局详见指尖触觉传感器分布图
    # 法向力：垂直于接触表面的力，可简单理解为压力
    # 切向力：沿接触面切线方向的任何力，例如惯性力、弹性力、摩擦力
    # 接近值：电容变化值
    # 触觉传感器状态，0表示正常，1表示数据异常，2表示与传感器通信异常
    while True:
        # 获取全部手指处的三维力数值、接近值、状态
        touch_status = await client.get_touch_sensor_status(slave_id)
        thumb: libstark.TouchFingerItem = touch_status[0]
        index: libstark.TouchFingerItem = touch_status[1]
        middle: libstark.TouchFingerItem = touch_status[2]
        ring: libstark.TouchFingerItem = touch_status[3]
        pinky: libstark.TouchFingerItem = touch_status[4]

        logger.debug(f"Index Sensor Desc: {index.description}")
        logger.info(f"Index Sensor status: {index.status}") # 触觉传感器状态
        logger.info(f"Index normal_force: {index.normal_force1}") # 法向力
        logger.info(f"Index tangential_force: {index.tangential_force1}") # 切向力
        logger.info(f"Index tangential_direction: {index.tangential_direction1}") # 切向力方向
        logger.info(f"Index proximity: {index.self_proximity1}") # 接近值

        # 获取单个手指处的三维力数值、接近值、状态
        thumb = await client.get_single_touch_sensor_status(slave_id, 0)
        index = await client.get_single_touch_sensor_status(slave_id, 1)
        middle = await client.get_single_touch_sensor_status(slave_id, 2)
        ring = await client.get_single_touch_sensor_status(slave_id, 3)
        pinky = await client.get_single_touch_sensor_status(slave_id, 4)

        logger.debug(f"Thumb Sensor Desc: {thumb.description}")
        logger.info(f"Thumb Sensor status: {thumb.status}")
        logger.info(f"Thumb normal_force: {thumb.normal_force1}") # 法向力
        logger.info(f"Thumb tangential_force: {thumb.tangential_force1}") # 切向力
        logger.info(f"Thumb tangential_direction: {thumb.tangential_direction1}") # 切向力方向
        logger.info(f"Thumb proximity: {thumb.self_proximity1}") # 接近值

        # 获取传感器原始通道值
        # touch_raw_data = await client.get_touch_sensor_raw_data(slave_id)
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

