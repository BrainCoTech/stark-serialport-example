"""
Revo2触觉版灵巧手控制示例

本示例演示如何控制Revo2触觉版（电容式传感器）灵巧手设备，包括：
- 触觉传感器的配置和启用
- 三维力数据的获取和处理（法向力、切向力）
- 接近值（电容变化）的监控
- 单个手指传感器数据的读取
- 传感器状态监控和异常处理
- 传感器校准和维护操作

注意事项：
- 本示例仅适用于Revo2触觉版（电容式传感器）硬件
- 触觉传感器开机默认开启
- 执行零漂校准时传感器不能受力
"""

import asyncio
import sys
from revo2_utils import *


async def main():
    """
    主函数：初始化触觉版灵巧手并执行触觉传感器控制
    """
    # 连接Revo2设备
    (client, slave_id) = await open_modbus_revo2()

    # 验证设备类型为触觉版
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    if not device_info.is_revo2_touch():
        logger.error("This example is only for Revo2 Touch hardware")
        sys.exit(1)

    # 配置触觉传感器
    await setup_touch_sensors(client, slave_id)

    # 监控触觉传感器数据
    await monitor_touch_sensors(client, slave_id)

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def setup_touch_sensors(client, slave_id):
    """
    配置和启用触觉传感器

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 启用触觉传感器功能（开机默认关闭）
    bits = 0x1F  # 0x1F: 启用5个手指上的所有触觉传感器
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)  # 等待触觉传感器准备就绪

    # 验证传感器启用状态
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x1F):05b}")

    # 获取触觉传感器固件版本（需要在启用后才能获取）
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")


async def monitor_touch_sensors(client, slave_id):
    """
    监控触觉传感器数据

    获取触觉传感器状态、法向力、切向力数值、通道值（电容）

    传感器数据说明：
    - 法向力：垂直于接触表面的力，可简单理解为压力
    - 切向力：沿接触面切线方向的任何力，例如惯性力、弹性力、摩擦力
    - 接近值：电容变化值
    - 传感器状态：0=正常，1=数据异常，2=与传感器通信异常

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    while True:
        # 获取所有手指的触觉传感器状态
        touch_status: list[libstark.TouchFingerItem] = await client.get_touch_sensor_status(slave_id)
        thumb: libstark.TouchFingerItem = touch_status[0]   # 拇指
        index: libstark.TouchFingerItem = touch_status[1]   # 食指
        middle: libstark.TouchFingerItem = touch_status[2]  # 中指
        ring: libstark.TouchFingerItem = touch_status[3]    # 无名指
        pinky: libstark.TouchFingerItem = touch_status[4]   # 小指

        # 记录食指传感器详细数据
        logger.debug(f"Index Sensor Desc: {index.description}")
        logger.info(f"Index Sensor status: {index.status}")                           # 触觉传感器状态
        logger.info(f"Index normal_force: {index.normal_force1}")                     # 法向力
        logger.info(f"Index tangential_force: {index.tangential_force1}")             # 切向力
        logger.info(f"Index tangential_direction: {index.tangential_direction1}")     # 切向力方向
        logger.info(f"Index proximity: {index.self_proximity1}")                      # 接近值

        # 获取单个手指的触觉传感器状态（可选方式）
        thumb = await client.get_single_touch_sensor_status(slave_id, 0)
        index = await client.get_single_touch_sensor_status(slave_id, 1)
        middle = await client.get_single_touch_sensor_status(slave_id, 2)
        ring = await client.get_single_touch_sensor_status(slave_id, 3)
        pinky = await client.get_single_touch_sensor_status(slave_id, 4)

        # 记录拇指传感器详细数据
        logger.debug(f"Thumb Sensor Desc: {thumb.description}")
        logger.info(f"Thumb Sensor status: {thumb.status}")
        logger.info(f"Thumb normal_force: {thumb.normal_force1}")                     # 法向力
        logger.info(f"Thumb tangential_force: {thumb.tangential_force1}")             # 切向力
        logger.info(f"Thumb tangential_direction: {thumb.tangential_direction1}")     # 切向力方向
        logger.info(f"Thumb proximity: {thumb.self_proximity1}")                      # 接近值

        # 获取传感器原始通道值（可选，用于调试）
        # touch_raw_data = await client.get_touch_sensor_raw_data(slave_id)
        # logger.info(f"Touch Sensor Raw Data: {touch_raw_data.description}")
        # logger.debug(f"Touch Sensor Raw Data Thumb: {touch_raw_data.thumb}")
        # logger.debug(f"Touch Sensor Raw Data Index: {touch_raw_data.index}")
        # logger.debug(f"Touch Sensor Raw Data Middle: {touch_raw_data.middle}")
        # logger.debug(f"Touch Sensor Raw Data Ring: {touch_raw_data.ring}")
        # logger.debug(f"Touch Sensor Raw Data Pinky: {touch_raw_data.pinky}")

        await asyncio.sleep(0.05)  # 控制数据采集频率
        break  # 示例中只执行一次，实际应用中可根据需要调整


async def perform_sensor_maintenance(client, slave_id):
    """
    执行传感器维护操作（可选）

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 触觉传感器复位
    # 发送传感器采集通道复位指令，执行时手指传感器尽量不要受力
    # await client.touch_sensor_reset(slave_id, 0x1f)  # 复位指定手指的传感器采集通道

    # 触觉传感器参数校准
    # 当空闲状态下的三维力数值不为0时，可通过该指令进行校准
    # 该指令的执行时间较长，期间的数据无法作为参考
    # 建议忽略校准后十秒内的数据，执行时手指传感器不能受力
    # await client.touch_sensor_calibrate(slave_id, 0x1f)  # 校准指定手指的传感器采集通道


if __name__ == "__main__":
    asyncio.run(main())
