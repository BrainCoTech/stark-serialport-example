"""
Revo2触觉版灵巧手控制示例

本示例演示如何控制Revo2触觉版（压感式传感器）灵巧手设备，包括：
- 触觉传感器的配置和启用
- 五指和手掌压力感知数据的获取

注意事项：
- 本示例仅适用于Revo2触觉版（压感式传感器）硬件
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

    if not client.is_touch_pressure():
        logger.error("This example is only for Revo2 Touch Pressure hardware")
        libstark.modbus_close(client)
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
    # bits = 0x3F  # 0x3F: 启用五指+手掌
    # await client.touch_sensor_setup(slave_id, bits)
    # await asyncio.sleep(1)  # 等待触觉传感器准备就绪

    # 验证传感器启用状态
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x3F):06b}")

    # 获取触觉传感器固件版本（需要在启用后才能获取）
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    # await client.get_modulus_touch_data_type(slave_id, libstark.ModulusTouchDataType.Calibrated)
    data_tpye = await client.get_modulus_touch_data_type(slave_id)
    logger.info(f"Modulus Touch Data Type: {data_tpye}")


async def monitor_touch_sensors(client, slave_id):
    """
    监控触觉传感器数据

    获取触觉传感器五指+手掌合力数据
    获取触觉传感器五指+手掌全部采样点数据

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    while True:
        # 获取单个手指的触觉传感器状态（可选方式）
        thumb = await client.get_single_modulus_touch_summary(slave_id, 0)
        pinky = await client.get_single_modulus_touch_summary(slave_id, 4)
        palm = await client.get_single_modulus_touch_summary(slave_id, 5)
        logger.info(f"Thumb: {thumb}")
        logger.info(f"Pinky: {pinky}")
        logger.info(f"Palm: {palm}")
        
        # 获取所有手指的触觉传感器状态
        touch_summary: list[int] = await client.get_modulus_touch_summary(slave_id)
        logger.info(f"Touch summary: {touch_summary}")
        touch_data: list[int] = await client.get_modulus_touch_data(slave_id)
        logger.info(f"Touch Data: {touch_data}")

        await asyncio.sleep(0.05)  # 控制数据采集频率
        # break  # 示例中只执行一次，实际应用中可根据需要调整


async def perform_sensor_maintenance(client, slave_id):
    """
    执行传感器维护操作（可选）

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """

    # 校准零漂
    # 当空闲状态下的数值不为0时，可通过该指令进行校准
    # 建议忽略校准后十秒内的数据，执行时手指传感器不能受力
    # await client.touch_sensor_calibrate(slave_id, 0x3f)


if __name__ == "__main__":
    asyncio.run(main())
