"""
Revo2灵巧手多设备控制示例

本示例演示如何同时控制多个Revo2灵巧手设备，包括：
- 多设备连接方式（单串口多设备 vs 多串口多设备）
- 触觉传感器配置和数据获取（触觉版本）
- 电机状态监控和自动控制
- 手指位置和电流控制
- 并发任务管理和异常处理

连接方式说明：
- 方式1：单个串口连接多个设备（需要配置不同的设备ID）
- 方式2：多个串口分别连接不同设备（可以使用相同的设备ID）

Revo2设备ID说明：
- 左手默认ID：0x7e（126）
- 右手默认ID：0x7f（127）
- 可通过配置修改设备ID

适用场景：
- 双手控制系统
- 多设备协同作业
- 设备性能对比测试
- 生产线自动化控制
"""

import asyncio
import sys
import time
from revo2_utils import *
from utils import setup_shutdown_event

# 多设备控制配置
# 方式1：单个串口连接多个设备（需要配置不同的设备ID）
# 方式2：多个串口分别连接不同设备（可以使用相同的设备ID）
# 例如：
# client_left_hand = await libstark.modbus_open(port_name_left, baudrate)
# client_right_hand = await libstark.modbus_open(port_name_right, baudrate)

# 设备ID列表配置
# slave_ids = [0x7e]        # 单个串口连接一个设备（左手）
slave_ids = [0x7e, 0x7f]   # 单个串口连接两个设备，左手ID为0x7e，右手ID为0x7f

# 设备触觉版本标记字典
# 用于记录每个设备是否为触觉版本，以便后续处理时区分
slave_is_touch = {0x7e: False, 0x7f: False}


async def main():
    """
    主函数：初始化设备连接和控制任务
    """
    # 设置关闭事件监听
    shutdown_event = setup_shutdown_event(logger)

    # 打开Modbus连接
    (client, detected_slave_id) = await open_modbus_revo2()

    # 为每个设备配置触觉传感器并启动状态监控任务
    for slave_id in slave_ids:
        await setup_touch_sensor(client, slave_id)
        asyncio.create_task(get_motor_status_periodically(client, slave_id))
    logger.info("Status monitoring tasks started")

    # 多个串口分别连接不同设备的示例（可选）
    # 适用于需要完全独立控制两个设备的场景
    # client_left_hand = await libstark.modbus_open("/dev/ttyUSB0", libstark.Baudrate.Baud460800)
    # client_right_hand = await libstark.modbus_open("/dev/ttyUSB1", libstark.Baudrate.Baud460800)
    # left_slave_id = 0x7e
    # right_slave_id = 0x7f
    # await setup_touch_sensor(client_left_hand, left_slave_id)
    # await setup_touch_sensor(client_right_hand, right_slave_id)
    # asyncio.create_task(get_motor_status_periodically(client_left_hand, left_slave_id))
    # asyncio.create_task(get_motor_status_periodically(client_right_hand, right_slave_id))

    # 等待关闭事件
    await shutdown_event.wait()

    # 清理资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def setup_touch_sensor(client, slave_id):
    """
    配置触觉传感器

    根据设备类型（触觉版/基础版）进行相应的配置。
    触觉版设备会启用触觉传感器功能，基础版设备会跳过此步骤。

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 验证设备类型
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device {slave_id:02x} info: {device_info.description}")
    is_revo2_touch = device_info.is_revo2_touch()
    slave_is_touch[slave_id] = is_revo2_touch

    if not is_revo2_touch:
        logger.warning(f"Device {slave_id:02x} is not Revo2 Touch hardware, skipping touch sensor setup")
        return

    # 启用触觉传感器功能（开机默认关闭）
    bits = 0x1F  # 0x1f: 启用5个手指上的所有触觉传感器
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)  # 等待触觉传感器准备就绪

    # 验证传感器启用状态
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Device {slave_id:02x} Touch Sensor Enabled: {(bits & 0x1F):05b}")

    # 获取触觉传感器固件版本（需要在setup之后才能获取）
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Device {slave_id:02x} Touch Fw Versions: {touch_fw_versions}")


async def get_touch_status(client, slave_id):
    """
    获取触觉传感器状态和数据

    触觉传感器数据说明：
    - 法向力：垂直于接触表面的力（压力）
    - 切向力：沿接触面切线方向的力（摩擦力、惯性力等）
    - 接近值：电容变化值，反映物体接近程度
    - 传感器状态：0=正常，1=数据异常，2=通信异常

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 获取所有手指的触觉传感器状态
    touch_status: list[libstark.TouchFingerItem] = await client.get_touch_sensor_status(slave_id)
    thumb: libstark.TouchFingerItem = touch_status[0]   # 拇指
    index: libstark.TouchFingerItem = touch_status[1]   # 食指
    middle: libstark.TouchFingerItem = touch_status[2]  # 中指
    ring: libstark.TouchFingerItem = touch_status[3]    # 无名指
    pinky: libstark.TouchFingerItem = touch_status[4]   # 小指

    # 记录食指传感器详细数据
    logger.debug(f"Device {slave_id:02x} Index Sensor Desc: {index.description}")
    logger.info(f"Device {slave_id:02x} Index Sensor status: {index.status}")                           # 触觉传感器状态
    logger.info(f"Device {slave_id:02x} Index normal_force: {index.normal_force1}")                     # 法向力
    logger.info(f"Device {slave_id:02x} Index tangential_force: {index.tangential_force1}")             # 切向力
    logger.info(f"Device {slave_id:02x} Index tangential_direction: {index.tangential_direction1}")     # 切向力方向
    logger.info(f"Device {slave_id:02x} Index proximity: {index.self_proximity1}")                      # 接近值

    # 获取单个手指的触觉传感器状态（另一种获取方式）
    thumb = await client.get_single_touch_sensor_status(slave_id, 0)
    index = await client.get_single_touch_sensor_status(slave_id, 1)
    middle = await client.get_single_touch_sensor_status(slave_id, 2)
    ring = await client.get_single_touch_sensor_status(slave_id, 3)
    pinky = await client.get_single_touch_sensor_status(slave_id, 4)

    # 记录拇指传感器详细数据
    logger.debug(f"Device {slave_id:02x} Thumb Sensor Desc: {thumb.description}")
    logger.info(f"Device {slave_id:02x} Thumb Sensor status: {thumb.status}")                           # 触觉传感器状态
    logger.info(f"Device {slave_id:02x} Thumb normal_force: {thumb.normal_force1}")                     # 法向力
    logger.info(f"Device {slave_id:02x} Thumb tangential_force: {thumb.tangential_force1}")             # 切向力
    logger.info(f"Device {slave_id:02x} Thumb tangential_direction: {thumb.tangential_direction1}")     # 切向力方向
    logger.info(f"Device {slave_id:02x} Thumb proximity: {thumb.self_proximity1}")                      # 接近值

    # 获取传感器原始通道值（可选，用于调试和高级分析）
    # touch_raw_data = await client.get_touch_sensor_raw_data(slave_id)
    # logger.info(f"Device {slave_id:02x} Touch Sensor Raw Data: {touch_raw_data.description}")
    # logger.debug(f"Device {slave_id:02x} Touch Sensor Raw Data Thumb: {touch_raw_data.thumb}")
    # logger.debug(f"Device {slave_id:02x} Touch Sensor Raw Data Index: {touch_raw_data.index}")
    # logger.debug(f"Device {slave_id:02x} Touch Sensor Raw Data Middle: {touch_raw_data.middle}")
    # logger.debug(f"Device {slave_id:02x} Touch Sensor Raw Data Ring: {touch_raw_data.ring}")
    # logger.debug(f"Device {slave_id:02x} Touch Sensor Raw Data Pinky: {touch_raw_data.pinky}")


async def get_motor_status_periodically(client, slave_id):
    """
    定期获取电机状态并执行自动控制

    该函数会持续监控电机状态，并根据手指位置执行简单的开合动作：
    - 当手指张开时，执行握手动作
    - 当手指闭合时，执行张开动作
    - 对于触觉版设备，同时获取触觉传感器数据

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    logger.info(f"Motor status monitoring started for device {slave_id:02x}")
    index = 0

    while True:
        try:
            # 检查是否为触觉版本，如果是则获取触觉传感器状态
            if slave_is_touch.get(slave_id, False):
                logger.debug(f"Getting touch sensor status for device {slave_id:02x}...")
                await get_touch_status(client, slave_id)

            # 获取电机状态
            logger.debug(f"Getting motor status for device {slave_id:02x}...")
            start = time.perf_counter()
            status: libstark.MotorStatusData = await client.get_motor_status(slave_id)
            cost_ms = (time.perf_counter() - start) * 1000

            # 记录状态信息
            logger.info(
                f"Device {slave_id:02x} [{index}] Motor status - cost: {cost_ms:.2f}ms, "
                f"is_idle: {status.is_idle()}"
            )
            logger.info(f"Device {slave_id:02x} [{index}] Motor status: {status.description}")
            index += 1

            # 根据当前状态执行自动控制
            if status.is_idle():
                if is_positions_open(status):
                    # 手指张开时，执行握手动作
                    # 位置值：[拇指, 食指, 中指, 无名指, 小指, 手腕]
                    await client.set_finger_positions(slave_id, [500, 500, 1000, 1000, 1000, 1000])
                    logger.debug(f"Device {slave_id:02x} executing grip action")
                elif is_positions_closed(status):
                    # 手指闭合时，执行张开动作
                    await client.set_finger_positions(slave_id, [400, 400, 0, 0, 0, 0])
                    logger.debug(f"Device {slave_id:02x} executing open action")

            # 添加延时避免过于频繁的查询
            await asyncio.sleep(0.001)

        except Exception as e:
            logger.error(f"Error in motor status monitoring for device {slave_id:02x}: {e}")
            await asyncio.sleep(1)  # 发生错误时等待更长时间再重试


if __name__ == "__main__":
    asyncio.run(main())
