"""
Revo1触觉版灵巧手控制示例

本示例演示如何控制Revo1触觉版灵巧手设备，包括：
- 触觉传感器的配置和启用
- 触觉传感器数据的获取和处理
- 电机状态监控和自动控制
- 手指位置控制和电流控制
"""

import asyncio
import sys
import time
from revo1_utils import *
from utils import setup_shutdown_event

async def main():
    """
    主函数：初始化触觉版灵巧手连接和控制任务
    """
    # 设置关闭事件监听
    shutdown_event = setup_shutdown_event(logger)

    # 检测灵巧手的波特率和设备ID，初始化client
    (client, slave_id) = await open_modbus_revo1()

    # 验证设备类型为触觉版
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")
    if not device_info.is_revo1_touch():
        logger.error("This example is only for Revo1 Touch hardware")
        sys.exit(1)

    # 配置触觉传感器
    await setup_touch_sensor(client, slave_id)

    # 启动电机状态监控任务
    asyncio.create_task(get_motor_status_periodically(client, slave_id))
    logger.info("Status task started")

    # 等待关闭事件
    await shutdown_event.wait()

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)

async def setup_touch_sensor(client, slave_id):
    """
    配置和启用触觉传感器

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 启用触觉传感器功能（开机默认关闭）
    bits = 0x1F  # 0x1f: 启用5个手指上的所有触觉传感器
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)  # 等待触觉传感器准备就绪

    # 验证传感器启用状态
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x1F):05b}")

    # 获取触觉传感器固件版本（需要在setup之后才能获取）
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    # 触觉传感器维护命令（可选）
    # 复位：发送传感器采集通道复位指令，执行时手指传感器尽量不要受力
    # await client.touch_sensor_reset(slave_id, 0x1f)

    # 校准：当空闲状态下的三维力数值不为0时使用，执行时间较长
    # 建议忽略校准后十秒内的数据，执行时手指传感器不能受力
    # await client.touch_sensor_calibrate(slave_id, 0x1f)

async def get_touch_status(client, slave_id):
    """
    获取触觉传感器状态和数据

    触觉传感器数据说明：
    - 法向力：垂直于接触表面的力（压力）
    - 切向力：沿接触面切线方向的力（摩擦力、惯性力等）
    - 自接近：自电容变化值
    - 互接近：互电容变化值
    - 传感器状态：0=正常，1=数据异常，2=通信异常

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 获取所有手指的触觉传感器状态
    touch_status: list[libstark.TouchFingerItem] = await client.get_touch_sensor_status(slave_id)
    thumb = touch_status[0]   # 拇指
    index = touch_status[1]   # 食指
    middle = touch_status[2]  # 中指
    ring = touch_status[3]    # 无名指
    pinky = touch_status[4]   # 小指

    # 记录食指传感器详细数据
    logger.debug(f"Index Sensor Desc: {index.description}")
    logger.info(f"Index Sensor status: {index.status}")  # 触觉传感器状态
    logger.info(f"Index normal_force1: {index.normal_force1}")  # 法向力1
    logger.info(f"Index normal_force2: {index.normal_force2}")  # 法向力2
    logger.info(f"Index normal_force3: {index.normal_force3}")  # 法向力3
    logger.info(f"Index tangential_force1: {index.tangential_force1}")  # 切向力1
    logger.info(f"Index tangential_force2: {index.tangential_force2}")  # 切向力2
    logger.info(f"Index tangential_force3: {index.tangential_force3}")  # 切向力3
    logger.info(f"Index tangential_direction1: {index.tangential_direction1}")  # 切向力方向1
    logger.info(f"Index tangential_direction2: {index.tangential_direction2}")  # 切向力方向2
    logger.info(f"Index tangential_direction3: {index.tangential_direction3}")  # 切向力方向3
    logger.info(f"Index self_proximity1: {index.self_proximity1}")  # 自接近1
    logger.info(f"Index self_proximity2: {index.self_proximity2}")  # 自接近2
    logger.info(f"Index mutual_proximity: {index.mutual_proximity}")  # 互接近

    # 获取单个手指的触觉传感器状态（另一种获取方式）
    thumb = await client.get_single_touch_sensor_status(slave_id, 0)
    index = await client.get_single_touch_sensor_status(slave_id, 1)
    middle = await client.get_single_touch_sensor_status(slave_id, 2)
    ring = await client.get_single_touch_sensor_status(slave_id, 3)
    pinky = await client.get_single_touch_sensor_status(slave_id, 4)

    # 记录拇指传感器数据
    logger.debug(f"Thumb Sensor Desc: {thumb.description}")
    logger.info(f"Thumb Sensor status: {thumb.status}")
    logger.info(f"Thumb normal_force1: {thumb.normal_force1}")  # 法向力1
    logger.info(f"Thumb normal_force2: {thumb.normal_force2}")  # 法向力2
    logger.info(f"Thumb tangential_force1: {thumb.tangential_force1}")  # 切向力1
    logger.info(f"Thumb tangential_force2: {thumb.tangential_force2}")  # 切向力2
    logger.info(f"Thumb tangential_direction1: {thumb.tangential_direction1}")  # 切向力方向1
    logger.info(f"Thumb tangential_direction2: {thumb.tangential_direction2}")  # 切向力方向2
    logger.info(f"Thumb self_proximity1: {thumb.self_proximity1}")  # 自接近1

    # 获取传感器原始通道值（可选，用于调试）
    # touch_raw_data = await client.get_touch_sensor_raw_data(slave_id)
    # logger.info(f"Touch Sensor Raw Data: {touch_raw_data.description}")
    # logger.debug(f"Touch Sensor Raw Data Thumb: {touch_raw_data.thumb}")
    # logger.debug(f"Touch Sensor Raw Data Index: {touch_raw_data.index}")
    # logger.debug(f"Touch Sensor Raw Data Middle: {touch_raw_data.middle}")
    # logger.debug(f"Touch Sensor Raw Data Ring: {touch_raw_data.ring}")
    # logger.debug(f"Touch Sensor Raw Data Pinky: {touch_raw_data.pinky}")

async def get_motor_status_periodically(client, slave_id):
    """
    定期获取电机状态并执行自动控制

    该函数会持续监控电机状态，并根据手指位置执行简单的开合动作：
    - 当手指张开时，执行握手动作
    - 当手指闭合时，执行张开动作

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    logger.info("Motor status monitoring started")
    index = 0

    while True:
        try:
            # 获取触觉传感器状态
            logger.debug("Getting touch sensor status...")
            await get_touch_status(client, slave_id)

            # 获取电机状态
            logger.debug("Getting motor status...")
            start = time.perf_counter()
            status: libstark.MotorStatusData = await client.get_motor_status(slave_id)
            cost_ms = (time.perf_counter() - start) * 1000

            # 记录状态信息
            logger.info(
                f"[{index}] Motor status - cost: {cost_ms:.2f}ms, "
                f"is_idle: {status.is_idle()}, "
                f"is_closed: {status.is_closed()}, "
                f"is_opened: {status.is_opened()}"
            )
            logger.info(f"[{index}] Motor status: {status.description}")
            index += 1

            # 根据当前状态执行动作
            if status.is_idle():
                if status.is_opened():
                    # 手指张开时，执行握手动作
                    await client.set_finger_positions(
                        slave_id, [60, 60, 100, 100, 100, 100]
                    )
                elif status.is_closed():
                    # 手指闭合时，执行张开动作
                    await client.set_finger_positions(slave_id, [0] * 6)

            # 添加延时避免过于频繁的查询
            await asyncio.sleep(0.001)

        except Exception as e:
            logger.error(f"Error in motor status monitoring: {e}")
            await asyncio.sleep(1)  # 发生错误时等待更长时间

async def set_finger_current(client, slave_id: int):
    """
    设置手指电流控制

    电流控制参数范围：-100~-20, 20~100
    正数表示闭合方向，负数表示张开方向

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 选择要控制的手指
    # finger_id = libstark.FingerId.Thumb     # 拇指
    # finger_id = libstark.FingerId.ThumbAux  # 拇指辅助
    # finger_id = libstark.FingerId.Index     # 食指
    # finger_id = libstark.FingerId.Middle    # 中指
    finger_id = libstark.FingerId.Ring       # 无名指
    # finger_id = libstark.FingerId.Pinky     # 小指

    # 设置单个手指电流
    await client.set_finger_current(slave_id, finger_id, -20)

    # 或设置所有手指电流
    # await client.set_finger_currents(slave_id, [-100] * 6)

if __name__ == "__main__":
    asyncio.run(main())
