"""
Revo1灵巧手自动控制示例

本示例演示如何实现Revo1灵巧手的自动控制，包括：
- 灵巧手的初始化和连接
- 电机状态的定期监控
- 基于状态的自动握手和张开动作
- 单个手指的精确控制
- 异步任务的创建和管理
"""

import asyncio
import sys
import time
from utils import setup_shutdown_event
from revo1_utils import *


async def get_motor_status_periodically(client, slave_id):
    """
    定期获取电机状态并执行自动控制

    该函数会持续监控电机状态，并根据手指位置执行自动开合动作：
    - 当手指张开时，执行握手动作
    - 当手指闭合时，执行张开动作
    - 记录每次操作的耗时和状态信息

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    logger.info("Motor status monitoring started")
    index = 0

    while True:
        try:
            # 获取电机状态
            logger.debug("get_motor_status")
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

            # 根据当前状态执行自动控制
            if status.is_idle():
                if status.is_opened():
                    # 手指张开时，执行握手动作
                    # 位置值：[拇指, 拇指辅助, 食指, 中指, 无名指, 小指]
                    await client.set_finger_positions(
                        slave_id, [60, 60, 100, 100, 100, 100]
                    )
                elif status.is_closed():
                    # 手指闭合时，执行张开动作
                    # 所有手指设置为最小位置值（0）
                    await client.set_finger_positions(slave_id, [0] * 6)

        except Exception as e:
            logger.error(f"Error in motor status monitoring: {e}")
            # 发生错误时等待1秒后重试
            await asyncio.sleep(1)


async def main():
    """
    主函数：初始化灵巧手连接并启动自动控制任务
    """
    # 设置关闭事件监听
    shutdown_event = setup_shutdown_event(logger)

    # 检测灵巧手的波特率和设备ID，初始化client
    (client, slave_id) = await open_modbus_revo1(port_name=None, quick=True) # 替换为实际的串口名称, 传None会尝试自动检测

    # 执行初始化动作
    # 设置单个手指控制：小指闭合到100%位置
    await client.set_finger_position(slave_id, libstark.FingerId.Pinky, 100)
    await asyncio.sleep(1)  # 等待1秒

    # 执行握手动作：设置所有手指到预定位置
    # 位置参数：[拇指60%, 拇指辅助60%, 食指100%, 中指100%, 无名指100%, 小指100%]
    await client.set_finger_positions(slave_id, [60, 60, 100, 100, 100, 100])
    await asyncio.sleep(1)  # 等待1秒

    # 执行OPEN动作：设置所有手指到最小位置（0%）
    await client.set_finger_positions(slave_id, [0] * 6)
    await asyncio.sleep(1)  # 等待1秒

    # 创建并启动电机状态监控任务
    asyncio.create_task(get_motor_status_periodically(client, slave_id))
    logger.info("Motor status monitoring task started")

    # 等待关闭事件（Ctrl+C 或其他关闭信号）
    await shutdown_event.wait()

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
