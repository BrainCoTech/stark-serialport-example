"""
Revo1灵巧手控制双手示例

连接方式说明：
- 使用两个串口分别连接左手和右手（可以使用相同的设备ID，设备ID默认为1）
"""

import asyncio
import sys
import time
from revo1_utils import *
from utils import setup_shutdown_event

LEFT_PORT = "/dev/ttyUSB0" # 替换为实际左手设备端口
RIGHT_PORT = "/dev/ttyUSB1" # 替换为实际右手设备端口
LEFT_ID = 1 # 替换为实际左手设备ID
RIGHT_ID = 1 # 替换为实际右手设备ID
BAUDRATE = libstark.Baudrate.Baud115200

async def main():
    """
    主函数：初始化设备连接和控制任务
    """
    libstark.init_config(libstark.StarkProtocolType.Modbus)
    shutdown_event = setup_shutdown_event(logger)

    # 打开两个设备
    client_left = await libstark.modbus_open(LEFT_PORT, BAUDRATE)
    client_right = await libstark.modbus_open(RIGHT_PORT, BAUDRATE)
    if not client_left or not client_right:
        logger.critical("Failed to open one or both serial ports.")
        sys.exit(1)

    # 获取设备信息
    left_info = await client_left.get_device_info(LEFT_ID)
    right_info = await client_right.get_device_info(RIGHT_ID)
    if not left_info or not right_info:
        logger.critical("Failed to get device info for one or both devices.")
        sys.exit(1)
    logger.info(f"Left: {left_info.description}")
    logger.info(f"Right: {right_info.description}")

    # 启动电机状态监控任务
    task = asyncio.create_task(
        monitor_and_control(client_left, client_right, LEFT_ID, RIGHT_ID)
    )

    # 等待关闭事件
    await shutdown_event.wait()

    # 清理资源
    task.cancel()
    await asyncio.gather(task, return_exceptions=True)
    await client_left.modbus_close()
    await client_right.modbus_close()
    logger.info("Modbus clients closed")
    sys.exit(0)

async def monitor_and_control(client_left, client_right, left_id, right_id):
    """
    定期获取电机状态并执行自动控制
    """
    logger.info(f"Motor status monitoring started for devices {left_id:02x} and {right_id:02x}")
    index = 0
    positions = [0] * 6
    period = 2.0

    while True:
        try:
            index += 1
            now_ms = int(time.time() * 1000)
            t = (now_ms / 1000.0) % period
            if t < period / 2:
                positions = [0, 0, 0, 0, 0, 0]
            else:
                positions = [0, 0, 100, 100, 100, 100]

            # 获取电机状态
            for client, dev_id, hand in [
                (client_left, left_id, "Left"),
                (client_right, right_id, "Right"),
            ]:
                start = time.perf_counter()
                try:
                    status = await client.get_motor_status(dev_id)
                    cost_ms = (time.perf_counter() - start) * 1000
                    logger.debug(
                        f"{hand} [{dev_id:02x}] [{index}] Motor status - cost: {cost_ms:.2f}ms, "
                        f"is_idle: {status.is_idle()}, is_closed: {status.is_closed()}, is_opened: {status.is_opened()}"
                    )
                    logger.debug(f"{hand} [{dev_id:02x}] [{index}] Motor status: {status.description}")
                except Exception as e:
                    logger.error(f"Error getting motor status for {hand} [{dev_id:02x}]: {e}")

            logger.info(f"Publishing positions: {positions}")
            await asyncio.gather(
                client_left.set_finger_positions(left_id, positions),
                client_right.set_finger_positions(right_id, positions),
            )
            await asyncio.sleep(0.01)
        except Exception as e:
            logger.error(f"Error in monitor_and_control loop: {e}")
            await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
