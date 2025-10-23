"""
Revo2灵巧手控制双手示例

连接方式说明：
- 使用两个串口分别连接左手和右手（可以使用相同的设备ID，左右默认为0x7e，右手默认为0x7f）
"""

import asyncio
import random
import sys
import time
from revo2_utils import *
from utils import setup_shutdown_event

LEFT_PORT = "/dev/ttyUSB0" # 替换为实际的串口名称
RIGHT_PORT = "/dev/ttyUSB1" # 替换为实际的串口名称
LEFT_ID = 0x7e
RIGHT_ID = 0x7f
BAUDRATE = libstark.Baudrate.Baud460800

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
    speeds = [1000] * 6
    period = 2.0

    while True:
        try:
            index += 1
            now_ms = int(time.time() * 1000)
            t = (now_ms / 100.0) % period

            if t < period / 2:
                pos_1 = random.randint(0, 590)
                pos_2 = random.randint(0, 1000)
                positions = [pos_1, pos_1, pos_2, pos_2, pos_2, pos_2]
            else:
                positions = [0, 0, 1000, 1000, 1000, 1000]

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
                        f"is_idle: {status.is_idle()}, is_closed: {is_positions_closed(status)}, is_opened: {is_positions_open(status)}"
                    )
                    logger.debug(f"{hand} [{dev_id:02x}] [{index}] Motor status: {status.description}")
                except Exception as e:
                    logger.error(f"Error getting motor status for {hand} [{dev_id:02x}]: {e}")

            logger.info(f"Publishing positions: {positions}")
            await asyncio.gather(
                client_left.set_finger_positions_and_speeds(left_id, positions, speeds),
                client_right.set_finger_positions_and_speeds(right_id, positions, speeds),
            )
            await asyncio.sleep(0.01)
        except Exception as e:
            logger.error(f"Error in monitor_and_control loop: {e}")
            await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
