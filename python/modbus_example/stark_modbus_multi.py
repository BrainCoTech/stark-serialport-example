import asyncio
import logging
import sys
import time
from logger import getLogger
from stark_utils import get_stark_port_name
from utils import setup_shutdown_event
import bc_device_sdk

libstark = bc_device_sdk.stark

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

slave_ids = [1, 2] # 灵巧手的设备ID, 多个设备在同一个BUS上

async def handle_finger_status(client, slave_id, index):
    status = await client.get_motor_status(slave_id)
    logger.info(f"[{index}] [{slave_id}] Finger status: {status.description}")
    if status.is_idle:
        if status.is_opened:
            # 握手
            await client.set_finger_positions(slave_id, [60, 60, 100, 100, 100, 100])
        elif status.is_closed:
            # 张开
            await client.set_finger_positions(slave_id, [0] * 6)

# 定期获取手指状态
async def get_motor_status_periodically(client):
    logger.info("get_motor_status_periodically start")
    index = 0
    while True:
        try:
            for slave_id in slave_ids:
                await handle_finger_status(client, slave_id, index)
            index += 1


        except Exception as e:
            logger.error(f"Error getting finger status: {e}")


# Main
async def main():
    shutdown_event = setup_shutdown_event(logger)

    port_name = get_stark_port_name()
    if port_name is None:
        return

    client = await libstark.modbus_open(port_name, libstark.Baudrate.Baud115200, 1)

    for slave_id in slave_ids:
        await client.set_finger_positions(slave_id, [60, 60, 100, 100, 100, 100])  # 握手

    # 创建并启动异步任务
    asyncio.create_task(get_motor_status_periodically(client))
    logger.info("Status task started")

    # 等待关闭事件
    await shutdown_event.wait()

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
