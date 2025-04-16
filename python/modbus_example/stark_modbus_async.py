import asyncio
import sys
import time
from utils import setup_shutdown_event
from stark_utils import get_stark_port_name, libstark, logger


# 定期获取手指状态
async def get_motor_status_periodically(client, slave_id):
    logger.info("get_motor_status_periodically start")
    index = 0
    while True:
        try:
            logger.debug("get_motor_status")
            start = time.time()
            status = await client.get_motor_status(slave_id)
            cost_ms = (time.time() - start) * 1000
            logger.info(
                f"[{index}] Finger status, cost: {cost_ms:.2f} ms, is_idle: {status.is_idle}, is_closed: {status.is_closed}, is_opened: {status.is_opened}"
            )
            logger.info(f"[{index}] Finger status: {status.description}")
            index += 1

            if status.is_idle:
                if status.is_opened:
                    # 握手
                    await client.set_finger_positions(
                        slave_id, [60, 60, 100, 100, 100, 100]
                    )
                elif status.is_closed:
                    # 张开
                    await client.set_finger_positions(slave_id, [0] * 6)

        except Exception as e:
            logger.error(f"Error getting finger status: {e}")


# Main
async def main():
    libstark.init_config(libstark.StarkFirmwareType.V1Standard)
    shutdown_event = setup_shutdown_event(logger)

    port_name = get_stark_port_name()
    if port_name is None:
        return
    slave_id = 1
    client = await libstark.modbus_open(
        port_name, libstark.Baudrate.Baud115200, slave_id
    )

    await client.set_finger_positions(slave_id, [60, 60, 100, 100, 100, 100])  # 握手

    # 创建并启动异步任务
    asyncio.create_task(get_motor_status_periodically(client, slave_id))
    logger.info("Status task started")

    # 等待关闭事件
    await shutdown_event.wait()

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
