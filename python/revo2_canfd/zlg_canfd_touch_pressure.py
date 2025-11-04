import asyncio
import sys
from canfd_utils import *

# 根据操作系统导入对应的 ZLG CAN 驱动模块
if platform.system() == "Windows":
    from zlg_win import *
elif platform.system() == "Linux":
    from zlg_linux import *
else:
    raise NotImplementedError(f"不支持的操作系统: {platform.system()}")

def canfd_send(_slave_id: int, can_id: int, data: list):
    # logger.debug(f"Sending CAN ID: {can_id}, Data: {data}, type: {type(data)}")
    if not zlgcan_send_message(can_id, bytes(data)):
        logger.error("Failed to send CANFD message")
        return

def canfd_read(_slave_id: int):
    recv_msg = zlgcan_receive_message()
    if recv_msg is None:
        logger.error("No message received")
        return 0, bytes([])

    (can_id, data) = recv_msg
    logger.debug(f"Received CANFD - ID: {can_id:02x}, Data: {bytes(data).hex()}")
    return can_id, data

async def get_and_display_motor_status(client, slave_id):
    """
    获取并显示电机状态信息

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    logger.debug("get_motor_status")
    status: libstark.MotorStatusData = await client.get_motor_status(slave_id)

    # 显示详细状态信息
    # logger.info(f"positions: {list(status.positions)}")  # 位置
    # logger.info(f"speeds: {list(status.speeds)}")        # 速度
    # logger.info(f"currents: {list(status.currents)}")    # 电流
    # logger.info(f"states: {list(status.states)}")        # 状态
    logger.info(f"Finger status: {status.description}")
    
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

async def get_motor_status_periodically(client, slave_id):
    logger.info(f"Motor status monitoring started for device {slave_id:02x}")
    while True:
        try:
            await get_and_display_motor_status(client, slave_id)
            # 添加延时避免过于频繁的查询
            await asyncio.sleep(0.001)
            # break

        except Exception as e:
            logger.error(
                f"Error in motor status monitoring for device {slave_id:02x}: {e}"
            )
            await asyncio.sleep(1)  # 发生错误时等待更长时间再重试

async def setup_touch_sensors(client, slave_id):
    """
    配置和启用触觉传感器

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # logger.debug("setup_touch_sensors")
    # bits = 0x3F  # 0x3F: 启用五指+手掌
    # await client.touch_sensor_setup(slave_id, bits)
    # await asyncio.sleep(1)  # 等待触觉传感器准备就绪

    # 验证传感器启用状态
    logger.debug("get_touch_sensor_enabled")
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x3F):06b}")

    # 获取触觉传感器固件版本（需要在启用后才能获取）
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    # await client.get_modulus_touch_data_type(slave_id, libstark.ModulusTouchDataType.Calibrated)
    data_tpye = await client.get_modulus_touch_data_type(slave_id)
    logger.info(f"Modulus Touch Data Type: {data_tpye}")

async def main():
    # fmt: off
    """
    主函数：初始化Revo2灵巧手并执行控制示例
    """
    # 连接Revo2设备
    master_id = 1
    slave_id = 0x7e # 左手默认ID为0x7e，右手默认ID为0x7f
    client = libstark.PyDeviceContext.init_canfd(master_id)

    zlgcan_open()
    libstark.set_can_tx_callback(canfd_send)
    libstark.set_can_rx_callback(canfd_read)

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD, Baudrate: {baudrate}")
    
    if not client.is_touch_pressure():
        logger.error("This example is only for Revo2 Touch Pressure hardware")
        zlgcan_close()
        sys.exit(1)

    # 配置触觉传感器
    await setup_touch_sensors(client, slave_id)

    shutdown_event = setup_shutdown_event(logger)

    # 获取电机状态
    reader_task = asyncio.create_task(get_motor_status_periodically(client, slave_id))

    await shutdown_event.wait()  # 等待关闭事件
    logger.info("Shutdown event received, stopping motor status monitoring...")
    reader_task.cancel()  # 取消电机状态监控任务
    # 关闭资源
    zlgcan_close()
    sys.exit(0)

if __name__ == "__main__":
    asyncio.run(main())
