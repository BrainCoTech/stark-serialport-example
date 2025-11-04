import asyncio
import sys
import pathlib

from zqwl_win import *
from canfd_utils import *


def canfd_send(slave_id: int, can_id: int, data: list):
    # logger.debug(f"Sending CAN ID: {can_id}, Data: {data}, type: {type(data)}")
    if not zcan_send_message(slave_id, can_id, bytes(data)):
        logger.error("Failed to send CANFD message")
        return


def canfd_read(slave_id: int):
    recv_msg = zcan_receive_message()
    if recv_msg is None:
        logger.error("No message received")
        return 0, bytes([])

    can_id = recv_msg.frame.can_id
    data = [recv_msg.frame.data[i] for i in range(recv_msg.frame.len)]
    logger.debug(f"Received CAN ID: {can_id:029b}, Data: {bytes(data).hex()}")
    return can_id, data


async def configure_control_mode(client, slave_id):
    """
    配置手指控制模式

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 设置手指控制参数的单位模式
    logger.debug("set_finger_unit_mode")
    await client.set_finger_unit_mode(
        slave_id, libstark.FingerUnitMode.Normalized
    )  # 千分比模式
    # await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Physical)  # 物理量模式

    # 获取并验证手指控制模式
    logger.debug("get_finger_unit_mode")
    finger_unit_mode = await client.get_finger_unit_mode(slave_id)
    logger.info(f"Finger unit mode: {finger_unit_mode}")

    # 参考文档：https://www.brainco-hz.com/docs/revolimb-hand/revo2/modbus_foundation.html#%E5%8D%95%E4%BD%8D%E6%A8%A1%E5%BC%8F%E8%AE%BE%E7%BD%AE-937


async def configure_finger_parameters(client, slave_id):
    """
    配置手指参数（可选）

    设置手指的最大角度、最小角度、最大速度、最大电流、保护电流等参数。
    各个手指参数范围详见官方文档。

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    finger_id = libstark.FingerId.Middle  # 选择中指作为示例

    # 设置最小位置（角度）
    # await client.set_finger_min_position(slave_id, finger_id, 0)
    # min_position = await client.get_finger_min_position(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] min position: {min_position}")

    # 设置最大位置（角度）
    # await client.set_finger_max_position(slave_id, finger_id, 80)
    # max_position = await client.get_finger_max_position(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max position: {max_position}")

    # 设置最大速度
    # await client.set_finger_max_speed(slave_id, finger_id, 130)
    # max_speed = await client.get_finger_max_speed(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max speed: {max_speed}")

    # 设置最大电流
    # await client.set_finger_max_current(slave_id, finger_id, 1000)
    # max_current = await client.get_finger_max_current(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max current: {max_current}")

    # 设置保护电流
    # await client.set_finger_protected_current(slave_id, finger_id, 500)
    # protected_current = await client.get_finger_protected_current(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] protected current: {protected_current}")


async def execute_control_examples(client, slave_id):
    """
    执行各种控制方式的示例

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    finger_id = libstark.FingerId.Index  # 选食指作为示例

    # 单个手指控制示例
    await single_finger_control_examples(client, slave_id, finger_id)

    # 全部手指控制示例
    await all_fingers_control_examples(client, slave_id)


async def single_finger_control_examples(client, slave_id, finger_id):
    """
    单个手指控制示例

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
        finger_id: 手指ID
    """
    # 速度控制：符号表示方向，正值为握紧方向，负值为松开方向
    await client.set_finger_speed(slave_id, finger_id, 500)  # 范围：-1000 ~ 1000
    await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 电流控制：符号表示方向，正值为握紧方向，负值为松开方向
    await client.set_finger_current(slave_id, finger_id, -300)  # 范围：-1000 ~ 1000
    await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # PWM控制：符号表示方向，正值为握紧方向，负值为松开方向
    # await client.set_finger_pwm(slave_id, finger_id, 700)  # 范围：-1000 ~ 1000
    # await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 位置控制：目标位置 + 期望时间
    await client.set_finger_position_with_millis(slave_id, finger_id, 1000, 1000)
    await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 位置控制：目标位置 + 期望速度
    await client.set_finger_position_with_speed(slave_id, finger_id, 1, 50)
    await asyncio.sleep(1.0)  # 等待手指到达目标位置


async def all_fingers_control_examples(client, slave_id):
    """
    全部手指控制示例

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 全部手指速度控制：符号表示方向，正值为握紧方向，负值为松开方向
    await client.set_finger_speeds(slave_id, [500] * 6)
    await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 全部手指电流控制：符号表示方向，正值为握紧方向，负值为松开方向
    await client.set_finger_currents(slave_id, [-300] * 6)
    await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 全部手指PWM控制：符号表示方向，正值为握紧方向，负值为松开方向
    await client.set_finger_pwms(slave_id, [300] * 6)
    await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 全部手指位置控制：目标位置 + 期望时间
    # 位置值：[拇指, 食指, 中指, 无名指, 小指, 手腕]
    positions = [500] * 6
    durations = [300] * 6  # 到达目标位置的期望时间（毫秒）
    await client.set_finger_positions_and_durations(slave_id, positions, durations)
    await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 全部手指位置控制：目标位置 + 期望速度
    positions = [500, 500] + [1000] * 4
    speeds = [500] * 6  # 到达目标位置的期望速度
    await client.set_finger_positions_and_speeds(slave_id, positions, speeds)
    await asyncio.sleep(1.0)  # 等待手指到达目标位置


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


async def get_motor_status_periodically(client, slave_id):
    logger.info(f"Motor status monitoring started for device {slave_id:02x}")
    while True:
        try:
            await get_and_display_motor_status(client, slave_id)
            # 添加延时避免过于频繁的查询
            await asyncio.sleep(0.001)

        except Exception as e:
            logger.error(
                f"Error in motor status monitoring for device {slave_id:02x}: {e}"
            )
            await asyncio.sleep(1)  # 发生错误时等待更长时间再重试


async def main():
    # fmt: off
    """
    主函数：初始化Revo2灵巧手并执行控制示例
    """
    # 连接Revo2设备
    master_id = 1
    slave_id = 0x7f # 左手默认ID为0x7e，右手默认ID为0x7f
    client = libstark.PyDeviceContext.init_canfd(master_id)

    # ZCAN_USBCANFD_100U
    zcan_open(device_type=42, channel=0, baudrate=5000000)
    libstark.set_can_tx_callback(canfd_send)
    libstark.set_can_rx_callback(canfd_read)

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD, Baudrate: {baudrate}")

    # 配置控制模式
    await configure_control_mode(client, slave_id)

    # 配置手指参数（可选）
    await configure_finger_parameters(client, slave_id)

    shutdown_event = setup_shutdown_event(logger)

    # 获取电机状态
    reader_task = asyncio.create_task(get_motor_status_periodically(client, slave_id))

    # 执行控制示例
    await execute_control_examples(client, slave_id)

    await shutdown_event.wait()  # 等待关闭事件
    logger.info("Shutdown event received, stopping motor status monitoring...")
    reader_task.cancel()  # 取消电机状态监控任务
    # 关闭资源
    zcan_close()
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
