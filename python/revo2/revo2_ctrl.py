"""
Revo2灵巧手控制示例

本示例演示如何控制Revo2灵巧手设备，包括：
- 手指控制模式的设置（千分比模式/物理量模式）
- 手指参数的配置（位置、速度、电流限制）
- 多种控制方式（速度控制、电流控制、PWM控制、位置控制）
- 单个手指和全部手指的控制
- 电机状态的获取和监控

注意事项：
- 控制参数的符号表示方向：正值为握紧方向，负值为松开方向
- 各个手指的参数范围详见官方文档
- 建议在执行控制指令后等待适当时间让手指到达目标位置
"""

import asyncio
import sys
from revo2_utils import libstark, logger, open_modbus_revo2


async def main():
    """
    主函数：初始化Revo2灵巧手并执行控制示例
    """
    # 连接Revo2设备
    (client, slave_id) = await open_modbus_revo2(port_name="/dev/ttyUSB0")

    # 配置控制模式
    await configure_control_mode(client, slave_id)

    # 配置手指参数（可选）
    await configure_finger_parameters(client, slave_id)

    # 执行控制示例
    await execute_control_examples(client, slave_id)

    # 获取并显示电机状态
    await get_and_display_motor_status(client, slave_id)

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def configure_control_mode(client, slave_id):
    """
    配置手指控制模式

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 设置手指控制参数的单位模式
    logger.debug("set_finger_unit_mode")
    await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized)  # 千分比模式
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
    finger_id = libstark.FingerId.Middle  # 选择中指作为示例

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
    # await client.set_finger_speed(slave_id, finger_id, 500)  # 范围：-1000 ~ 1000
    # await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 电流控制：符号表示方向，正值为握紧方向，负值为松开方向
    # await client.set_finger_current(slave_id, finger_id, -300)  # 范围：-1000 ~ 1000
    # await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # PWM控制：符号表示方向，正值为握紧方向，负值为松开方向
    # await client.set_finger_pwm(slave_id, finger_id, 700)  # 范围：-1000 ~ 1000
    # await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 位置控制：目标位置 + 期望时间
    # await client.set_finger_position_with_millis(slave_id, finger_id, 1000, 1000)
    # await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 位置控制：目标位置 + 期望速度
    # await client.set_finger_position_with_speed(slave_id, finger_id, 1, 50)
    # await asyncio.sleep(1.0)  # 等待手指到达目标位置


async def all_fingers_control_examples(client, slave_id):
    """
    全部手指控制示例

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 全部手指速度控制：符号表示方向，正值为握紧方向，负值为松开方向
    # await client.set_finger_speeds(slave_id, [500] * 6)
    # await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 全部手指电流控制：符号表示方向，正值为握紧方向，负值为松开方向
    # await client.set_finger_currents(slave_id, [-300] * 6)
    # await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 全部手指PWM控制：符号表示方向，正值为握紧方向，负值为松开方向
    # await client.set_finger_pwms(slave_id, [700] * 6)
    # await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 全部手指位置控制：目标位置 + 期望时间
    # 位置值：[拇指, 食指, 中指, 无名指, 小指, 手腕]
    positions = [500] * 6
    durations = [300] * 6  # 到达目标位置的期望时间（毫秒）
    await client.set_finger_positions_and_durations(slave_id, positions, durations)
    await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 全部手指位置控制：目标位置 + 期望速度
    positions = [1000] * 6
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


if __name__ == "__main__":
    asyncio.run(main())
