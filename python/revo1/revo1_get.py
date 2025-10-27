"""
Revo1灵巧手基础控制示例

本示例演示如何控制Revo1灵巧手设备，包括：
- 设备信息获取和串口配置
- 电压和力量等级监控
- 手指位置控制和动作执行
- 电机状态监控和数据读取
- 速度控制和电流控制示例
"""

import asyncio
import sys
from revo1_utils import *

async def main():
    """
    主函数：初始化灵巧手连接并执行基础控制操作
    """
    # 检测灵巧手的波特率和设备ID，初始化client
    (client, slave_id) = await open_modbus_revo1()

    # 获取串口配置信息
    logger.debug("get_serialport_cfg")  # 获取串口配置，波特率
    baudrate = await client.get_serialport_baudrate(slave_id)
    logger.info(f"Baudrate: {baudrate}")

    # 获取设备信息并检查设备类型
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)

    # 触觉版设备使用电流控制接口
    # 非触觉版设备使用力量等级配置
    if not device_info.is_revo1_touch():
        logger.debug("get_force_level")  # 获取力量等级：大-中-小
        force_level = await client.get_force_level(slave_id)
        logger.info(f"Force level: {force_level}")

    # 获取设备电压状态
    logger.debug("get_voltage")  # 获取电池电量
    voltage = await client.get_voltage(slave_id)
    logger.info(f"Voltage: {voltage:.1f} mV")

    # 速度控制示例（已注释）
    # await client.set_finger_speeds(slave_id, [100] * 6)  # 设置手指速度，速度环，手指闭合
    # await client.set_finger_speeds(slave_id, [-100] * 6)  # 设置手指速度，速度环，手指张开

    # 执行手指位置控制序列
    await execute_finger_movements(client, slave_id)

    # 获取并显示电机状态信息
    await get_and_display_motor_status(client, slave_id)

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


async def execute_finger_movements(client, slave_id):
    """
    执行手指动作序列：设置角度 -> 握手 -> 张开

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    # 设置手指位置，使用物理角度
    angles = [20] * 6  # 一代手角度范围 [55, 90, 70, 70, 70, 70]
    await client.set_finger_positions(slave_id, convert_to_position(angles))
    await asyncio.sleep(1.0)

    # 执行握手动作（所有手指设置为最大位置值）
    await client.set_finger_positions(slave_id, [60, 60, 100, 100, 100, 100])

    # 等待手指到达目标位置
    await asyncio.sleep(1.0)

    # 执行张开动作（所有手指设置为最小位置值）
    await client.set_finger_positions(slave_id, [0] * 6)
    await asyncio.sleep(1.5)


async def get_and_display_motor_status(client, slave_id):
    """
    获取并显示电机状态信息

    包括：
    - 手指位置（0~100范围和物理角度）
    - 电机电流（原始值和mA单位）
    - 电机运行状态

    Args:
        client: Modbus客户端实例
        slave_id: 设备ID
    """
    logger.debug("get_motor_status")  # 获取手指状态：位置、电流、motor状态
    status: libstark.MotorStatusData = await client.get_motor_status(slave_id)

    # 显示位置信息
    logger.info(f"positions(0~100): {list(status.positions)}")
    logger.info(f"angles(角度): {convert_to_angle(list(status.positions))}")  # 物理角度

    # 显示电流信息
    logger.info(f"currents: {list(status.currents)}")
    logger.info(f"currents(mA): {convert_to_mA(list(status.currents))}")  # 转换为mA单位

    # 显示电机状态
    logger.info(f"states: {list(status.states)}")
    logger.debug(f"motor status: {status.description}")


if __name__ == "__main__":
    asyncio.run(main())
