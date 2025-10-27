"""
Revo1灵巧手配置管理示例

本示例演示如何管理Revo1灵巧手的各种配置参数，包括：
- 串口配置（从站ID、波特率）
- 力量等级设置（大、中、小）
- Turbo模式配置和管理
- 位置校准设置和执行
- 设备重启和重新连接

重要注意事项：
- 修改从站ID或波特率后设备会自动重启
- 重启后需要使用新的配置参数重新连接设备
- 触觉版建议直接使用电流控制，基础版使用力量等级控制
- 位置校准会影响手指的基准位置
"""

import asyncio
import sys
from revo1_utils import *


async def set_slave_id(client, slave_id, new_slave_id):
    """
    修改设备从站ID

    修改后设备会自动重启，需要使用新的从站ID重新连接。

    Args:
        client: Modbus客户端实例
        slave_id: 当前从站ID
        new_slave_id: 新的从站ID（范围：1-247）
    """
    await client.set_slave_id(slave_id, new_slave_id)
    logger.info(f"Slave ID set to {new_slave_id}, device will reboot in 3 seconds")

    # 关闭当前连接
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    logger.info(f"New Slave ID: {new_slave_id}, please reconnect the device with new Slave ID")
    sys.exit(0)


async def set_baudrate(client, slave_id, new_baudrate):
    """
    修改设备波特率

    修改后设备会自动重启，需要使用新的波特率重新连接。

    Args:
        client: Modbus客户端实例
        slave_id: 设备从站ID
        new_baudrate: 新的波特率（支持的波特率见libstark.Baudrate枚举）
    """
    # 获取当前波特率
    baudrate = await client.get_serialport_baudrate(slave_id)
    logger.info(f"Current Baudrate: {baudrate}")

    # 设置新波特率
    await client.set_serialport_baudrate(slave_id, new_baudrate)
    logger.info(f"Baudrate set to {new_baudrate}, device will reboot in 3 seconds")

    # 关闭当前连接
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    logger.info(f"New Baudrate: {new_baudrate}, please reconnect the device with new Baudrate")
    sys.exit(0)


async def configure_force_level(client, slave_id):
    """
    配置力量等级

    触觉版本可直接使用电流控制，基础版使用该功能设置力量等级。

    Args:
        client: Modbus客户端实例
        slave_id: 设备从站ID
    """
    # 设置力量等级（可选）
    # await client.set_force_level(slave_id, force_level=libstark.ForceLevel.Full)    # 最大力量
    # await client.set_force_level(slave_id, force_level=libstark.ForceLevel.Normal)  # 正常力量
    # await client.set_force_level(slave_id, force_level=libstark.ForceLevel.Small)   # 小力量

    # 获取并显示当前力量等级
    level = await client.get_force_level(slave_id)
    logger.info(f"Current Force level: {level}")


async def configure_turbo_mode(client, slave_id):
    """
    配置Turbo模式

    Turbo模式可以提高设备的响应速度和性能。

    Args:
        client: Modbus客户端实例
        slave_id: 设备从站ID
    """
    # 设置Turbo模式开关（可选）
    # await client.set_turbo_mode_enabled(slave_id, True)   # 开启Turbo模式
    # await client.set_turbo_mode_enabled(slave_id, False)  # 关闭Turbo模式

    # 获取并显示Turbo模式状态
    turbo_mode = await client.get_turbo_mode_enabled(slave_id)
    if turbo_mode:
        logger.info("Turbo mode: enabled")
    else:
        logger.info("Turbo mode: disabled")

    # 配置Turbo参数（可选）
    # logger.debug("set_turbo_conf")
    # turbo_interval = 2000  # Turbo间隔时间（毫秒）
    # turbo_duration = 3000  # Turbo持续时间（毫秒）
    # turbo_conf = libstark.TurboConfig(turbo_interval, turbo_duration)
    # await client.set_turbo_config(slave_id, turbo_conf)
    #
    # # 获取并显示Turbo配置
    # turbo_conf = await client.get_turbo_config(slave_id)
    # logger.info(f"Turbo conf: {turbo_conf.description}")


async def configure_position_calibration(client, slave_id):
    """
    配置位置校准

    位置校准用于设置手指的基准位置，确保位置控制的准确性。

    Args:
        client: Modbus客户端实例
        slave_id: 设备从站ID
    """
    # 设置是否在上电后自动执行位置校准（可选）
    # 注意：修改此设置后设备会自动重启
    # await client.set_auto_calibration(slave_id, False)

    # 获取并显示自动校准状态
    # calibration_enabled = await client.get_auto_calibration_enabled(slave_id)
    # logger.info(f"Auto calibration enabled: {calibration_enabled}")

    # 手动执行位置校准（可选）
    # 建议在执行校准前先将手指移动到合适的位置
    # await client.set_finger_positions(slave_id, [60, 60, 100, 100, 100, 100])  # 握手动作
    # await client.calibrate_position(slave_id)  # 执行位置校准
    # logger.info("Position calibration completed")


async def main():
    """
    主函数：设备配置管理
    """
    # 连接Revo1设备
    (client, slave_id) = await open_modbus_revo1()

    # 获取并显示串口配置信息
    logger.debug("get_serialport_cfg")
    serialport_cfg = await client.get_serialport_cfg(slave_id)
    logger.info(f"Serial Port Config: {serialport_cfg.description}")

    # 如果只需要查看当前配置，可以在此处退出
    # exit(0)

    # 根据需要选择以下配置操作之一：

    # 1. 修改从站ID（取消注释以使用）
    # logger.debug("set_slave_id")
    # await set_slave_id(client, slave_id, new_slave_id=1)  # 修改slave_id为1
    # await set_slave_id(client, slave_id, new_slave_id=2)  # 修改slave_id为2
    # return  # 修改后会退出程序

    # 2. 修改波特率（取消注释以使用）
    # logger.debug("set_baudrate")
    # await set_baudrate(client, slave_id, new_baudrate=libstark.Baudrate.Baud115200)  # 修改波特率为115200
    # await set_baudrate(client, slave_id, new_baudrate=libstark.Baudrate.Baud460800)  # 修改波特率为460800
    # return  # 修改后会退出程序

    # 3. 配置力量等级（仅适用于非触觉版）
    await configure_force_level(client, slave_id)

    # 4. 配置Turbo模式
    await configure_turbo_mode(client, slave_id)

    # 5. 配置位置校准
    await configure_position_calibration(client, slave_id)

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
