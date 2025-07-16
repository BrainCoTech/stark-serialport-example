import asyncio
import sys
from revo1_utils import convert_to_mA, convert_to_position, get_stark_port_name, libstark, logger, convert_to_angle, open_modbus_revo1


# Main
async def main():
    (client, slave_id) = await open_modbus_revo1()

    logger.debug("get_serialport_cfg")  # 获取串口配置, 波特率
    baudrate = await client.get_serialport_baudrate(slave_id)
    logger.info(f"Baudrate: {baudrate}")

    # 基础版使用该功能，触觉版可以直接使用电流控制
    # logger.debug("get_force_level")  # 获取力量等级，大-中-小
    # force_level = await client.get_force_level(slave_id)
    # logger.info(f"Force level: {force_level}")

    logger.debug("get_voltage")  # 获取电量
    voltage = await client.get_voltage(slave_id)
    logger.info(f"Voltage: {voltage:.1f} mV")

    # await client.set_finger_speeds(slave_id, [100] * 6)  # 设置手指速度，速度环, 手指闭合
    # await client.set_finger_speeds(slave_id, [-100] * 6)  # 设置手指速度，速度环, 手指张开

    # 设置手指位置, 按物理角度
    angles = [20] * 6 # 一代手角度范围 [55, 90, 70, 70, 70, 70]
    await client.set_finger_positions(slave_id, convert_to_position(angles))
    await asyncio.sleep(1.0)

    # 握手
    await client.set_finger_positions(slave_id, [100, 100, 100, 100, 100, 100])

    # 等待手指到达目标位置
    await asyncio.sleep(1.0)

    # 张开
    await client.set_finger_positions(slave_id, [0] * 6)
    await asyncio.sleep(1.5)

    logger.debug("get_motor_status")  # 获取手指状态, 位置，电流，motor状态
    status = await client.get_motor_status(slave_id)
    logger.info(f"positions(0~100): {list(status.positions)}")
    logger.info(f"angles(角度): {convert_to_angle(list(status.positions))}") # 角度
    logger.info(f"currents: {list(status.currents)}")
    logger.info(f"currents(mA): {convert_to_mA(list(status.currents))}") # mA
    logger.info(f"states: {list(status.states)}")
    logger.debug(f"motor status: {status.description}")

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
