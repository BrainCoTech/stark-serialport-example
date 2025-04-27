import asyncio
import sys
from stark_utils import get_stark_port_name, libstark, logger


# Main
async def main():
    libstark.init_config(libstark.StarkFirmwareType.V2Standard)
    port_name = get_stark_port_name()
    if port_name is None:
        return

    slave_id = 0x7E  # 左手默认ID为0x7e，右手默认ID为0x7f
    # fmt: off
    client = await libstark.modbus_open(port_name, libstark.Baudrate.Baud460800, slave_id)

    logger.debug("get_serialport_baudrate")  # 获取串口配置, 波特率
    baudrate = await client.get_serialport_baudrate(slave_id)
    logger.info(f"Baudrate: {baudrate}")

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    # logger.info(f"Device info: {device_info.firmware_version}")  # 固件版本
    # logger.info(f"Device info: {device_info.serial_number}")  # 序列号
    # logger.info(f"Device info: {device_info.sku_type}")  # 获取手类型, 左右手
    logger.info(f"Device info: {device_info.description}")

    # TODO: NOT IMPLEMENTED
    # logger.debug("get_voltage")  # 获取电量
    # voltage = await client.get_voltage(slave_id)
    # logger.info(f"Voltage: {voltage:.1f} mV")

    # set_finger_unit_mode
    logger.debug("set_finger_unit_mode")  # 设置手指控制参数的单位模式
    await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized) # 归一化，参数范围详见文档
    # await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Physical) # 物理单位
    # get_finger_unit_mode
    logger.debug("get_finger_unit_mode")  # 获取手指单位模式
    finger_unit_mode = await client.get_finger_unit_mode(slave_id)
    logger.info(f"Finger unit mode: {finger_unit_mode}")
    # https://brainco.yuque.com/tykrbo/hws0nr/pynh5qnmfa1bgamc#EyHBW

    finger_id = libstark.FingerId.Middle

    # 设置手指参数，最大角度，最小角度，最大速度，最大电流，保护电流, 各个手指参数范围详见文档
    # await client.set_finger_min_position(slave_id, finger_id, 0)
    # min_position = await client.get_finger_min_position(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] min position: {min_position}")

    # await client.set_finger_max_position(slave_id, finger_id, 80)
    # max_position = await client.get_finger_max_position(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max position: {max_position}")

    # await client.set_finger_max_speed(slave_id, finger_id, 130)
    # max_speed = await client.get_finger_max_speed(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max speed: {max_speed}")

    # await client.set_finger_max_current(slave_id, finger_id, 1000)
    # max_current = await client.get_finger_max_current(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] max current: {max_current}")

    # await client.set_finger_protect_current(slave_id, finger_id,  500)
    # protected_current = await client.get_finger_protect_current(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] protected current: {protected_current}")

    # 单个手指，按速度/电流/PWM控制
    # 其中符号表示方向，正表示为握紧方向，负表示为松开方向
    # await client.set_finger_speed(slave_id, finger_id, 500)  # -1000 ~ 1000
    # await asyncio.sleep(1.0) # 等待手指到达目标位置
    # await client.set_finger_current(slave_id, finger_id, -300)  # -1000 ~ 1000
    # await asyncio.sleep(1.0) # 等待手指到达目标位置
    # await client.set_finger_pwm(slave_id, finger_id, 700)  # -1000 ~ 1000
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 多个手指，按速度/电流/PWM控制
    # 其中符号表示方向，正表示为握紧方向，负表示为松开方向
    # await client.set_finger_speeds(slave_id, [500] * 6)
    # await asyncio.sleep(1.0) # 等待手指到达目标位置
    # await client.set_finger_currents(slave_id, [-300] * 6)
    # await asyncio.sleep(1.0) # 等待手指到达目标位置
    # await client.set_finger_pwms(slave_id, [700] * 6)
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 单个手指，按位置+速度/期望时间，无符号
    # await client.set_finger_position_with_millis(slave_id, finger_id, 1000, 1000)
    # await asyncio.sleep(1.0) # 等待手指到达目标位置
    # await client.set_finger_position_with_speed(slave_id, finger_id, 1, 50)
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 多个手指，按位置+速度/期望时间，无符号
    positions = [500] * 6
    durations = [300] * 6
    await client.set_finger_positions_and_durations(slave_id, positions, durations)
    await asyncio.sleep(1.0) # 等待手指到达目标位置
    positions = [1] * 6
    speeds = [500] * 6
    await client.set_finger_positions_and_speeds(slave_id, positions, speeds)

    logger.debug("get_motor_status")  # 获取手指状态, 位置，电流，motor状态
    status = await client.get_motor_status(slave_id)
    # logger.info(f"positions: {list(status.positions)}")
    # logger.info(f"currents: {list(status.currents)}")
    # logger.info(f"states: {list(status.states)}")
    logger.info(f"Finger status: {status.description}")

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
