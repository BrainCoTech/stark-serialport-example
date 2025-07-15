import asyncio
import sys
from revo2_utils import libstark, logger, open_modbus_revo2


# Main
async def main():
    # fmt: off
    (client, slave_id) = await open_modbus_revo2()

    # 控制模式：千分比模式/物理量模式
    logger.debug("set_finger_unit_mode")  # 设置手指控制参数的单位模式
    await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized) # 千分比模式
    # await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Physical) # 物理量模式
    # 获取手指控制模式
    logger.debug("get_finger_unit_mode")
    finger_unit_mode = await client.get_finger_unit_mode(slave_id)
    logger.info(f"Finger unit mode: {finger_unit_mode}")
    # https://www.brainco-hz.com/docs/revolimb-hand/protocol/stark_v2.html#%E5%8D%95%E4%BD%8D%E6%A8%A1%E5%BC%8F%E8%AE%BE%E7%BD%AE-937

    finger_id = libstark.FingerId.Middle

    # 设置手指参数，最大角度，最小角度，最大速度，最大电流，保护电流, 各个手指参数范围详见文档
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
    # await client.set_finger_protected_current(slave_id, finger_id,  500)
    # protected_current = await client.get_finger_protected_current(slave_id, finger_id)
    # logger.info(f"Finger[{finger_id}] protected current: {protected_current}")

    # 控制单个手指，速度控制，其中符号表示方向，正表示为握紧方向，负表示为松开方向
    # await client.set_finger_speed(slave_id, finger_id, 500)  # -1000 ~ 1000
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 控制单个手指，电流控制，其中符号表示方向，正表示为握紧方向，负表示为松开方向
    # await client.set_finger_current(slave_id, finger_id, -300)  # -1000 ~ 1000
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 控制单个手指，PWM控制，其中符号表示方向，正表示为握紧方向，负表示为松开方向
    # await client.set_finger_pwm(slave_id, finger_id, 700)  # -1000 ~ 1000
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 控制全部手指，速度控制，其中符号表示方向，正表示为握紧方向，负表示为松开方向
    # await client.set_finger_speeds(slave_id, [500] * 6)
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 控制全部手指，电流控制，其中符号表示方向，正表示为握紧方向，负表示为松开方向
    # await client.set_finger_currents(slave_id, [-300] * 6)
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 控制全部手指，PWM控制，其中符号表示方向，正表示为握紧方向，负表示为松开方向
    # await client.set_finger_pwms(slave_id, [700] * 6)
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 控制单个手指，目标位置+期望时间
    # await client.set_finger_position_with_millis(slave_id, finger_id, 1000, 1000)
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 控制单个手指，目标位置+期望速度
    # await client.set_finger_position_with_speed(slave_id, finger_id, 1, 50)
    # await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 控制全部手指，目标位置+期望时间
    positions = [500] * 6
    durations = [300] * 6
    await client.set_finger_positions_and_durations(slave_id, positions, durations)
    await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 控制全部手指，目标位置+期望速度
    positions = [1000] * 6
    speeds = [500] * 6
    await client.set_finger_positions_and_speeds(slave_id, positions, speeds)
    await asyncio.sleep(1.0)  # 等待手指到达目标位置

    # 获取手指状态, 位置，电流，motor状态
    logger.debug("get_motor_status")
    status = await client.get_motor_status(slave_id)
    # logger.info(f"positions: {list(status.positions)}") # 位置
    # logger.info(f"positions: {list(status.speeds)}") # 速度
    # logger.info(f"currents: {list(status.currents)}") # 电流
    # logger.info(f"states: {list(status.states)}") # 状态
    logger.info(f"Finger status: {status.description}")

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
