import asyncio
import math
import time
from ec_utils import *

TRAJ_LEN = 10  # 轨迹点数
CTRL_INTERVAL = 0.1  # 控制间隔 100ms
TEST_DURATION = 400.0  # 测试时长 400秒


# 1. 初始化余弦轨迹
def init_trajectory():
    traj = []
    for i in range(TRAJ_LEN):
        value = 50.0 + 50.0 * math.cos(2 * math.pi * i / TRAJ_LEN)  # [0,100]
        traj.append(int(value * 10))  # 转换为0.1%精度 [0,1000]
    return traj


# Main
async def main():
    shutdown_event = setup_shutdown_event(logger)

    master_pos = 0
    ctx = libstark.PyDeviceContext.open_ethercat_master(master_pos)
    slave_pos = 0
    await ctx.ec_setup_sdo(slave_pos)

    # 获取设备信息
    logger.debug("get_device_info")
    device_info: libstark.DeviceInfo = await ctx.get_device_info(slave_pos)
    logger.info(f"Device info: {device_info.description}")

    # 控制模式：千分比模式/物理量模式
    logger.debug("set_finger_unit_mode")  # 设置手指控制参数的单位模式
    await ctx.set_finger_unit_mode(
        slave_pos, libstark.FingerUnitMode.Normalized
    )  # 千分比模式
    # await ctx.set_finger_unit_mode(slave_pos, libstark.FingerUnitMode.Physical) # 物理量模式
    # 获取手指控制模式
    logger.debug("get_finger_unit_mode")
    finger_unit_mode = await ctx.get_finger_unit_mode(slave_pos)
    logger.info(f"Finger unit mode: {finger_unit_mode}")
    # https://www.brainco-hz.com/docs/revolimb-hand/revo2/modbus_foundation.html#%E5%8D%95%E4%BD%8D%E6%A8%A1%E5%BC%8F%E8%AE%BE%E7%BD%AE-937

    # 设置/读取保护电流, 参数范围详见文档
    finger_id = libstark.FingerId.Ring
    # await ctx.set_finger_protected_current(slave_pos, finger_id,  500)
    protected_current = await ctx.get_finger_protected_current(slave_pos, finger_id)
    logger.info(f"Finger[{finger_id}] protected current: {protected_current}")

    trajectory = init_trajectory()
    traj_len = len(trajectory)

    index = 0
    stop_flag = False

    async def control_task():
        nonlocal index, stop_flag
        while not stop_flag:
            start_time = time.perf_counter()
            target = trajectory[index]
            index = (index + 1) % traj_len
            logger.debug(f"Target: {target}, Index: {index}")
            await ctx.set_finger_position_with_millis(
                slave_pos, finger_id, target, 1
            )  # 1ms, 固件将以最快速度响应
            elapsed = time.perf_counter() - start_time
            logger.info(
                f"[{(index - 1) % traj_len}] Target: {target}, Elapsed: {elapsed:.3f}s"
            )
            if elapsed < CTRL_INTERVAL:
                await asyncio.sleep(CTRL_INTERVAL - elapsed)

    async def read_task():
        nonlocal stop_flag
        while not stop_flag:
            try:
                logger.debug(f"get_motor_status")
                start_time = time.perf_counter()
                status = await ctx.get_motor_status(slave_pos)
                elapsed = (time.perf_counter() - start_time) * 1000
                logger.info(f"get_motor_status, elapsed:  {elapsed:.2f}ms")
                logger.debug(f"Motor status: {status.description}")

                if ctx.is_touch_hand():
                    if ctx.is_touch_pressure():
                      logger.debug("get_modulus_touch_data")
                      start_time = time.perf_counter()
                      modulus_touch_data = await ctx.get_modulus_touch_data(slave_pos)
                      elapsed = (time.perf_counter() - start_time) * 1000
                      logger.info(f"get_modulus_touch_data, elapsed:  {elapsed:.2f}ms")
                      logger.info(f"Modulus Touch Data: {modulus_touch_data}")
                      modulus_touch_summary = await ctx.get_modulus_touch_summary(slave_pos)
                      logger.info(f"Modulus Touch Summary: {modulus_touch_summary}")

                    else:
                      logger.debug("get_touch_sensor_status")
                      start_time = time.perf_counter()
                      touch_status: list[libstark.TouchFingerItem] = await ctx.get_touch_sensor_status(slave_pos)
                      elapsed = (time.perf_counter() - start_time) * 1000
                      logger.info(f"get_touch_sensor_status, elapsed:  {elapsed:.2f}ms")
                      thumb: libstark.TouchFingerItem = touch_status[0]   # 拇指
                      index: libstark.TouchFingerItem = touch_status[1]   # 食指
                      middle: libstark.TouchFingerItem = touch_status[2]  # 中指
                      ring: libstark.TouchFingerItem = touch_status[3]    # 无名指
                      pinky: libstark.TouchFingerItem = touch_status[4]   # 小指
                      logger.debug(f"Touch Sensor Status Thumb: {thumb.description}")
                      logger.debug(f"Touch Sensor Status Index: {index.description}")
                      logger.debug(f"Touch Sensor Status Middle: {middle.description}")
                      logger.debug(f"Touch Sensor Status Ring: {ring.description}")
                      logger.info(f"Touch Sensor Status Pinky: {pinky.description}")

            except Exception as e:
                logger.error(f"Read error: {e}")

            await asyncio.sleep(0.001)  # 尽可能快，但防止过载

    await ctx.ec_reserve_master()
    logger.info("EtherCAT master reserved")

    await ctx.ec_start_loop([slave_pos], 0, 1000_000, 0, 0, 0)  # 启动PDO循环Thread，不启用DC时钟同步
    control = asyncio.create_task(control_task())
    reader = asyncio.create_task(read_task())

    # 等待关闭事件
    await shutdown_event.wait()
    await asyncio.sleep(TEST_DURATION)

    await ctx.ec_stop_loop()
    stop_flag = True
    control.cancel()
    reader.cancel()
    await ctx.close()


if __name__ == "__main__":
    asyncio.run(main())
