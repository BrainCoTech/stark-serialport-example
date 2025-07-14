import asyncio
import sys
import math
import time
from revo2_utils import get_stark_port_name, libstark, logger
import matplotlib.pyplot as plt

TRAJ_LEN = 200  # 轨迹点数
CTRL_INTERVAL = 0.005  # 控制间隔 5ms (200Hz)
TEST_DURATION = 4.0  # 测试时长 4秒

# 1. 初始化余弦轨迹
def init_trajectory():
    traj = []
    for i in range(TRAJ_LEN):
        value = 50.0 + 50.0 * math.cos(2 * math.pi * i / TRAJ_LEN)  # [0,100]
        traj.append(int(value * 10))  # 转换为0.1%精度 [0,1000]
    return traj

plt.figure(figsize=(12, 6))
plt.xlabel('Time (s)')
plt.ylabel('Position (%)')
plt.title('Motor Response Performance')
plt.legend()
plt.grid(True)

# Main
async def main():
    # fmt: off
    port_name = '/dev/ttyUSB0'
    # port_name = get_stark_port_name()
    if port_name is None:
        return

    slave_id = 0x7e  # 左手默认ID为0x7e，右手默认ID为0x7f
    # client = await libstark.modbus_open(port_name, libstark.Baudrate.Baud2Mbps)
    client = await libstark.modbus_open(port_name, libstark.Baudrate.Baud460800) # 默认波特率460800

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    # 控制模式：千分比模式/物理量模式
    logger.debug("set_finger_unit_mode")  # 设置手指控制参数的单位模式
    await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized) # 千分比模式
    # await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Physical) # 物理量模式
    # 获取手指控制模式
    logger.debug("get_finger_unit_mode")
    finger_unit_mode = await client.get_finger_unit_mode(slave_id)
    logger.info(f"Finger unit mode: {finger_unit_mode}")
    # https://www.brainco-hz.com/docs/revolimb-hand/protocol/stark_v2.html#%E5%8D%95%E4%BD%8D%E6%A8%A1%E5%BC%8F%E8%AE%BE%E7%BD%AE-937

    trajectory = init_trajectory()
    traj_len = len(trajectory)

    target_positions = []
    measured_positions = []
    timestamps = []

    index = 0
    start_time = time.perf_counter()

    stop_flag = False

    finger_id = libstark.FingerId.Ring  # 控制的手指，这里选择无名指

    # 2.1 控制任务（200Hz发送目标）
    async def control_task():
        nonlocal index, stop_flag
        while not stop_flag:
            start_time = time.perf_counter()
            index = (index + 1) % traj_len
            target = trajectory[index]
            await client.set_finger_position_with_millis(slave_id, finger_id, target, 1) # 1ms, 固件将以最快速度响应
            elapsed = time.perf_counter() - start_time
            logger.debug(f"[{index}] Target: {target}, Elapsed: {elapsed:.3f}s")
            if elapsed < CTRL_INTERVAL:
                await asyncio.sleep(CTRL_INTERVAL - elapsed)

    # # 2.2 读取任务（尽可能高频读取当前位置）
    async def read_task():
        nonlocal stop_flag
        while not stop_flag:
            try:
                status = await client.get_motor_status(slave_id)
                pos = status.positions[4]  # 读取Finger Ring位置

                current_time = time.perf_counter() - start_time
                target = trajectory[index]
                measured = pos
                # logger.debug(f"Time: {current_time:.3f}s, Target: {target}, Measured: {measured}")
                # logger.debug(f"Time: {current_time:.3f}s, Target: {target}, Measured: {status.positions}")

                if current_time > 0.8:  # 只记录0.8秒之后的数据
                    target_positions.append(target)
                    measured_positions.append(measured)
                    timestamps.append(current_time)

            except Exception as e:
                logger.error(f"Read error: {e}")

            await asyncio.sleep(0.001)  # 尽可能快，但防止过载

    # 2.3 同时启动
    control = asyncio.create_task(control_task())
    reader = asyncio.create_task(read_task())

    await asyncio.sleep(TEST_DURATION)
    stop_flag = True

    await control
    await reader

    # 3. 绘图分析
    plt.plot(timestamps, target_positions, label='Target Position (0~1000)') # , linestyle='--')
    plt.plot(timestamps, measured_positions, label='Measured Position (0~1000)', alpha=0.7)
    plt.show()
    plt.savefig("motor_response.png")
    logger.info("Test completed, timestamps length: %d", len(timestamps))

    # 数据保存到文件
    with open("motor_response_data.txt", "w") as f:
        f.write("Time (s), Target Position, Measured Position\n")
        for t, target, measured in zip(timestamps, target_positions, measured_positions):
            f.write(f"{t:.3f}, {target}, {measured}\n")

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)

if __name__ == "__main__":
    asyncio.run(main())
