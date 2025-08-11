import asyncio
import sys
from edu_utils import *


# Main
async def main():
    libstark.init_config(libstark.StarkHardwareType.Revo1Basic)
    # slave_id = 1 # 默认设备ID
    # baudrate = libstark.Baudrate.Baud460800
    # baudrate = libstark.Baudrate.Baud115200 # 默认波特率
    port_name = get_stark_port_name()
    if port_name is None:
        return

    port_name = None
    # 自动检测第一个可用从机
    (protocol, port_name, baudrate, slave_id) = await libstark.auto_detect_device(port_name, True)
    client = await libstark.modbus_open(port_name, baudrate)

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device Firmware: {device_info.firmware_version}")  # 固件版本
    logger.info(f"Device info: {device_info.description}")

    # await client.set_finger_positions(slave_id, [60, 60, 100, 100, 100, 100])  # 握拳
    # await asyncio.sleep(1)
    # await client.set_finger_positions(slave_id, [0] * 6)  # 张开手指

    await client.run_action_sequence(slave_id, libstark.ActionSequenceId.DefaultGestureFist)
    await asyncio.sleep(2)  # 等待动作序列执行完成
    await client.run_action_sequence(slave_id, libstark.ActionSequenceId.DefaultGestureOpen)

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
