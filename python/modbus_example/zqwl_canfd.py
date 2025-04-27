import asyncio
import sys
# import time
import bc_stark_sdk
from stark_utils import libstark, logger
# from zlgcan import ZCAN_USBCANFD_100U
from zqwl_win import zcan_open, zcan_close, zcan_send_message, zcan_read_messages

def canfd_send(slave_id: int, can_id: int, data: list):
    # logger.debug(f"Sending CAN ID: {can_id}, Data: {data}, type: {type(data)}")
    if not zcan_send_message(slave_id, can_id, bytes(data)):
        logger.error("Failed to send CANFD message")
        return

def canfd_read(slave_id: int):
    recv_msg = zcan_read_messages()
    if recv_msg is None:
        logger.error("No message received")
        return 0, bytes([])

    can_id = recv_msg.frame.can_id
    data = [recv_msg.frame.data[i] for i in range(recv_msg.frame.len)]
    logger.debug(f"Received CAN ID: {can_id:029b}, Data: {bytes(data).hex()}")
    return can_id, data

# Main
async def main():
    # fmt: off
    libstark.init_config(libstark.StarkFirmwareType.V2Standard, libstark.StarkProtocolType.CanFd)
    slave_id = 0x7e # 左手默认ID为0x7e，右手默认ID为0x7f
    client = await libstark.canfd_open(libstark.BaudrateCAN.Baud5Mbps, slave_id)
    
    # ZCAN_USBCANFD_100U
    zcan_open(device_type=42, channel=0, baudrate=5000000)
    bc_stark_sdk.set_canfd_tx_callback(canfd_send)
    bc_stark_sdk.set_canfd_rx_callback(canfd_read)

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD, Baudrate: {baudrate}")

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    # logger.info(f"Device info: {device_info.firmware_version}")  # 固件版本
    # logger.info(f"Device info: {device_info.serial_number}")  # 序列号
    # logger.info(f"Device info: {device_info.sku_type}")  # 获取手类型, 左右手
    logger.info(f"Device info: {device_info.description}")

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
    await client.set_finger_min_position(slave_id, finger_id, 0)
    min_position = await client.get_finger_min_position(slave_id, finger_id)
    logger.info(f"Finger[{finger_id}] min position: {min_position}")

    await client.set_finger_max_position(slave_id, finger_id, 80)
    max_position = await client.get_finger_max_position(slave_id, finger_id)
    logger.info(f"Finger[{finger_id}] max position: {max_position}")

    await client.set_finger_max_speed(slave_id, finger_id, 130)
    max_speed = await client.get_finger_max_speed(slave_id, finger_id)
    logger.info(f"Finger[{finger_id}] max speed: {max_speed}")

    await client.set_finger_max_current(slave_id, finger_id, 1000)
    max_current = await client.get_finger_max_current(slave_id, finger_id)
    logger.info(f"Finger[{finger_id}] max current: {max_current}")

    await client.set_finger_protect_current(slave_id, finger_id,  500)
    protected_current = await client.get_finger_protect_current(slave_id, finger_id)
    logger.info(f"Finger[{finger_id}] protected current: {protected_current}")

    # 单个手指，按速度/电流/PWM控制
    # 其中符号表示方向，正表示为握紧方向，负表示为松开方向
    await client.set_finger_speed(slave_id, finger_id, 500)  # -1000 ~ 1000
    await asyncio.sleep(1.0) # 等待手指到达目标位置
    await client.set_finger_current(slave_id, finger_id, -300)  # -1000 ~ 1000
    await asyncio.sleep(1.0) # 等待手指到达目标位置
    await client.set_finger_pwm(slave_id, finger_id, 700)  # -1000 ~ 1000
    await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 多个手指，按速度/电流/PWM控制
    # 其中符号表示方向，正表示为握紧方向，负表示为松开方向
    await client.set_finger_speeds(slave_id, [500] * 6)
    await asyncio.sleep(1.0) # 等待手指到达目标位置
    await client.set_finger_currents(slave_id, [-300] * 6)
    await asyncio.sleep(1.0) # 等待手指到达目标位置
    await client.set_finger_pwms(slave_id, [700] * 6)
    await asyncio.sleep(1.0) # 等待手指到达目标位置

    # 单个手指，按位置+速度/期望时间，无符号
    await client.set_finger_position_with_millis(slave_id, finger_id, 1000, 1000)
    await asyncio.sleep(1.0) # 等待手指到达目标位置
    await client.set_finger_position_with_speed(slave_id, finger_id, 1, 50)
    await asyncio.sleep(1.0) # 等待手指到达目标位置

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
    logger.info(f"Motor status: {status.description}")

    # 关闭资源
    zcan_close()
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
