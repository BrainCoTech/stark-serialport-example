import asyncio
import sys
from stark_utils import get_stark_port_name, libstark, logger
import bc_stark_sdk
from collections import OrderedDict

# 定义手指名称和每个手指的传感器点数量
FINGER_CONFIG = OrderedDict([
    ("thumb", 2),   # 拇指: 2个传感器点
    ("index", 3),   # 食指: 3个传感器点
    ("middle", 3),  # 中指: 3个传感器点
    ("ring", 3),    # 无名指: 3个传感器点
    ("pinky", 2)    # 小指: 2个传感器点
])

def format_touch_sensor_data(slave_id, items):
    # 检查items长度是否正确（总共13个传感器点）
    expected_length = sum(FINGER_CONFIG.values())  # 2 + 3 + 3 + 3 + 2 = 13
    if len(items) != expected_length:
        logger.error(f"[{slave_id}] 触觉传感器数据长度错误，期望 {expected_length}，实际 {len(items)}")
        return

    # 按照手指分组数据
    sensor_data = {}
    current_index = 0
    for finger, count in FINGER_CONFIG.items():
        # 提取当前手指的传感器点
        finger_data = []
        for i in range(count):
            try:
                item = items[current_index + i]
                # 假设item有state和force属性
                finger_data.append({
                    "state": item.state,  # 状态，如NoContact
                    "force": f"{item.force:.2f}"  # 力值，保留2位小数
                })
            except (AttributeError, IndexError) as e:
                logger.error(f"[{slave_id}] 解析 {finger} 数据失败: {e}")
                return
        sensor_data[finger] = finger_data
        current_index += count

    # 格式化输出
    output_lines = [f"[{slave_id}] 触觉传感器数据:"]
    for finger, data in sensor_data.items():
        output_lines.append(f"  {finger}: [")
        for point in data:
            output_lines.append(f"    {{state: {point['state']}, force: {point['force']}}},")
        output_lines.append("  ],")
    formatted_output = "\n".join(output_lines)
    logger.info(formatted_output)

# Main
async def main():
    libstark.init_config(libstark.StarkFirmwareType.V1Touch)
    port_name = get_stark_port_name()
    if port_name is None:
        return
    slave_id = 1
    # baudrate = libstark.Baudrate.Baud460800
    baudrate = libstark.Baudrate.Baud115200
    client = await libstark.modbus_open(port_name, baudrate)

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device Firmware: {device_info.firmware_version}")  # 固件版本
    logger.info(f"Device info: {device_info.description}")

    # 启用触觉传感器功能
    bits = 0x1F  # 0x1f: 5个手指上的触觉传感器都使能
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)  # // wait for touch sensor to be ready
    touch_fw_versions = await client.get_touch_sensor_fw_versions(slave_id)  # setup之后才能获取到版本号
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    # 设置触觉传感器回调
    bc_stark_sdk.set_touch_sensor_callback(format_touch_sensor_data)
    interval_read = 0 # 读取触觉传感器的间隔，单位毫秒
    interval_cb = 1   # 检测间隔：1表示每次都执行，2表示每隔一次执行一次，依此类推
    await client.start_touch_sensor_stream(slave_id, interval_read, interval_cb) # 开始读取触觉传感器数据，检测是否接触及滑移
    await asyncio.sleep(1000)
    await client.stop_touch_sensor_stream(slave_id) # 停止读取触觉传感器数据
    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
