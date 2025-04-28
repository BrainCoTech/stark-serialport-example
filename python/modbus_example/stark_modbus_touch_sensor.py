import asyncio
import sys
from stark_utils import get_stark_port_name, libstark, logger


# Main
async def main():
    libstark.init_config(libstark.StarkFirmwareType.V1Touch)
    port_name = get_stark_port_name()
    if port_name is None:
        return
    slave_id = 1
    client = await libstark.modbus_open(
        port_name, libstark.Baudrate.Baud460800, slave_id
    )

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device Firmware: {device_info.firmware_version}")  # 固件版本
    logger.info(f"Device info: {device_info.description}")

    # 启用触觉传感器功能
    bits = 0x1F  # 0x1f: 5个手指上的触觉传感器都使能
    await client.touch_sensor_setup(slave_id, bits)
    await asyncio.sleep(1)  # // wait for touch sensor to be ready
    bits = await client.get_touch_sensor_enabled(slave_id)
    logger.info(f"Touch Sensor Enabled: {(bits & 0x1F):05b}")
    touch_fw_versions = await client.get_touch_sensor_fw_versions(
        slave_id
    )  # setup之后才能获取到版本号
    logger.info(f"Touch Fw Versions: {touch_fw_versions}")

    # 获取触觉传感器状态、三维力数值、通道值
    while True:
        touch_status = await client.get_touch_sensor_status(slave_id)
        logger.info(f"Touch Sensor Status: {touch_status[0].description}")
        logger.debug(f"Thumb normal_force1: {touch_status[0].normal_force1}")
        logger.debug(f"Index self_proximity1: {touch_status[1].self_proximity1}")
        logger.debug(f"Index self_proximity2: {touch_status[1].self_proximity2}")
        logger.debug(f"Index mutual_proximity: {touch_status[1].mutual_proximity}")
        touch_raw_data = await client.get_touch_sensor_raw_data(slave_id)
        logger.info(f"Touch Sensor Raw Data: {touch_raw_data.description}")
        logger.debug(f"Touch Sensor Raw Data Thumb: {touch_raw_data.thumb}")
        await asyncio.sleep(0.05)
        break

    # 触觉传感器复位，发送传感器采集通道复位指令。在执行该指令时，手指传感器尽量不要受力。
    # await client.touch_sensor_reset(slave_id, 0x1f)  # 复位指定手指的传感器采集通道

    # 触觉传感器参数校准，参数校准指令。 当空闲状态下的三维力数值不为0时，可通过该寄存器进行校准。该指令的执行时间较长，期间的数据无法作为参考。建议忽略在该参数校准寄存器设置后十秒内的数据。在执行该指令时，手指传感器不能受力， 否则将导致校准后的传感器数据异常。
    # await client.touch_sensor_calibrate(slave_id, 0x1f)  # 校准指定手指的传感器采集通道

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
