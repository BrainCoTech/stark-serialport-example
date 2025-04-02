import asyncio
import sys
from stark_utils import get_stark_port_name, libstark, logger


# Main
async def main():
    libstark.init_config(libstark.StarkFirmwareType.V1Standard)
    port_name = get_stark_port_name()
    if port_name is None:
        return
    slave_id = 1
    client = await libstark.modbus_open(
        port_name, libstark.Baudrate.Baud115200, slave_id
    )

    logger.debug("get_serialport_cfg")  # 获取串口配置, 波特率
    baudrate = await client.get_serialport_baudrate(slave_id)
    logger.info(f"Baudrate: {baudrate}")

    logger.debug("get_device_info")  # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.firmware_version}")  # 固件版本
    logger.info(f"Device info: {device_info.serial_number}")  # 序列号
    logger.info(f"Device info: {device_info.sku_type}")  # 获取手类型, 左右手
    logger.info(f"Device info: {device_info.description}")

    # 触觉版本废弃了该功能
    # logger.debug("get_force_level")  # 获取力量等级，大-中-小
    # force_level = await client.get_force_level(slave_id)
    # logger.info(f"Force level: {force_level}")

    logger.debug("get_voltage")  # 获取电量
    voltage = await client.get_voltage(slave_id)
    logger.info(f"Voltage: {voltage:.1f} mV")

    logger.debug("get_motor_status")  # 获取手指状态, 位置，电流，motor状态
    status = await client.get_motor_status(slave_id)
    logger.info(f"positions: {list(status.positions)}")
    logger.info(f"states: {list(status.states)}")
    logger.info(f"Finger status: {status.description}")

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
