import asyncio
import sys
from stark_utils import get_stark_port_name, libstark, logger


async def set_slave_id(client, slave_id, new_slave_id):
    await client.set_slave_id(slave_id, new_slave_id)
    logger.info(f"write success, wait 3s for device reboot")
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    logger.info(
        f"New Slave ID: {new_slave_id}, please reconnect the device with new Slave ID"
    )
    sys.exit(0)


async def set_baudrate(client, slave_id, new_baudrate):
    baudrate = await client.get_serialport_baudrate(slave_id)
    logger.info(f"Baudrate: {baudrate}")
    await client.set_serialport_baudrate(slave_id, new_baudrate)
    logger.info(f"write success, wait 3s for device reboot")
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    logger.info(
        f"New Baudrate: {new_baudrate}, please reconnect the device with new Baudrate"
    )
    sys.exit(0)


# Main
async def main():
    # libstark.init_config(libstark.StarkFirmwareType.V1Standard)
    port_name = get_stark_port_name()
    if port_name is None:
        return
    slave_id = 1  # default slave_id is 1
    baudrate = libstark.Baudrate.Baud115200
    client = await libstark.modbus_open(port_name, baudrate, slave_id)

    logger.debug("get_serialport_cfg")  # 获取串口配置, slave_id, 波特率
    serialport_cfg = await client.get_serialport_cfg(slave_id)
    logger.info(f"Slave ID: {serialport_cfg.description}")

    # logger.debug("set_slave_id")  # 修改slave_id，设置后，会执行重启操作
    # await set_slave_id(client, slave_id, new_slave_id=1) # 修改slave_id为1
    # await set_slave_id(client, slave_id, new_slave_id=2) # 修改slave_id为2
    # exit(0)

    # logger.debug("set_baudrate")  # 修改波特率，设置后，会执行重启操作
    # await set_baudrate(client, slave_id, new_baudrate=libstark.Baudrate.Baud115200) # 修改波特率为115200
    await set_baudrate(client, slave_id, new_baudrate=libstark.Baudrate.Baud460800)  # 修改波特率为460800
    exit(0)

    # ------------------- 设置力量等级，大-中-小 -------------------
    # 触觉版本废弃了该功能
    # logger.debug(f"set_force_level")
    # # level = await client.get_force_level()
    # # logger.info(f"Force level: {level}")
    # await client.set_force_level(slave_id, force_level=libstark.ForceLevel.Full)
    # await client.set_force_level(slave_id, force_level=libstark.ForceLevel.Normal)
    # # await client.set_force_level(slave_id, force_level=libstark.ForceLevel.Small)
    # level = await client.get_force_level(slave_id)
    # logger.info(f"Force level: {level}")

    # ------------------- 设置Turbo模式及参数 -------------------
    logger.debug(f"set_turbo_mode")
    await client.set_turbo_mode_enabled(slave_id, False)
    turbo_mode = await client.get_turbo_mode_enabled(slave_id)
    logger.info(f"Turbo mode: {turbo_mode}")

    logger.debug(f"set_turbo_conf")
    turbo_interval = 2000
    turbo_duration = 3000
    turbo_conf = libstark.TurboConfig(turbo_interval, turbo_duration)
    await client.set_turbo_config(slave_id, turbo_conf)
    turbo_conf = await client.get_turbo_config(slave_id)
    logger.info(f"Turbo conf: {turbo_conf.description}")

    # ------------------- 位置校准设置 -------------------
    # 设置是否在上电后自动执行位置校准
    # await client.set_position_auto_calibration(slave_id, True) # 设置后，会执行重启操作
    calibration_enabled = await client.get_auto_calibration_enabled(slave_id)
    logger.info(f"Auto calibration enabled: {calibration_enabled}")

    # await client.set_finger_positions(slave_id, [60, 60, 100, 100, 100, 100]) # 握手
    # 位置校准
    await client.calibrate_position(slave_id)

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
