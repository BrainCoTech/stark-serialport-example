import asyncio
import sys
from ec_utils import *

# Main
async def main():
    # fmt: off
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
    await ctx.set_finger_unit_mode(slave_pos, libstark.FingerUnitMode.Normalized) # 千分比模式
    # await ctx.set_finger_unit_mode(slave_pos, libstark.FingerUnitMode.Physical) # 物理量模式
    # 获取手指控制模式
    logger.debug("get_finger_unit_mode")
    finger_unit_mode = await ctx.get_finger_unit_mode(slave_pos)
    logger.info(f"Finger unit mode: {finger_unit_mode}")
    # https://www.brainco-hz.com/docs/revolimb-hand/revo2/modbus_foundation.html#%E5%8D%95%E4%BD%8D%E6%A8%A1%E5%BC%8F%E8%AE%BE%E7%BD%AE-937

    # 设置/读取保护电流, 参数范围详见文档
    finger_id = libstark.FingerId.Middle
    # await ctx.set_finger_protected_current(slave_pos, finger_id,  500)
    protected_current = await ctx.get_finger_protected_current(slave_pos, finger_id)
    logger.info(f"Finger[{finger_id}] protected current: {protected_current}")

    if ctx.is_touch_hand():
      touch_fw_versions = await ctx.get_touch_sensor_fw_versions(slave_pos)
      logger.info(f"Touch Fw Versions: {touch_fw_versions}")

      if ctx.is_touch_pressure():
        data_type = await ctx.get_modulus_touch_data_type(slave_pos)
        logger.info(f"Modulus Touch Data Type: {data_type}")

      # 触觉传感器参数-校准零漂
      # 当空闲状态下的三维力数值不为0时，可通过该指令进行校准
      # 该指令的执行时间较长，期间的数据无法作为参考
      # 建议忽略校准后十秒内的数据，执行时手指传感器不能受力
      # await ctx.touch_sensor_calibrate(slave_pos, 0x1f)  # 校准全部/指定通道

      # 触觉传感器参数调整
      # await ctx.touch_sensor_reset(slave_pos, 0x1f)  # 调整全部/指定通道


    # NOTE: 使用PDO通信 控制/读取马达状态、触觉等信息

    # 关闭资源
    # libstark.modbus_close(ctx)
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
