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

    # get device info
    logger.debug("get_device_info")
    device_info: libstark.DeviceInfo = await ctx.get_device_info(slave_pos)
    logger.info(f"Device info: {device_info.description}")

    # Control mode: normalized (per mille) / physical units
    logger.debug("set_finger_unit_mode")  # Set finger control unit mode
    await ctx.set_finger_unit_mode(slave_pos, libstark.FingerUnitMode.Normalized) # Normalized mode (per mille)
    # await ctx.set_finger_unit_mode(slave_pos, libstark.FingerUnitMode.Physical) # Physical units mode
    # Get finger control mode
    logger.debug("get_finger_unit_mode")
    finger_unit_mode = await ctx.get_finger_unit_mode(slave_pos)
    logger.info(f"Finger unit mode: {finger_unit_mode}")

    # Set/Get protected current, refer to documentation for parameter ranges
    finger_id = libstark.FingerId.Middle
    # await ctx.set_finger_protected_current(slave_pos, finger_id,  500)
    protected_current = await ctx.get_finger_protected_current(slave_pos, finger_id)
    logger.info(f"Finger[{finger_id}] protected current: {protected_current}")

    turbo_conf = await ctx.get_turbo_config(slave_pos)
    logger.info(f"Turbo config: {turbo_conf.description}")

    if ctx.is_touch_hand(slave_pos):
      touch_fw_versions = await ctx.get_touch_sensor_fw_versions(slave_pos)
      logger.info(f"Touch Fw Versions: {touch_fw_versions}")

      if ctx.is_touch_pressure(slave_pos):
        data_type = await ctx.get_modulus_touch_data_type(slave_pos)
        logger.info(f"Modulus Touch Data Type: {data_type}")

      # Touch sensor parameter - zero-drift calibration
      # When the 3D force values in idle state are not zero, you can calibrate using this command
      # This command takes a long time to execute, and data during execution cannot be used as reference
      # It is recommended to ignore data within ten seconds after calibration; finger sensors must not be under force during execution
      # await ctx.touch_sensor_calibrate(slave_pos, 0x1f)  # Calibrate all/specified channels

      # Touch sensor parameter adjustment
      # await ctx.touch_sensor_reset(slave_pos, 0x1f)  # Adjust all/specified channels


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
