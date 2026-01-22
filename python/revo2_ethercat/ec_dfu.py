
import asyncio
import sys
import pathlib
from ec_utils import *

current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
sys.path.append(str(parent_dir))

# Main
async def main():
    # fmt: off
    master_pos = 0
    ctx = libstark.PyDeviceContext.open_ethercat_master(master_pos)
    slave_pos = 0
    await ctx.ec_setup_sdo(slave_pos)

    import os
    # ota_bin_path = os.path.join(
    #     parent_dir,
    #     "ota_bin",
    #     "ethercat",
    #     "wrist",
    #     "Revo2EC_0.0.1_2507011423.bin",
    # )
    ota_bin_path = os.path.join(
        parent_dir,
        "ota_bin",
        "ethercat",
        "control",
        "Revo2_V1.0.8.F_2509191719.bin",
    )

    logger.info("DFU started, waiting for completion...") # Wait for DFU to complete, about 25 seconds
    # await ctx.ec_start_dfu(slave_pos, libstark.EtherCATFoeType.Wrist, ota_bin_path)
    await ctx.ec_start_dfu(slave_pos, libstark.EtherCATFoeType.Control, ota_bin_path)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
