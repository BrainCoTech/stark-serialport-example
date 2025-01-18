#!/usr/bin/env python
import os
import asyncio
from canopen_utils import *

filename = os.path.basename(__file__).split(".")[0]
SKLog.apply_logging_config(logging.INFO, log_file_name=f"{filename}.log")

# 定期获取手指状态
async def get_touch_status_periodically(node):
    SKLog.info("get_touch_status_periodically start")
    index = 0
    while True:
        # 获取手指状态
        try:
            SKLog.debug("get_touch_data")
            data = get_touch_data(node)
            SKLog.info(f"[{index}] Touch data: {data}") 
            index += 1
        except Exception as e:
            SKLog.error(f"Error getting touch status: {e}")
        await asyncio.sleep(0.002)

async def main():
    setup_shutdown_event()

    network = canopen_connect(interface='kvaser', channel=0, bitrate=1000000, search_limit=3)
    node = add_node_to_network(network, node_id=1)

    data = get_touch_data(node)
    SKLog.info(f"Touch data: {data}") 
    # asyncio.create_task(get_touch_status_periodically(node))

    close_canopen(network)
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
