#!/usr/bin/env python
import os
import asyncio
from canopen_utils import *

filename = os.path.basename(__file__).split(".")[0]
SKLog.apply_logging_config(LogLevel.info, log_file_name=f"{filename}.log")

# 定期获取手指状态
async def get_finger_status_periodically(node):
    SKLog.info("get_finger_status_periodically start")
    index = 0
    while True:
        # 获取手指状态
        try:
            SKLog.debug("get_finger_positions")
            positions = get_finger_positions(node)
            if is_finger_closed(positions):
                set_finger_positions(node, [0] * 6)  # 张开
            elif is_finger_opened(positions):
                set_finger_positions(node, [60, 60, 100, 100, 100, 100])  # 握手
            index += 1
        except Exception as e:
            SKLog.error(f"Error getting finger status: {e}")
        await asyncio.sleep(0.002)

async def main():
    setup_shutdown_event()

    # Connect to CANopen node
    # network, node = connect_to_canopen_node(node_id=1, interface='socketcan', channel='can0')
    network, node = connect_to_canopen_node(node_id=1, interface='kvaser', channel=0, bitrate=250000)

    get_finger_position(node, FingerId.index, wait_for_reception=True)
    get_finger_positions(node)

    # set_finger_position(node, FingerId.index, 100)
    # set_finger_positions(node, [0] * 6)

    asyncio.create_task(get_finger_status_periodically(node))

    close_canopen(network)
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
