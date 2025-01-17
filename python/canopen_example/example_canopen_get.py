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
            SKLog.info(f"[{index}] finger positions: {positions}")
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
    network, node = connect_to_canopen_node(node_id=1, interface='kvaser', channel=0, bitrate=1000000)

    # set_finger_position(node, FingerId.thumb, 0)
    set_finger_positions(node, [0] * 6)

    # position = get_finger_position(node, FingerId.index, wait_for_reception=True)
    # SKLog.info(f"index finger position: {position}")
    # positions = get_finger_positions(node)
    # SKLog.info(f"finger positions: {positions}")

    # asyncio.create_task(get_finger_status_periodically(node))

    await asyncio.sleep(100)
    close_canopen(network)
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
