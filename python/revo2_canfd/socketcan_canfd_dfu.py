import asyncio
import os
import pathlib
import platform
import sys
import time

from canfd_utils import *

if platform.system() != "Linux":
    raise NotImplementedError("SocketCAN is only supported on Linux.")

from socketcan_linux_utils import *

# Firmware upgrade file path
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
logger.info(f"parent_dir: {parent_dir}")

# ModbusV2 firmware
ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "stark2",
    "stark2_fw_V0.0.14_20250723135853.bin",
)

if not os.path.exists(ota_bin_path):
    logger.warning(f"OTA firmware file not found: {ota_bin_path}")
    sys.exit(0)
else:
    logger.info(f"OTA firmware file path: {ota_bin_path}")

shutdown_event = None
main_loop = None


def on_dfu_state(_slave_id, state):
    logger.info(f"DFU STATE: {libstark.DfuState(state)}")
    dfu_state = libstark.DfuState(state)
    if (
        dfu_state == libstark.DfuState.Completed
        or dfu_state == libstark.DfuState.Aborted
    ):
        if main_loop and shutdown_event:
            if not shutdown_event.is_set():
                logger.info("Using call_soon_threadsafe to set event")
                main_loop.call_soon_threadsafe(shutdown_event.set)


def on_dfu_progress(_slave_id, progress):
    logger.info(f"progress: {progress * 100.0 :.2f}%")


def canfd_send(_slave_id: int, can_id: int, data: list):
    if not socketcan_send_message(can_id, bytes(data)):
        logger.error("Failed to send CANFD message")
        return


def canfd_read(_slave_id: int):
    recv_msg = socketcan_receive_message()
    if recv_msg is None:
        logger.error("No message received")
        return 0, bytes([])

    (can_id, data) = recv_msg
    logger.debug(f"Received CAN ID: {can_id:029b}, Data: {bytes(data).hex()}")
    return can_id, data


def exception_hook(exc_type, exc_value, exc_traceback):
    import traceback

    error_msg = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
    print(f"Error occurred:\n{error_msg}")

    with open("error_log.txt", "a", encoding="utf-8") as f:
        f.write(f"\n--- {os.path.basename(__file__)} Error ---\n")
        f.write(error_msg)
        f.write("\n--- Error end ---\n")

    sys.__excepthook__(exc_type, exc_value, exc_traceback)


async def main():
    sys.excepthook = exception_hook
    global shutdown_event, main_loop
    main_loop = asyncio.get_running_loop()
    shutdown_event = setup_shutdown_event(logger)

    master_id = int(os.getenv("STARK_MASTER_ID", "1"), 0)
    slave_id = int(os.getenv("STARK_SLAVE_ID", "0x7f"), 0)
    client = libstark.PyDeviceContext.init_canfd(master_id)

    socketcan_open()
    libstark.set_can_tx_callback(canfd_send)
    libstark.set_can_rx_callback(canfd_read)

    logger.debug("get_device_info")
    device_info = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    baudrate = await client.get_canfd_baudrate(slave_id)
    logger.info(f"CANFD, Baudrate: {baudrate}")

    try:
        logger.info("start_dfu")
        start_time = time.perf_counter()
        wait_seconds = 5
        await client.start_dfu(
            slave_id,
            ota_bin_path,
            wait_seconds,
            on_dfu_state,
            on_dfu_progress,
        )
        logger.info(f"DFU finished in {time.perf_counter() - start_time:.2f}s")
    except Exception as e:
        logger.error(f"DFU error: {e}")

    await shutdown_event.wait()
    logger.info("Shutdown event received, stopping...")
    socketcan_close()


if __name__ == "__main__":
    asyncio.run(main())
