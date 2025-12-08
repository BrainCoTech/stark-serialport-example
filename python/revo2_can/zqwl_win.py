import sys
import os
from pathlib import Path

current_dir = Path(__file__).resolve().parent
parent_dir = current_dir.parent
sys.path.append(str(parent_dir))

from dll.zqwl.zqwl import *
from can_utils import logger

logger.info(f"parent_dir: {parent_dir}")

read_timeout_ms = 50

zcan = ZCAN()
zcan_handler = None
zcan_device_handler = None


def zcan_open(device_type: int, channel: int, baudrate: int):
    """
    Open standard CAN 2.0 channel

    Args:
        device_type: Device type
        channel: Channel number
        baudrate: Baud rate, default 1Mbps (supports 125K, 250K, 500K, 1M)
    """
    try:
        # open device
        dev_handler = zcan.OpenDevice(ZCAN_DEVICE_TYPE(device_type), 0, 0)
        if dev_handler == INVALID_DEVICE_HANDLE:
            logger.error("Open device failed")
            return

        # Configure standard CAN 2.0 parameters
        set_can_baudrate(dev_handler, channel, baudrate)
        zcan.ZCAN_SetValue(dev_handler, f"{channel}/filter_clear", "0")  # Clear filter
        # zcan.ZCAN_SetValue(dev_handler, f"{channel}/filter_mode", "0")               # Filter mode
        # zcan.ZCAN_SetValue(dev_handler, f"{channel}/acc_code", "0")                  # Acceptance code
        # zcan.ZCAN_SetValue(dev_handler, f"{channel}/acc_mask", "0xffffffff")         # Acceptance mask
        # zcan.ZCAN_SetValue(dev_handler, f"{channel}/brp", "1")                       # Baud rate prescaler

        # Initialize channel configuration
        chn_cfg = ZCAN_CHANNEL_INIT_CONFIG()
        chn_cfg.can_type = ZCAN_TYPE_CAN  # Standard CAN 2.0

        # Configure standard CAN parameters
        # chn_cfg.config.can.acc_code = 0x00000000      # Acceptance code
        # chn_cfg.config.can.acc_mask = 0xffffffff      # Acceptance mask (receive all frames)
        # chn_cfg.config.can.filter = 0                 # Filter settings
        chn_cfg.config.can.mode = 0  # 0: normal mode, 1: listen only mode

        can_handler = zcan.InitCAN(dev_handler, channel, chn_cfg)
        if can_handler == INVALID_CHANNEL_HANDLE:
            logger.error("Initialize channel failed!")
            zcan.CloseDevice(dev_handler)
            return

        ret = zcan.StartCAN(can_handler)
        if ret != ZCAN_STATUS_OK:
            logger.error("Open channel failed!")
            return

        global zcan_handler, zcan_device_handler
        zcan_handler = can_handler
        zcan_device_handler = dev_handler

        logger.info(f"CAN 2.0 channel opened successfully: Baud rate={baudrate}bps, Sample point=80%")

    except Exception as e:
        logger.error(f"CAN initialization exception: {str(e)}")


def zcan_close():
    """Close CAN channel and device"""
    global zcan_handler, zcan_device_handler
    if zcan_handler is not None:
        zcan.ResetCAN(zcan_handler)
        zcan_handler = None

    if zcan_device_handler is not None:
        zcan.CloseDevice(zcan_device_handler)
        zcan_device_handler = None

    logger.info("CAN device closed")


def zcan_clear_buffer():
    """Clear receive buffer"""
    if zcan_handler is not None:
        zcan.ClearBuffer(zcan_handler)


def zcan_send_message(slave_id: int, can_id: int, data: bytes):
    """
    Send standard CAN message

    Args:
        slave_id: Slave ID (not used, for compatibility)
        can_id: CAN identifier (11-bit standard frame)
        data: Data (maximum 8 bytes)
    """
    if zcan_handler is None:
        logger.error("CAN handler is None")
        return False

    # Check data length
    if len(data) > 8:
        logger.error(f"CAN 2.0 data length exceeds limit: {len(data)} bytes (maximum 8 bytes)")
        return False

    # Check CAN ID range (11-bit standard frame)
    if can_id > 0x7FF:
        logger.warning(f"CAN ID {can_id:03X} exceeds standard frame range, using extended frame")

    logger.debug(f"Send CAN message - ID: 0x{can_id:03X}, data: {data.hex()}")

    # Create standard CAN message
    msg = ZCAN_Transmit_Data()
    msg.transmit_type = 0  # 0: normal send
    msg.frame.can_id = can_id  # CAN identifier
    msg.frame.can_dlc = len(data)  # data length

    # Fill data
    for i in range(len(data)):
        msg.frame.data[i] = data[i]

    # Send message
    ret = zcan.Transmit(zcan_handler, msg, 1)
    if ret != 1:
        logger.error("Send CAN message failed!")
        return False

    logger.debug(f"Send data successfully: {ret}")
    return True


def zqwl_can_receive_message(quick_retries: int = 2, dely_retries: int = 0):
    """
    Receive CAN bus message, first try quick receive, if failed then try slow receive in DFU mode.

    Args:
        quick_retries (int): Quick receive attempt times, default 2 times.
        dely_retries (int): Slow receive attempt times in DFU mode, default 0 times.

    Returns:
        Received message or None(receive failed)
    """
    if zcan_handler is None:
        logger.error("CANFD handler is None")
        return None

    # Quick receive attempts
    for attempt in range(quick_retries):
        # time.sleep(0.0001)  # Very short delay
        time.sleep(0.02)  # Very short delay
        message = _zcan_read_messages(attempt)
        if message is not None:
            return message

    logger.warning("Quick receive timeout")

    # Slow receive attempts
    for attempt in range(dely_retries):
        time.sleep(0.5)  # Long delay, suitable for DFU mode
        message = _zcan_read_messages(attempt)
        if message is not None:
            return message

    if dely_retries > 0:
        logger.error("Delay receive also timed out!")
    return None


def _zcan_read_messages(index: int = 0):
    """Internal function: read CAN message"""
    num = zcan.GetReceiveNum(zcan_handler, ZCAN_TYPE_CAN)  # Change to standard CAN type
    if num == 0:
        logger.debug(f"No message received, attempt times: {index}")
        return None

    # Receive standard CAN message
    recv_msgs, act_num = zcan.Receive(zcan_handler, num, c_int(read_timeout_ms))
    if act_num == 0:
        logger.error("Receive CAN message failed!")
        return None

    logger.debug(f"Received {act_num} CAN messages")
    debug_text = "Received data: "
    for i in range(act_num):
        msg = recv_msgs[i]
        can_id = msg.frame.can_id
        len = msg.frame.can_dlc
        debug_text += (
            "\nline[%d], timestamp=%d, can_id=%s, err=%d, rtr=%d, eff=%d, len=%d, data: %s"
            % (
                i,
                msg.timestamp,
                "{:011b}".format(can_id),
                msg.frame.err,
                msg.frame.rtr,
                msg.frame.eff,
                len,
                " ".join("%02x" % msg.frame.data[j] for j in range(len)),
            )
        )
    logger.debug(f"Received message: %s" % debug_text)

    # When multiple messages are received, return the first message
    if act_num > 0:
        recv_msg = recv_msgs[0]
        can_id = recv_msg.frame.can_id
        data = [recv_msg.frame.data[i] for i in range(recv_msg.frame.can_dlc)]

        if act_num > 1:
            # Received multiple messages, when can_id is the same, read multiple frames for concatenation
            for i in range(act_num - 1):
                msg = recv_msgs[i + 1]
                if msg.frame.can_id != can_id:
                    logger.warning(
                        f"Received multiple messages, can_id is different: {can_id:03X} != {msg.frame.can_id:03X}"
                    )
                    break
                # if msg.frame.can_dlc <= 2: # Multiple data frames, length greater than 2
                if msg.frame.can_dlc != 8:  # Multiple data frames, length not equal to 8
                    logger.warning(
                        f"Received multiple messages, data length is less than or equal to 2: {msg.frame.can_dlc}"
                    )
                    break  # Data length is less than or equal to 2, it is a single message
                data.extend([msg.frame.data[j] for j in range(msg.frame.can_dlc)])

        return (can_id, data)

    return None


# Baud rate preset configuration
# CAN one bit time = Sync + TSEG1 + TSEG2
# timing1 = 0x1C（TSEG1 = 12，TSEG2 = 7）→ Sampling point is later, fault tolerance is good, suitable for low speed (e.g. 125Kbps～500Kbps)
# timing1 = 0x14（TSEG1 = 4，TSEG2 = 5） → Time slot is short, sampling point is in the middle, suitable for high speed (e.g. 1Mbps)
# Sampling point % = (1 + TSEG1) / (1 + TSEG1 + TSEG2) × 100%
# Sampling point is 80%, should use 0x1C (i.e. TSEG1 = 12, TSEG2 = 3)
CAN_BAUDRATE_CONFIGS = {
    125000: {"timing0": 0x03, "timing1": 0x1C},
    250000: {"timing0": 0x01, "timing1": 0x1C},
    500000: {"timing0": 0x00, "timing1": 0x1C},
    1000000: {"timing0": 0x00, "timing1": 0x14},
}


def set_can_baudrate(device_handler, channel: int, baudrate: int) -> bool:
    """
    Set CAN baud rate.

    Args:
        device_handler: Device handle
        channel: Channel number (e.g. 0, 1)
        baudrate: Baud rate (only supports 125K, 250K, 500K, 1M)

    Returns:
        True if successful, False if failed
    """
    config = CAN_BAUDRATE_CONFIGS.get(baudrate)
    if not config:
        logger.error(f"Unsupported CAN baud rate: {baudrate}")
        return False

    try:
        zcan.ZCAN_SetValue(device_handler, f"{channel}/baud_rate", str(baudrate))
        zcan.ZCAN_SetValue(
            device_handler, f"{channel}/abit_timing0", f"0x{config['timing0']:02X}"
        )
        zcan.ZCAN_SetValue(
            device_handler, f"{channel}/abit_timing1", f"0x{config['timing1']:02X}"
        )
        logger.info(f"CAN baud rate has been set to: {baudrate} bps")
        return True
    except Exception as e:
        logger.exception(f"Set CAN baud rate failed: {e}")
        return False
