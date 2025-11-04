import threading
import time
import sys
from pathlib import Path
from typing import Optional
from canfd_utils import logger

current_dir = Path(__file__).resolve().parent
parent_dir = current_dir.parent
sys.path.append(str(parent_dir))
from dll.zlgcan.zlgcan import *

zcanlib = ZCAN()
device_handle = None
chn_handles = []
# threads = []

thread_flag = True
print_lock = threading.Lock()   # 线程锁，只是为了打印不冲突
enable_merge_receive = 0        # 合并接收标识

# 获取设备信息
def get_device_info(device_handle):
    info = zcanlib.GetDeviceInf(device_handle)
    if info is None:
        logger.error("获取设备信息失败")
        return None
    logger.info("设备信息: \n%s" % info)

    can_number = info.can_num
    return can_number

def zlgcan_open():
    global device_handle
    try:
        # 打开设备
        device_handle = zcanlib.OpenDevice(ZCAN_USBCANFD_100U, 0, 0)
        if device_handle == INVALID_DEVICE_HANDLE:
            logger.error("打开设备失败！")
            exit(0)
        logger.debug("设备句柄: %d." % device_handle)

        # 获取设备信息
        can_number = get_device_info(device_handle)
        if can_number is None:
            logger.error("获取设备信息失败")
            return

        # 启动通道
        for i in range(can_number):
            chn_handle = start_channel(zcanlib, device_handle, i)
            if chn_handle is None:
                logger.error("启动通道%d失败！" % i)
                exit(0)
            chn_handles.append(chn_handle)  # 将通道句柄添加到列表中
            logger.debug("通道句柄: %d." % chn_handle)

    except Exception as e:
        logger.error(str(e))

    # if enable_merge_receive == 1:   #   若开启合并接收，所有通道都由一个接收线程处理
    #     thread = threading.Thread(target=receive_thread, args=(handle,chn_handles[0]))    # 开启独立接收线程
    #     threads.append(thread)
    #     thread.start()
    # else:                           #   若没有开启合并接收，所有通道的数据需要各自开一个线程接收
    #     for i in range(len(chn_handles)):
    #         thread = threading.Thread(target=receive_thread, args=(handle, chn_handles[i]))  # 开启独立接收线程
    #         threads.append(thread)
    #         thread.start()

def zlgcan_close():
    # 关闭 定时发送/队列发送 任务
    # Clear_Send_Task(handle, 0)

    # 关闭接收线程
    # if enable_merge_receive == 1:
    #     threads[0].join()
    # else:
    #     for i in range(len(chn_handles)):
    #         threads[i].join()

    # 关闭通道
    for i in range(len(chn_handles)):
        ret = zcanlib.ResetCAN(chn_handles[i])
        if ret == 1:
            print(f"关闭通道{i}成功")

    # 关闭设备
    if device_handle is not None:
        ret = zcanlib.CloseDevice(device_handle)
        if ret == 1:
            print("关闭设备成功")

def zlgcan_send_message(can_id: int, data: bytes):
    # if chn_handles is empty
    if len(chn_handles) == 0:
        logger.error("通道句柄列表为空")
        return False

    channel_handle = chn_handles[0]

    logger.debug(f"can_id: 0x{can_id:02X}")
    logger.debug(f"data: {data.hex()}")

    # 发送 CANFD 报文
    transmit_canfd_num = 1
    canfd_msgs = (ZCAN_TransmitFD_Data * transmit_canfd_num)()
    for i in range(transmit_canfd_num):
        canfd_msgs[i].transmit_type = 0     # 0-正常发送，2-自发自收
        canfd_msgs[i].frame.can_id = can_id     # ID
        canfd_msgs[i].frame.can_id |= 1 << 31   # 扩展帧标志位 (bit 31)
        # CANFD 最大数据长度为 64 字节
        data_len = min(len(data), 64)
        canfd_msgs[i].frame.len = data_len     # 数据长度
        canfd_msgs[i].frame.flags = 0         # 先清零
        # canfd_msgs[i].frame.flags |= 0x20    # bit 5: 发送回显标志位
        canfd_msgs[i].frame.flags |= 0x1     # bit 0: BRS 加速标志位：0不加速，1加速
        canfd_msgs[i].frame._res0 = 10      # 保留字段
        # 复制数据
        for j in range(data_len):
            canfd_msgs[i].frame.data[j] = data[j]

    ret = zcanlib.TransmitFD(channel_handle, canfd_msgs, transmit_canfd_num)
    if ret != transmit_canfd_num:
        logger.error("发送数据失败!")
        return False
    logger.debug(f"发送数据成功: {ret}")
    return True

def zlgcan_receive_message(quick_retries: int = 2, dely_retries: int = 0) -> Optional[tuple[int, list[int]]]:
    """
    接收CAN总线消息，先尝试快速接收，若失败则尝试DFU模式下的慢速接收。

    参数:
        quick_retries (int): 快速接收尝试次数，默认2次。
        dely_retries (int): DFU模式下的慢速接收尝试次数，默认0次。

    返回:
        接收到的消息或None(接收失败)
    """
    # if chn_handles is empty
    if len(chn_handles) == 0:
        logger.error("通道句柄列表为空")
        return None

    channel_handle = chn_handles[0]

    # 快速接收尝试
    for _attempt in range(quick_retries):
        time.sleep(0.001)  # 较短延时
        message = _zlgcan_read_messages(channel_handle)
        if message is not None:
            can_id = message.frame.can_id
            data = [message.frame.data[i] for i in range(message.frame.len)]
            return (can_id, data)

    logger.warning("快速接收超时")

    # 慢速接收尝试
    for _attempt in range(dely_retries):
        # time.sleep(0.5)  # 较长延时，适用于DFU模式
        time.sleep(0.005)
        message = _zlgcan_read_messages(channel_handle)
        if message is not None:
            return message

    if dely_retries > 0:
        logger.error("慢速接收尝试也超时!")
    return None

def _zlgcan_read_messages(channel_handle: int) -> Optional[ZCAN_ReceiveFD_Data]:
    """
    从 ZLG CANFD 设备读取消息，跳过 TX 回显

    返回:
        最后一条有效接收消息（跳过 TX 回显）
    """
    # 方便打印对齐 --无实际作用
    CANType_width = len("CANFD加速    ")
    id_width = len(hex(0x1FFFFFFF))

    rcv_canfd_num = zcanlib.GetReceiveNum(channel_handle, ZCAN_TYPE_CANFD)  # CANFD
    if rcv_canfd_num:
        if rcv_canfd_num > 100:
            rcv_canfd_msgs, rcv_canfd_num = zcanlib.ReceiveFD(channel_handle, 100, c_int(100))
        else:
            rcv_canfd_msgs, rcv_canfd_num = zcanlib.ReceiveFD(channel_handle, rcv_canfd_num, c_int(100))

        # 打印接收到的消息（调试用）
        with print_lock:
            for msg in rcv_canfd_msgs[:rcv_canfd_num]:
                frame = msg.frame
                brs = "加速" if frame.flags & 0x1 else "   "
                can_type = "CANFD" + brs
                direction = "TX" if frame.flags & 0x20 else "RX"
                frame_type = "扩展帧" if frame.can_id & (1 << 31) else "标准帧"
                frame_format = "远程帧" if frame.can_id & (1 << 30) else "数据帧"  # CANFD没有远程帧
                can_id = hex(frame.can_id & 0x1FFFFFFF)
                data = " ".join([f"{num:02X}" for num in frame.data[:frame.len]])

                print(f"[{msg.timestamp}] CAN{channel_handle & 0xFF} {can_type:<{CANType_width}}\t{direction} ID: {can_id:<{id_width}}\t{frame_type} {frame_format}"
                      f" DLC: {frame.len}\tDATA(hex): {data}")

        # 找到最后一条接收消息（跳过 TX 回显）
        last_rx_msg = None
        for i in range(rcv_canfd_num - 1, -1, -1):  # 从后往前查找
            # 检查是否为 TX 回显（flags bit5 为 1 表示 TX）
            if not (rcv_canfd_msgs[i].frame.flags & 0x20):  # 非 TX 回显
                last_rx_msg = rcv_canfd_msgs[i]
                break

        if last_rx_msg is None:
            # 所有消息都是 TX 回显，无有效接收数据
            logger.debug("所有消息都是 TX 回显，无有效接收数据")
            return None

        return last_rx_msg

    return None

# 启动通道
def start_channel(zcanlib: ZCAN, device_handle: int, chn: int):

    # 设置控制器设备类型，如果不是 Non-ISO CANFD 可注释掉按设备默认即可 CAN/CANFD 都是默认
    # ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/canfd_standard", "0".encode("utf-8"))

    # 仲裁域波特率 和 数据域波特率
    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/canfd_abit_baud_rate", "1000000".encode("utf-8"))
    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/canfd_dbit_baud_rate", "5000000".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d baud failed!" % chn)
        return None

    # 自定义波特率    当产品波特率对采样点有要求，或者需要设置非常规波特率时使用   ---默认不管
    # ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/baud_rate_custom", "500Kbps(80%),2.0Mbps(80%),(80,07C00002,01C00002)".encode("utf-8"))
    # if ret != ZCAN_STATUS_OK:
    #     print("Set CH%d baud failed!" % chn)
    #     return None

    # 终端电阻
    # ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/initenal_resistance", "1".encode("utf-8"))
    # if ret != ZCAN_STATUS_OK:
    #     print("Open CH%d resistance failed!" % chn)
    #     return None

    # 初始化通道
    chn_init_cfg = ZCAN_CHANNEL_INIT_CONFIG()
    chn_init_cfg.can_type = ZCAN_TYPE_CANFD  # USBCANFD 必须选择CANFD
    chn_init_cfg.config.canfd.mode = 0  # 0-正常模式 1-只听模式
    chn_handle = zcanlib.InitCAN(device_handle, chn, chn_init_cfg)
    if chn_handle is None:
        print("initCAN failed!" % chn)
        return None

    # 设置滤波
    # Set_Filter(device_handle, chn)

    # 设置发送回显    USBCANFD系列老卡（无LIN口，设备白色标签值版本为V1.02即以下,包括V1.02）需要以下操作，否则由CAN报文结构体_pad结构体bit5标识，见Transmit_Test 发送示例
    # ret = zcanlib.ZCAN_SetValue(device_handle,str(chn)+"/set_device_tx_echo","0".encode("utf-8"))   #发送回显设置，0-禁用，1-开启
    # if ret != ZCAN_STATUS_OK:
    #     print("Set CH%d  set_device_tx_echo failed!" %(chn))
    #     return None

    # 使能/关闭合并接收(startCAN 之前)    0-关闭 1-使能
    ret = zcanlib.ZCAN_SetValue(device_handle, str(0) + "/set_device_recv_merge", repr(enable_merge_receive))
    if ret != ZCAN_STATUS_OK:
        print("Open CH%d recv merge failed!" % chn)
        return None

    # 启动通道
    ret = zcanlib.StartCAN(chn_handle)
    if ret != ZCAN_STATUS_OK:
        print("startCAN failed!" % chn)
        return None

    return chn_handle
