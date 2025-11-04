from ctypes import *
from typing import Optional
import time

# 加载 USBCANFD 动态库
# 使用实际的库文件路径，例如: "./libusbcanfd.so" 或 "/path/to/libusbcanfd.so"
lib = cdll.LoadLibrary("/usr/lib/libusbcanfd.so")

CMD_CAN_FILTER                  = 0x14   # 滤波
CMD_CAN_TTX                     = 0x16   # 定时发送
CMD_CAN_TTX_CTL                 = 0x17   # 使能定时发送
CMD_CAN_TRES                    = 0x18   # CAN终端电阻
ZCAN_CMD_SET_CHNL_RECV_MERGE    = 0x32   # 设置合并接收 0:不合并接收;1:合并接收
ZCAN_CMD_GET_CHNL_RECV_MERGE    = 0x33   # 获取是否开启合并接收 0:不合并接收;1:合并接收
CMD_SET_SN                      = 0x42   # 获取SN号
CMD_GET_SN                      = 0x43   # 设置SN号
CMD_CAN_TX_TIMEOUT              = 0x44   # 发送超时
ZCAN_CMD_GET_SEND_QUEUE_SIZE    = 0x100  # 获取队列大小，uint32_t
ZCAN_CMD_GET_SEND_QUEUE_SPACE   = 0x101  # 获取队列剩余空间, uint32_t
ZCAN_CMD_SET_SEND_QUEUE_CLR     = 0x102  # 清空发送队列,1：清空
ZCAN_CMD_SET_SEND_QUEUE_EN      = 0x103  # 开启发送队列,1：使能

USBCANFD = c_uint32(33)  # 设备类型号
MAX_CHANNELS = 2         # 通道最大数量
g_thd_run = 1            # 线程运行标志
threads = []             # 接收线程

# can/canfd messgae info
class ZCAN_MSG_INFO(Structure):
    _fields_ = [("txm", c_uint, 4),  # TXTYPE:0 normal,1 once, 2self
                ("fmt", c_uint, 4),  # 0-can2.0 frame,  1-canfd frame
                ("sdf", c_uint, 1),  # 0-data frame, 1-remote frame
                ("sef", c_uint, 1),  # 0-std_frame, 1-ext_frame
                ("err", c_uint, 1),  # error flag
                ("brs", c_uint, 1),  # bit-rate switch ,0-Not speed up ,1-speed up
                ("est", c_uint, 1),  # error state
                ("tx", c_uint, 1),      # received valid, tx frame
                ("echo", c_uint, 1),    # tx valid, echo frame
                ("qsend_100us", c_uint, 1), # queue send delay unit, 1-100us, 0-ms
                ("qsend", c_uint, 1),       # send valid, queue send frame
                ("pad", c_uint, 15)]


# CAN Message Header
class ZCAN_MSG_HDR(Structure):
    _fields_ = [("ts", c_uint32),  # timestamp
                ("id", c_uint32),  # can-id
                ("inf", ZCAN_MSG_INFO),
                ("pad", c_uint16),
                ("chn", c_uint8),  # channel
                ("len", c_uint8)]  # data length

# CAN2.0-frame
class ZCAN_20_MSG(Structure):
    _fields_ = [("hdr", ZCAN_MSG_HDR),
                ("dat", c_ubyte*8)]


# CANFD frame
class ZCAN_FD_MSG(Structure):
    _fields_ = [("hdr", ZCAN_MSG_HDR),
                ("dat", c_ubyte*64)]

# filter_set
class ZCAN_FILTER(Structure):
    _fields_ = [("type", c_uint8),  # 0-std_frame,1-extend_frame
                ("pad", c_uint8*3),  # reserved
                ("sid", c_uint32),  # start_ID
                ("eid", c_uint32)]  # end_ID


class ZCAN_FILTER_TABLE(Structure):
    _fields_ = [("size", c_uint32),  # 滤波数组table实际生效部分的长度
                ("table", ZCAN_FILTER*64)]


class abit_config(Structure):
    _fields_ = [("tseg1", c_uint8),
                ("tseg2", c_uint8),
                ("sjw", c_uint8),
                ("smp", c_uint8),
                ("brp", c_uint16)]


class dbit_config(Structure):
    _fields_ = [("tseg1", c_uint8),
                ("tseg2", c_uint8),
                ("sjw", c_uint8),
                ("smp", c_uint8),
                ("brp", c_uint16)]


class ZCANFD_INIT(Structure):
    _fields_ = [("clk", c_uint32),
                ("mode", c_uint32),
                ("abit", abit_config),
                ("dbit", dbit_config)]

# Terminating resistor
class Resistance(Structure):
    _fields_ = [("res", c_uint8)]

# autosend
class ZCAN_TTX(Structure):
    _fields_ = [("interval", c_uint32),   # 定时发送周期，单位百微秒
                ("repeat", c_uint16),     # 发送次数，0等于循环发
                ("index", c_uint8),       # 定时发送列表的帧索引号，也就是第几条定时发送报文
                ("flags", c_uint8),       # 0-此帧禁用定时发送，1-此帧使能定时发送
                ("msg", ZCAN_FD_MSG)]     # CANFD帧结构体

# autosend list
class ZCAN_TTX_CFG(Structure):
    _fields_ = [("size", c_uint32),       # 实际生效的数组的长度
                ("table", ZCAN_TTX * 8)]  # 最大设置8条

############  uds resoponse #############
class PARAM_DATA(Structure):
    _pack_ = 1
    _fields_ = [("data", c_ubyte*4096)]

class DATA_BUFFER(Structure):
    _pack_ = 1
    _fields_ = [("data", c_ubyte*4096)]

class POSITIVE_DATA(Structure):
    _pack_ = 1
    _fields_ = [("sid", c_ubyte),
                ("data_len", c_uint),
                ]

class NEGATIVE_DATA(Structure):
    _pack_ = 1
    _fields_ = [("neg_code", c_ubyte),
                ("sid", c_ubyte),
                ("error_code", c_ubyte),
                ]

class RESPONSE_DATA(Union):
    _pack_ = 1
    _fields_ = [("positive", POSITIVE_DATA),
                ("negative", NEGATIVE_DATA),
                ("raw", c_byte*8),
                ]

class ZCAN_UDS_RESPONSE(Structure):
    _pack_ = 1
    _fields_ = [
        ("status", c_byte),  # 见ZCAN_UDS_ERROR说明
        ("reserved", c_byte*6),
        ("type", c_byte),  # 0-消极响应,1-积极响应
        ("response", RESPONSE_DATA),
    ]

#######################  uds request #################

class ZCAN_UDS_SESSION_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("p2_timeout", c_uint),
                # 收到消极响应错误码为0x78后的超时时间(ms)。因PC定时器误差，建议设置不小于200ms
                ("enhanced_timeout", c_uint),
                # 接收到非本次请求服务的消极响应时是否需要判定为响应错误
                ("check_any_negative_response", c_ubyte, 1),
                # 抑制响应时是否需要等待消极响应，等待时长为响应超时时间
                ("wait_if_suppress_response", c_ubyte, 1),
                ("flag", c_ubyte, 6),  # 保留
                ("reserved0", c_byte*7)
                ]

class ZCAN_UDS_TRANS_PARAM(Structure):
    _pack_ = 1
    _fields_ = [
        ("version", c_byte),  # 0-2004版本，1-2016版本
        ("max_data_len", c_byte),  # 单帧最大数据长度, can:8, canfd:64
        # 本程序发送流控时用，连续帧之间的最小间隔, 0x00-0x7F(0ms~127ms), 0xF1-0xF9(100us~900us)
        ("local_st_min", c_byte),
        ("block_size", c_byte),  # 流控帧的块大小
        ("fill_byte", c_byte),  # 无效字节的填充数据
        ("ext_frame", c_byte),  # 0:标准帧 1:扩展帧
        # 是否忽略ECU返回流控的STmin，强制使用本程序设置的 remote_st_min
        ("is_modify_ecu_st_min", c_byte),
        # 发送多帧时用, is_ignore_ecu_st_min = 1 时有效, 0x00-0x7F(0ms~127ms), 0xF1-0xF9(100us~900us)
        ("remote_st_min", c_byte),
        ("fc_timeout", c_uint),  # 接收流控超时时间(ms), 如发送首帧后需要等待回应流控帧
        ("fill_mode", c_ubyte),  # 0-FILL_MODE_SHORT,1-FILL_MODE_NONE,2-FILL_MODE_MAX
        ("reserved0", c_byte*3),
    ]

class ZCAN_UDS_REQUEST(Structure):
    _pack_ = 1
    _fields_ = [("req_id", c_uint),         # 请求的事务ID，范围0~65535，本次请求的唯一标识
                ("channel", c_ubyte),       # 设备通道索引
                ("frame_type", c_ubyte),    # 0-can,1-CANFD,2-CANFD加速
                ("reserved0", c_byte*2),
                ("src_addr", c_uint),           # 请求ID
                ("dst_addr", c_uint),           # 响应ID
                ("suppress_response", c_byte),  # 1-抑制响应
                ("sid", c_ubyte),               # 请求服务id
                ("reserved1", c_byte*6),
                ("session_param", ZCAN_UDS_SESSION_PARAM),  # 会话层参数
                ("trans_param", ZCAN_UDS_TRANS_PARAM),      # 传输层参数
                ("data", POINTER(c_ubyte)),                 # 请求参数
                ("data_len", c_uint),                       # 请求参数长度
                ("reserved2", c_uint),
                ]

# 构建 CAN 帧
def construct_can_frame(can_id: int, ChIdx: int, data: bytes) -> ZCAN_20_MSG:
    can_frame = ZCAN_20_MSG()
    can_frame.hdr.inf.txm = 0  # 0-正常发送, 2-自发自收
    can_frame.hdr.inf.fmt = 0  # 0-CAN
    can_frame.hdr.inf.sdf = 0  # 0-数据帧 1-远程帧
    can_frame.hdr.inf.sef = 0  # 0-标准帧 1-扩展帧
    # can_frame.hdr.inf.echo = 1  # 发送回显

    can_dlc = len(data)
    if can_dlc > 8:
        raise ValueError("CAN data length must be less than or equal to 8")
    can_frame.hdr.id = can_id
    can_frame.hdr.chn = ChIdx
    can_frame.hdr.len = can_dlc

    for i in range(can_dlc):
        can_frame.dat[i] = data[i]

    return can_frame


# 构建 CANFD 帧
def construct_canfd_frame(can_id: int, ChIdx: int, data: bytes) -> ZCAN_FD_MSG:
    canfd_frame = ZCAN_FD_MSG()
    canfd_frame.hdr.inf.txm = 0  # 0-正常发送
    canfd_frame.hdr.inf.fmt = 1  # 1-CANFD
    canfd_frame.hdr.inf.sdf = 0  # 0-数据帧 CANFD只有数据帧!
    canfd_frame.hdr.inf.sef = 1  # 0-标准帧, 1-扩展帧
    canfd_frame.hdr.inf.brs = 1  # 1-CANFD加速
    canfd_frame.hdr.inf.echo = 1  # 1-发送回显

    can_dlc = len(data)
    if can_dlc > 64:
        raise ValueError("CANFD data length must be less than or equal to 64")
    canfd_frame.hdr.id = can_id
    canfd_frame.hdr.chn = ChIdx
    canfd_frame.hdr.len = can_dlc

    for i in range(can_dlc):
        canfd_frame.dat[i] = data[i]

    return canfd_frame


# 打开设备
def open_device(DevType: int, DevIdx: int):
    global lib
    ret = lib.VCI_OpenDevice(DevType, DevIdx, 0)
    if ret == 0:
        print("Open device fail")
        exit(0)
    else:
        print("Open device success")


# 关闭设备
def close_device(DevType: int, DevIdx: int):
    global lib
    # for i in range(MAX_CHANNELS):
    ret = lib.VCI_ResetCAN(DevType, DevIdx, 0)
    if ret == 0:
        print("ResetCAN(%d) fail" % 0)
    else:
        print("ResetCAN(%d) success!" % 0)

    ret = lib.VCI_CloseDevice(DevType, DevIdx)
    if ret == 0:
        print("Close device fail")
        exit(0)
    else:
        print("Close device success")
    del lib


# 通道初始化，并开启接收线程
def canfd_start(DevType: int, DevIdx: int, ChIdx: int):
    # 波特率结构体，数据根据zcanpro的波特率计算器得出
    canfd_init = ZCANFD_INIT()
    canfd_init.clk = 60000000
    canfd_init.mode = 0

    # 仲裁域 1Mbps 75%采样
    canfd_init.abit.tseg1 = 7
    canfd_init.abit.tseg2 = 2
    canfd_init.abit.sjw = 1
    canfd_init.abit.smp = 0   # smp是采样点，不涉及波特率计算
    canfd_init.abit.brp = 4

    # 数据域 5Mbps 75%采样
    canfd_init.dbit.tseg1 = 7
    canfd_init.dbit.tseg2 = 2
    canfd_init.dbit.sjw = 1
    canfd_init.dbit.smp = 0
    canfd_init.dbit.brp = 0

    # 初始化通道
    ret = lib.VCI_InitCAN(DevType, DevIdx, ChIdx, byref(canfd_init))
    if ret == 0:
        print("InitCAN(%d) fail" % ChIdx)
        exit(0)
    else:
        print("InitCAN(%d) success" % ChIdx)

    # 使能终端电阻
    # Res = c_uint8(1)
    # lib.VCI_SetReference(DevType, DevIdx, ChIdx, CMD_CAN_TRES, byref(Res))

    # 滤波设置
    # filter_set = 0x14
    # filter_table = ZCAN_FILTER_TABLE()
    # memset(byref(filter_table), 0, sizeof(filter_table))
    # filter_table.size = sizeof(ZCAN_FILTER)*1  # 设置一组滤波参数
    # filter_table.table[0].type = 0
    # filter_table.table[0].sid = 0x0
    # filter_table.table[0].eid = 0x7FF  # 设置滤波标准帧，范围0~0x7FF
    # ret = lib.VCI_SetReference(
    #     Devicetype, DeviceIndex, Channel, filter_set, byref(filter_table))
    #
    # wait_tx = 0x44  # tx_timeout
    # tx_timeout = ZCAN_tx_timeout(200)
    # ret = lib.VCI_SetReference(
    #     Devicetype, DeviceIndex, Channel, wait_tx, byref(tx_timeout))

    # 启动通道
    ret = lib.VCI_StartCAN(DevType, DevIdx, ChIdx)
    if ret == 0:
        print("StartCAN(%d) fail" % ChIdx)
        exit(0)
    else:
        print("StartCAN(%d) success" % ChIdx)


# 发送CAN数据
def can_send(DevType: int, DevIdx: int, ChIdx: int, can_id: int, data: bytes) -> bool:
    send_num = 1
    can_frame = (ZCAN_20_MSG * send_num)()
    for i in range(send_num):
        can_frame[i] = construct_can_frame(can_id, ChIdx, data)
    ret = lib.VCI_Transmit(DevType, DevIdx, ChIdx, byref(can_frame), send_num)
    # print("Transmit num is: %d" % ret)
    return ret == send_num


# 发送CANFD数据
def canfd_send(DevType: int, DevIdx: int, ChIdx: int, can_id: int, data: bytes) -> bool:
    send_num = 1
    canfd_frame = (ZCAN_FD_MSG * send_num)()
    for i in range(send_num):
        canfd_frame[i] = construct_canfd_frame(can_id, ChIdx, data)
    ret = lib.VCI_TransmitFD(DevType, DevIdx, ChIdx, byref(canfd_frame), send_num)
    # print("TransmitFD num is: %d" % ret)
    return ret == send_num


# 接收CAN数据（过滤TX回显，返回所有RX数据）
# 返回: ctypes 数组 Array[ZCAN_20_MSG] 或 None
def can_receive(DevType: int, DevIdx: int, ChIdx: int):
    # time.sleep(0.001)
    count = lib.VCI_GetReceiveNum(DevType, DevIdx, ChIdx)  # CAN 报文数量
    # print("CAN Receive count: %d" % count)
    if count > 0:
        can_data = (ZCAN_20_MSG * count)()
        rcount = lib.VCI_Receive(DevType, DevIdx, ChIdx, byref(can_data), count, 100)  # 读报文

        # 打印所有接收到的消息（包括TX回显）
        # for i in range(rcount):
        #     print("[%u] chn: %d " % (can_data[i].hdr.ts, ChIdx), end='')
        #     print("TX  " if can_data[i].hdr.inf.tx == 1 else "RX  ", end='')  # 判断是否回显

        #     print("CAN ID: 0x%x " % (can_data[i].hdr.id & 0x1FFFFFFF), end='')
        #     print("扩展帧  " if can_data[i].hdr.inf.sef == 1 else "标准帧  ", end='')
        #     print("Data: ", end='')
        #     if can_data[i].hdr.inf.sdf == 0:  # 数据帧
        #         for j in range(can_data[i].hdr.len):
        #             print("%02x " % can_data[i].dat[j], end='')
        #     print("")

        # 过滤掉TX回显，只保留RX接收数据
        rx_count = 0
        for i in range(rcount):
            if can_data[i].hdr.inf.tx == 0:  # 非TX回显，即RX接收数据
                rx_count += 1

        if rx_count == 0:
            # 如果所有消息都是TX回显，返回None
            print("All messages are TX echo, no RX data")
            return None

        # 创建只包含RX数据的新数组
        rx_data = (ZCAN_20_MSG * rx_count)()
        rx_idx = 0
        for i in range(rcount):
            if can_data[i].hdr.inf.tx == 0:  # 非TX回显
                rx_data[rx_idx] = can_data[i]
                rx_idx += 1

        return rx_data
    else:
        print("No CAN data received")
    return None


# 接收CANFD数据（过滤TX回显，返回所有RX数据）
# 返回: ctypes 数组 Array[ZCAN_FD_MSG] 或 None
def canfd_receive(DevType: int, DevIdx: int, ChIdx: int):
    count = lib.VCI_GetReceiveNum(DevType, DevIdx, (0x80000000 + ChIdx))  # CANFD 报文数量
    # print("CANFD Receive count: %d" % count)
    if count > 0:
        canfd_data = (ZCAN_FD_MSG * count)()
        rcount = lib.VCI_ReceiveFD(DevType, DevIdx, ChIdx, byref(canfd_data), count, 100)  # 读报文

        # 打印所有接收到的消息（包括TX回显）
        # for i in range(rcount):
        #     print("[%u] chn: %d " % (canfd_data[i].hdr.ts, ChIdx), end='')
        #     print("TX  " if canfd_data[i].hdr.inf.tx == 1 else "RX  ", end='')
        #     print("CANFD加速 " if canfd_data[i].hdr.inf.brs == 1 else "CANFD  ", end='')
        #     print("ID: 0x%x " % (canfd_data[i].hdr.id & 0x1FFFFFFF), end='')
        #     print("扩展帧  " if canfd_data[i].hdr.inf.sef == 1 else "标准帧  ", end='')
        #     print("Data: ", end='')
        #     for j in range(canfd_data[i].hdr.len):
        #         print("%02x " % canfd_data[i].dat[j], end='')
        #     print("")

        # 过滤掉TX回显，只保留RX接收数据
        rx_count = 0
        for i in range(rcount):
            if canfd_data[i].hdr.inf.tx == 0:  # 非TX回显，即RX接收数据
                rx_count += 1

        if rx_count == 0:
            # 如果所有消息都是TX回显，返回None
            print("All messages are TX echo, no RX data")
            return None

        # 创建只包含RX数据的新数组
        rx_data = (ZCAN_FD_MSG * rx_count)()
        rx_idx = 0
        for i in range(rcount):
            if canfd_data[i].hdr.inf.tx == 0:  # 非TX回显
                rx_data[rx_idx] = canfd_data[i]
                rx_idx += 1

        return rx_data
    else:
        print("No CANFD data received")
    return None
