from ctypes import *
import threading
import time
import datetime

lib = cdll.LoadLibrary("./libusbcanfd.so")

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
def construct_can_frame(id, ChIdx, pad):
    can_frame = ZCAN_20_MSG()
    can_frame.hdr.inf.txm = 0  # 0-正常发送, 2--自发自收
    can_frame.hdr.inf.fmt = 0  # 0-CAN
    can_frame.hdr.inf.sdf = 0  # 0-数据帧 1-远程帧
    can_frame.hdr.inf.sef = 0  # 0-标准帧 1- 扩展帧
    #can_frame[i].hdr.inf.echo = 1   # 发送回显

    can_frame.hdr.id = id
    can_frame.hdr.chn = ChIdx
    can_frame.hdr.len = 8

    # 队列发送
    if(pad > 0):
        can_frame.hdr.pad = pad;              # 发送后延迟 pad ms
        can_frame.hdr.inf.qsend = 1;          # 队列发送帧，仅判断首帧
        can_frame.hdr.inf.qsend_100us = 0;    # 队列发送单位，0-ms，1-100us

    for i in range(can_frame.hdr.len):
        can_frame.dat[i] = i
    
    return can_frame


# 构建 CAN 帧
def construct_canfd_frame(id, ChIdx, pad):
    canfd_frame = ZCAN_FD_MSG()
    canfd_frame.hdr.inf.txm = 0  # 0-正常发送
    canfd_frame.hdr.inf.fmt = 1  # 1-CANFD
    canfd_frame.hdr.inf.sdf = 0  # 0-数据帧 CANFD只有数据帧!
    canfd_frame.hdr.inf.sef = 1  # 0-标准帧, 1-扩展帧
    canfd_frame.hdr.inf.brs = 1  # 1-CANFD加速
    #canfd_frame[i].hdr.inf.echo  = 1;  # 发送回显
    
    canfd_frame.hdr.id = id
    canfd_frame.hdr.chn = ChIdx
    canfd_frame.hdr.len = 64

    # 队列发送
    if(pad > 0):
        canfd_frame.hdr.pad = pad;              # 发送后延迟 pad ms
        canfd_frame.hdr.inf.qsend = 1;          # 队列发送帧，仅判断首帧
        canfd_frame.hdr.inf.qsend_100us = 0;    # 队列发送单位，0-ms，1-100us
    
    for i in range(canfd_frame.hdr.len):
        canfd_frame.dat[i] = i

    return canfd_frame


# 通道初始化，并开启接收线程
def canfd_start(DevType, DevIdx, ChIdx):
    # 波特率结构体，数据根据zcanpro的波特率计算器得出
    canfd_init = ZCANFD_INIT()
    canfd_init.clk = 60000000
    canfd_init.mode = 0

    canfd_init.abit.tseg1 = 14  # 仲裁域
    canfd_init.abit.tseg2 = 3
    canfd_init.abit.sjw = 2
    canfd_init.abit.smp = 0   # smp是采样点，不涉及波特率计算
    canfd_init.abit.brp = 5

    canfd_init.dbit.tseg1 = 10  # 数据域
    canfd_init.dbit.tseg2 = 2
    canfd_init.dbit.sjw = 2
    canfd_init.dbit.smp = 0
    canfd_init.dbit.brp = 1

    # 初始化通道
    ret = lib.VCI_InitCAN(DevType, DevIdx, ChIdx, byref(canfd_init))
    if ret == 0:
        print("InitCAN(%d) fail" % ChIdx)
        exit(0)
    else:
        print("InitCAN(%d) success" % ChIdx)

    # 使能终端电阻
    Res = c_uint8(1)
    lib.VCI_SetReference(DevType, DevIdx, ChIdx, CMD_CAN_TRES, byref(Res))

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

    thread = threading.Thread(target=rx_thread, args=(DevType, DevIdx, ChIdx,))
    threads.append(thread) # 独立接收线程
    thread.start()


# 接收线程
def rx_thread(DevType, DevIdx, ChIdx):
    global g_thd_run

    while g_thd_run == 1:
        time.sleep(0.1)
        count = lib.VCI_GetReceiveNum(DevType, DevIdx, ChIdx) # CAN 报文数量
        if count > 0:
            can_data = (ZCAN_20_MSG * count)()
            rcount = lib.VCI_Receive(DevType, DevIdx, ChIdx, byref(can_data), count, 100) # 读报文
            for i in range(rcount):
                print("[%u] chn: %d " %(can_data[i].hdr.ts, ChIdx), end='')
                print("TX  " if can_data[i].hdr.inf.tx == 1 else "RX  ", end='')       # 判断是否回显
                print("CAN ID: 0x%x "%(can_data[i].hdr.id & 0x1FFFFFFF), end='')
                print("扩展帧  " if can_data[i].hdr.inf.sef == 1 else "标准帧  ", end='')
                print("Data: ", end='')
                if(can_data[i].hdr.inf.sdf == 0):   #数据帧
                    for j in range(can_data[i].hdr.len):
                        print("%02x " % can_data[i].dat[j], end='')
                print("")

        count = lib.VCI_GetReceiveNum(DevType, DevIdx, (0x80000000 + ChIdx)) # CANFD 报文数量
        if count > 0:
            canfd_data = (ZCAN_FD_MSG * count)()
            rcount = lib.VCI_ReceiveFD(DevType, DevIdx, ChIdx, byref(canfd_data), count, 100)
            for i in range(rcount):
                print("[%u] chn: %d " %(canfd_data[i].hdr.ts, ChIdx), end='')
                print("TX  " if canfd_data[i].hdr.inf.tx == 1 else "RX  ", end='')
                print("CANFD加速 " if canfd_data[i].hdr.inf.brs == 1 else "CANFD  ", end='')
                print("ID: 0x%x "%(canfd_data[i].hdr.id & 0x1FFFFFFF), end='')
                print("扩展帧  " if canfd_data[i].hdr.inf.sef == 1 else "标准帧  ", end='')

                print("Data: ", end='')
                for j in range(canfd_data[i].hdr.len):
                    print("%02x " % canfd_data[i].dat[j], end='')
                print("")


# 发送数据
def can_send(DevType, DevIdx, ChIdx):
    send_num = 10

    # CAN
    can_frame = (ZCAN_20_MSG * send_num)()
    for i in range(send_num):
        can_frame[i] = construct_can_frame(i, ChIdx, 0)
    ret = lib.VCI_Transmit(DevType, DevIdx, ChIdx, byref(can_frame), 10)
    print("Transmit num is:%d" % ret)

    # CANFD
    canfd_frame = (ZCAN_FD_MSG * send_num)()
    for i in range(send_num):
        canfd_frame[i] = construct_canfd_frame(i, ChIdx, 0)
    ret = lib.VCI_TransmitFD(DevType, DevIdx, ChIdx, byref(canfd_frame), 10)
    print("TransmitFD num is:%d" % ret)


# 队列发送
def queue_send(DevType, DevIdx, ChIdx):
    snd_queue_size = c_uint32(0);    # 队列大小
    snd_queue_remain = c_uint32(0);  # 队列剩余空间
    on = c_uint8(1)

    # 开启队列发送（带LIN版本无需调用）
    lib.VCI_SetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_SET_SEND_QUEUE_EN, byref(on))

    # 获取队列大小，最多填充个数
    lib.VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SIZE, byref(snd_queue_size))
    
    # 获取队列剩余可填充报文帧数
    lib.VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, byref(snd_queue_remain))
    print("Chn%d send queue size:%u, remain space:%u\n"% (ChIdx, snd_queue_size.value, snd_queue_remain.value))

    # 发送 10 次100帧的报文
    transmit_time = 10
    transmit_num = 100
    can_msg = (ZCAN_20_MSG * 100)()
    canfd_msg = (ZCAN_FD_MSG * 100)()
    while(True):
        # 队列发送前需要判断可填充报文帧数!!
        lib.VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, byref(snd_queue_remain))   
        if(snd_queue_remain.value >= transmit_num):
            for i in range(100):
                can_msg[i] = construct_can_frame(i, ChIdx, 100);   # 10ms

            ret = lib.VCI_Transmit(DevType, DevIdx, ChIdx, can_msg, 100)       # CAN
            print("Chn%d queue send can frame %u, ret=%u\n" % (ChIdx, 100, ret))
            
            transmit_time = transmit_time - 1
            if(transmit_time == 0):
                break

        # lib.VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, byref(snd_queue_remain))  
        # if(snd_queue_remain.value >= transmit_num):                   
        #     for i in range(100):
        #         canfd_msg[i] = construct_canfd_frame(i, ChIdx, 100)   # 100ms

        #     ret = lib.VCI_TransmitFD(DevType, DevIdx, ChIdx, canfd_msg, 100)      # CANFD
        #     print("Chn%d remain space:%u, queue send canfd frame %u\n" % (ChIdx, snd_queue_remain.value, ret))
            
        #     transmit_time -= 1
        #     if(transmit_time == 0):
        #         break

    time.sleep(1)

    # 发送后，队列空间变化
    lib.VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, byref(snd_queue_remain))
    print("Chn%d remain space:%u" % (ChIdx, snd_queue_remain.value))

    # 清空队列发送
    lib.VCI_SetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_SET_SEND_QUEUE_CLR, byref(on))
    print("Chn%d send queue clear" % ChIdx)

    # 清空队列后，队列空间恢复初始值
    lib.VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, byref(snd_queue_remain))
    print("Chn%d remain space:%u" % (ChIdx, snd_queue_remain.value))


# 定时发送
def auto_send(DevType, DevIdx, ChIdx):
    autosend_num = 2    # (最多8条)

    # 定时发送
    autosend_cfg = ZCAN_TTX_CFG()
    autosend_cfg.size = sizeof(ZCAN_TTX) * autosend_num  # 数量

    for i in range(autosend_num):
        autosend_cfg.table[i].interval = 5000   # 500ms，单位 100 us
        autosend_cfg.table[i].repeat = 0        # 0-一直发
        autosend_cfg.table[i].index = i         # 索引
        autosend_cfg.table[i].flags = 1         # 使能
        
        autosend_cfg.table[i].msg.hdr.id = 0x100 + i
        autosend_cfg.table[i].msg.hdr.inf.txm = 0   # 0为正常模式，2为自发自收
        autosend_cfg.table[i].msg.hdr.inf.fmt = 0   # 0-CAN帧，1-CANFD帧
        autosend_cfg.table[i].msg.hdr.inf.sdf = 0   # 0-数据帧，1-远程帧
        autosend_cfg.table[i].msg.hdr.inf.sef = 0   # 0-标准帧，1-扩展帧
        autosend_cfg.table[i].msg.hdr.inf.brs = 0   # 0-CANFD不加速，1-CANFD加速
        autosend_cfg.table[i].msg.hdr.inf.echo = 1  #发送回显

        autosend_cfg.table[i].msg.hdr.pad = 0
        autosend_cfg.table[i].msg.hdr.chn = 0
        autosend_cfg.table[i].msg.hdr.len = 8
        for j in range(autosend_cfg.table[i].msg.hdr.len):
            autosend_cfg.table[i].msg.dat[j] = j

    ret = lib.VCI_SetReference(DevType, DevIdx, ChIdx, CMD_CAN_TTX, byref(autosend_cfg))
    if(ret == 0):
        print("设置定时发送 失败")
    else:
        print("设置定时发送 成功")
    
    on = c_uint(1)
    ret = lib.VCI_SetReference(DevType, DevIdx, ChIdx, CMD_CAN_TTX_CTL, byref(on))
    if (ret == 0):
        print("使能定时发送 失败")
    else:
        print("使能定时发送 成功")

    time.sleep(3)

    on = c_uint(0)
    ret = lib.VCI_SetReference(DevType, DevIdx, ChIdx, CMD_CAN_TTX_CTL, byref(on))
    if (ret == 0):
        print("关闭定时发送 失败")
    else:
        print("关闭定时发送 成功")


# 主函数
if __name__ == "__main__":
    DEVICE_INDEX = c_uint32(0)  # 设备索引
    CHANNELS_INDEX = 0          # 测试发送的通道号
    #IsMerge = 0;                # 是否合并接收

    # 打开设备
    ret = lib.VCI_OpenDevice(USBCANFD, DEVICE_INDEX, 0)
    if ret == 0:
        print("Open device fail")
        exit(0)
    else:
        print("Open device success")

    # 打开通道
    for i in range(MAX_CHANNELS):
        canfd_start(USBCANFD, DEVICE_INDEX, i)  # 初始化通道，并且开启接收线程

    # 测试发送数据
    can_send(USBCANFD, DEVICE_INDEX, CHANNELS_INDEX)

    # 队列发送
    # queue_send(USBCANFD, DEVICE_INDEX, CHANNELS_INDEX)

    # 定时发送
    # auto_send(USBCANFD, DEVICE_INDEX, CHANNELS_INDEX)

    # 阻塞等待
    input()
    g_thd_run = 0

    # 等待所有线程完成
    for thread in threads:
        thread.join()

    for i in range(MAX_CHANNELS):
        ret = lib.VCI_ResetCAN(USBCANFD, DEVICE_INDEX, i)
        if ret == 0:
            print("ResetCAN(%d) fail" % i)
        else:
            print("ResetCAN(%d) success!" % i)

    ret = lib.VCI_CloseDevice(USBCANFD, DEVICE_INDEX)
    if ret == 0:
        print("Closedevice fail!")
    else:
        print("Closedevice success")
    del lib
