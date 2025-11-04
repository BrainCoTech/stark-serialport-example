'''
    支持的设备有 USBCANFD-100U mini   USBCANFD-100U/200U/400U/800U
'''

from zlgcan import *
import threading
import time

thread_flag = True
print_lock = threading.Lock()   # 线程锁，只是为了打印不冲突
enable_merge_receive = 0        # 合并接收标识

#读取设备信息
def Read_Device_Info(device_handle):

    # 获取设备信息
    info = zcanlib.GetDeviceInf(device_handle)
    print("设备信息: \n%s" % info)

    can_number = info.can_num
    return can_number

# 接收线程
def receive_thread(device_handle,chn_handle):

    # 方便打印对齐 --无实际作用
    CANType_width = len("CANFD加速    ")
    id_width = len(hex(0x1FFFFFFF))

    while thread_flag:
        time.sleep(0.005)
        rcv_num = zcanlib.GetReceiveNum(chn_handle, ZCAN_TYPE_CAN)  # CAN
        if rcv_num:
            if rcv_num > 100 :
                rcv_msg, rcv_num = zcanlib.Receive(chn_handle, 100,100)
            else :
                rcv_msg, rcv_num = zcanlib.Receive(chn_handle, rcv_num, 100)
            with print_lock:
                for msg in rcv_msg[:rcv_num]:
                    can_type = "CAN   "
                    frame = msg.frame
                    direction = "TX" if frame._pad & 0x20 else "RX"
                    frame_type = "扩展帧" if frame.can_id & (1 << 31) else "标准帧"
                    frame_format = "远程帧" if frame.can_id & (1 << 30) else "数据帧"
                    can_id = hex(frame.can_id & 0x1FFFFFFF)

                    if frame.can_id & (1 << 30):
                        data = ""
                        dlc = 0
                    else:
                        dlc = frame.can_dlc
                        data = " ".join([f"{num:02X}" for num in frame.data[:dlc]])

                    print(f"[{msg.timestamp}] CAN{chn_handle & 0xFF} {can_type:<{CANType_width}}\t{direction} ID: {can_id:<{id_width}}\t{frame_type} {frame_format}"
                          f" DLC: {dlc}\tDATA(hex): {data}")

        rcv_canfd_num = zcanlib.GetReceiveNum(chn_handle, ZCAN_TYPE_CANFD)  # CANFD
        if rcv_canfd_num:
            if rcv_num > 100 :
                rcv_canfd_msgs, rcv_canfd_num = zcanlib.ReceiveFD(chn_handle, 100,100)
            else :
                rcv_canfd_msgs, rcv_canfd_num = zcanlib.ReceiveFD(chn_handle, rcv_canfd_num,100)
            with print_lock:
                for msg in rcv_canfd_msgs[:rcv_canfd_num]:
                    frame = msg.frame
                    brs = "加速" if frame.flags & 0x1 else "   "
                    can_type = "CANFD" + brs
                    direction = "TX" if frame.flags & 0x20 else "RX"
                    frame_type = "扩展帧" if frame.can_id & (1 << 31) else "标准帧"
                    frame_format = "远程帧" if frame.can_id & (1 << 30) else "数据帧"     # CANFD没有远程帧
                    can_id = hex(frame.can_id & 0x1FFFFFFF)
                    data = " ".join([f"{num:02X}" for num in frame.data[:frame.len]])

                    print(f"[{msg.timestamp}] CAN{chn_handle & 0xFF} {can_type:<{CANType_width}}\t{direction} ID: {can_id:<{id_width}}\t{frame_type} {frame_format}"
                          f" DLC: {frame.len}\tDATA(hex): {data}")

        rcv_merge_num = zcanlib.GetReceiveNum(device_handle, ZCAN_TYPE_MERGE)  # CANFD
        if rcv_merge_num:
            if rcv_num > 100 :
                rcv_merger_msgs, rcv_merge_num = zcanlib.ReceiveData(device_handle, 100,100)
            else :
                rcv_merger_msgs, rcv_merge_num = zcanlib.ReceiveData(device_handle, rcv_merge_num, 100)
            with print_lock:
                for msg in rcv_merger_msgs[:rcv_merge_num]:
                    if msg.dataType == ZCAN_DT_ZCAN_CAN_CANFD_DATA:
                        flag = msg.zcanfddata.flag
                        frame = msg.zcanfddata.frame
                        type = "CANFD" if flag.frameType else "CAN"
                        brs = "加速" if  (frame.flags & 0x1) else "   "
                        can_type = type + brs
                        direction = "TX" if msg.zcanfddata.flag.txEchoed else "RX"
                        frame_type = "扩展帧" if frame.can_id & (1 << 31) else "标准帧"
                        frame_format = "远程帧" if frame.can_id & (1 << 30) else "数据帧"
                        can_id = frame.can_id & 0x1FFFFFFF
                        data = " ".join([f"{num:02X}" for num in frame.data[:frame.len]])

                        print(f"[{msg.zcanfddata.timestamp}] CAN{msg.chnl} {can_type:<{CANType_width}}\t{direction} ID: {hex(can_id):<{id_width}}\t{frame_type} {frame_format}"
                        f" DLC: {frame.len}\tDATA(hex): {data}")
    print("=====")

# 设置滤波  白名单过滤(只接收范围内的数据)    USBCANFD设备支持设置最多64组滤波
def Set_Filter(device_handle,chn):

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/filter_clear", "0".encode("utf-8")) # 清除滤波
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  filter_clear failed!" % chn)
        return None

    # 流程为：for【set_mode(与前一组滤波同类型可以省略) + set_start + set_end】+ ack
    # 这里设置第一道滤波：标准帧0~0x7F，第二道滤波：标准帧0xFF~0x1FF，第三道滤波：扩展帧0xFF~0x2FF
    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/filter_mode", "0".encode("utf-8"))  # 设置滤波模式 标准帧滤波
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  filter_mode failed!" % chn)
        return None

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/filter_start", "0".encode("utf-8")) # 设置白名单范围 起始ID
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  filter_start failed!" % chn)
        return None

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/filter_end", "0x7F".encode("utf-8"))   # 设置白名单范围 结束ID
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  filter_end failed!" % chn)
        return None

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/filter_start", "0xFF".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  filter_start failed!" % chn)
        return None

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/filter_end", "0x1FF".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  filter_end failed!" % chn)
        return None

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/filter_mode", "1".encode("utf-8"))  # 扩展帧滤波
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  filter_mode failed!" % chn)
        return None

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/filter_start", "0xFF".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  filter_start failed!" % chn)
        return None

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/filter_end", "0x2FF".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  filter_end failed!" % chn)
        return None

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/filter_ack", "0".encode("utf-8"))  # 使能滤波
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  filter_ack failed!" % chn)
        return None

# 启动通道
def USBCANFD_Start(zcanlib, device_handle, chn):

    # 设置控制器设备类型，如果不是 Non-ISO CANFD 可注释掉按设备默认即可 CAN/CANFD 都是默认
    # ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/canfd_standard", "0".encode("utf-8"))

    # 仲裁域波特率 和 数据域波特率
    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/canfd_abit_baud_rate", "500000".encode("utf-8"))
    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/canfd_dbit_baud_rate", "2000000".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d baud failed!" % chn)
        return None

    # 自定义波特率    当产品波特率对采样点有要求，或者需要设置非常规波特率时使用   ---默认不管
    # ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/baud_rate_custom", "500Kbps(80%),2.0Mbps(80%),(80,07C00002,01C00002)".encode("utf-8"))
    # if ret != ZCAN_STATUS_OK:
    #     print("Set CH%d baud failed!" % chn)
    #     return None

    # 终端电阻
    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/initenal_resistance", "1".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Open CH%d resistance failed!" % chn)
        return None

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
    ret = zcanlib.ZCAN_SetValue(device_handle,str(chn)+"/set_device_tx_echo","0".encode("utf-8"))   #发送回显设置，0-禁用，1-开启
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d  set_device_tx_echo failed!" %(chn))
        return None

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

# 定时发送 设置   每通道最多100条   老卡无法在使能队列发送 的情况下，启动定时发送！！！
def Auto_Send_test(device_handle,chn):

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/clear_auto_send", "0".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Clear CH%d USBCANFD AutoSend failed!" % (chn))
        exit(0)

    auto_can = ZCAN_AUTO_TRANSMIT_OBJ()
    memset(addressof(auto_can), 0, sizeof(auto_can))
    auto_can.index = 0      # 定时发送序列号 用于标记这条报文
    auto_can.enable = 1     # 使能该条定时发送 0-关闭 1-启用
    auto_can.interval = 100 # 定时周期，单位ms

    # auto_can.obj 同 ZCAN_Transmit_Data结构体
    auto_can.obj.transmit_type = 0
    auto_can.obj.frame.can_id = 17  # id
    # auto_can.obj.frame.can_id |= 1 << 31    # 扩展帧
    auto_can.obj.frame.can_dlc = 8  # 数据长度
    auto_can.obj.frame._pad |= 0x20  # 发送回显
    for j in range(auto_can.obj.frame.can_dlc):
        auto_can.obj.frame.data[j] = j

    ret = zcanlib.ZCAN_SetValue(device_handle,str(chn)+"/auto_send",byref(auto_can))
    if ret != ZCAN_STATUS_OK:
        print("设置定时发送 CAN%d 失败!" % chn)
        return None

    auto_canfd = ZCANFD_AUTO_TRANSMIT_OBJ()
    memset(addressof(auto_canfd), 0, sizeof(auto_canfd))
    auto_canfd.index = 1        # 定时发送序列号 用于标记这条报文
    auto_canfd.enable = 1       # 使能该条定时发送 0-关闭 1-启用
    auto_canfd.interval = 500   # 定时周期，单位ms
    auto_canfd.obj.transmit_type = 0  # 0-正常发送，2-自发自收
    auto_canfd.obj.frame.can_id = 0x7F  # ID
    auto_canfd.obj.frame.can_id |= 1 << 31
    auto_canfd.obj.frame.len = 64  # 长度
    # auto_canfd.obj.frame.flags |= 0x20  # 发送回显
    for j in range(auto_canfd.obj.frame.len):
        auto_canfd.obj.frame.data[j] = j

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/auto_send", byref(auto_canfd))
    if ret != ZCAN_STATUS_OK:
        print("设置定时发送 CANFD%d 失败!" % chn)
        return None

    # 使能所有定时发送报文
    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/apply_auto_send", "0".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Apply CH%d USBCANFD AutoSend failed!" % (chn))
        return None

# 关闭发送任务 即关闭 队列发送 和 定时发送
def Clear_Send_Task(device_handle,chn):

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/clear_auto_send", "0".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Clear CH%d AutoSend failed!" % (chn))
        exit(0)

    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/clear_delay_send_queue", "0".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Clear CH%d QueueSend failed!" % (chn))
        exit(0)

# 发送示例
def Transmit_Test(chn_handle):
    # 发送 CAN 报文
    transmit_num = 10
    msgs = (ZCAN_Transmit_Data * transmit_num)()
    for i in range(transmit_num):
        msgs[i].transmit_type = 0       # 0-正常发送，2-自发自收
        msgs[i].frame.can_id = 10       # 发送id
        msgs[i].frame.can_id |= 1<<31   # 最高位(bit31)为 扩展帧/标准帧 标识位 同理 bit30为 数据帧/远程帧
        msgs[i].frame.can_dlc = 8       # 数据长度
        msgs[i].frame._pad |= 0x20      # 发送回显
        msgs[i].frame._res0 = 10        # res0，res1共同表示队列发送间隔(可以理解为一个short 2Byte分开传入)
        msgs[i].frame._res1 = 0
        for j in range(msgs[i].frame.can_dlc):
            msgs[i].frame.data[j] = j
    ret = zcanlib.Transmit(chn_handle, msgs, transmit_num)
    with print_lock: print("成功发送 %d 条CAN报文" % ret)

    # 发送 CANFD 报文
    transmit_canfd_num = 10
    canfd_msgs = (ZCAN_TransmitFD_Data * transmit_canfd_num)()
    for i in range(transmit_num):
        canfd_msgs[i].transmit_type = 0     # 0-正常发送，2-自发自收
        canfd_msgs[i].frame.can_id = 0x1ffffff0     # ID
        canfd_msgs[i].frame.can_id |= 1 << 31
        canfd_msgs[i].frame.len = 64         # 长度
        canfd_msgs[i].frame.flags |= 0x20    # 发送回显
        canfd_msgs[i].frame.flags |= 0x1    # BRS 加速标志位：0不加速，1加速
        canfd_msgs[i].frame._res0 = 10
        for j in range(canfd_msgs[i].frame.len):
            canfd_msgs[i].frame.data[j] = j
    ret = zcanlib.TransmitFD(chn_handle, canfd_msgs, transmit_canfd_num)
    with print_lock: print("成功发送 %d 条CANFD报文" % ret)

# 队列发送示例
def Queue_Transmit_Test(device_handle,chn,chn_handle):

    # 队列发送  设置使能队列发送 0-关闭 1-使能  老卡需要这里使能队列发送+结构体中使能队列发送，新卡只需要执行后者
    ret = zcanlib.ZCAN_SetValue(device_handle, str(chn) + "/set_send_mode", "1".encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d Queue Mode failed!" % chn)
        return None

    # 发送 CAN 报文
    transmit_num = 10
    msgs = (ZCAN_Transmit_Data * transmit_num)()
    for i in range(transmit_num):
        msgs[i].transmit_type = 0  # 0-正常发送，2-自发自收
        msgs[i].frame.can_id = 10  # 发送id
        msgs[i].frame.can_id |= 1 << 31  # 最高位(bit31)为 扩展帧/标准帧 标识位 同理 bit30为 数据帧/远程帧
        msgs[i].frame.can_dlc = 8  # 数据长度
        msgs[i].frame._pad |= 0x20  # 发送回显
        msgs[i].frame._pad |= 1<<7      # 设置队列发送
        msgs[i].frame._res0 = 10  # 10ms res0，res1共同表示队列发送间隔(可以理解为一个short 2Byte分开传入)
        msgs[i].frame._res1 = 0
        for j in range(msgs[i].frame.can_dlc):
            msgs[i].frame.data[j] = j
    ret = zcanlib.ZCAN_GetValue(device_handle, str(chn) + "/get_device_available_tx_count/1")
    if ret < transmit_num:
        ret = 0
    else:
        ret = zcanlib.Transmit(chn_handle, msgs, transmit_num)
    with print_lock:
        print("队列发送--成功发送 %d 条CAN报文" % ret)

    # 发送 CANFD 报文
    transmit_canfd_num = 10
    canfd_msgs = (ZCAN_TransmitFD_Data * transmit_canfd_num)()
    for i in range(transmit_num):
        canfd_msgs[i].transmit_type = 0  # 0-正常发送，2-自发自收
        canfd_msgs[i].frame.can_id = 0x1ffffff0  # ID
        canfd_msgs[i].frame.can_id |= 1 << 31
        canfd_msgs[i].frame.len = 64  # 长度
        canfd_msgs[i].frame.flags |= 0x20    # 发送回显
        canfd_msgs[i].frame.flags |= 0x1  # BRS 加速标志位：0不加速，1加速
        canfd_msgs[i].frame.flags |= 1 << 7  # 设置队列发送
        canfd_msgs[i].frame._res0 = 0
        canfd_msgs[i].frame._res0 = 1   # 256ms
        for j in range(canfd_msgs[i].frame.len):
            canfd_msgs[i].frame.data[j] = j

    # 获取队列发送缓存
    ret = zcanlib.ZCAN_GetValue(device_handle, str(chn) + "/get_device_available_tx_count/1")
    if ret < transmit_num:
        ret = 0
    else:
        ret = zcanlib.TransmitFD(chn_handle, canfd_msgs, transmit_canfd_num)
    with print_lock:
        print("队列发送--成功发送 %d 条CANFD报文" % ret)

# 合并发送示例    --- 将所有通道的所有数据，都用同一结构体，同一函数
def Merge_Transmit_Test(device_handle,chn,chn_handle):

    DataObj = (ZCANDataObj * 10)()
    memset(DataObj, 0, sizeof(DataObj))
    for i in range(10):
        DataObj[i].dataType = 1  # can/canfd frame
        DataObj[i].chnl = 0  # can_channel
        DataObj[i].zcanfddata.flag.frameType = 0  # 0-can,1-canfd
        DataObj[i].zcanfddata.flag.txDelay = 0  # 不添加延迟
        DataObj[i].zcanfddata.flag.transmitType = 0  # 发送方式，0-正常发送，2-自发自收
        DataObj[i].zcanfddata.flag.txEchoRequest = 1  # 发送回显请求，0-不回显，1-回显
        DataObj[i].zcanfddata.frame.can_id = i
        DataObj[i].zcanfddata.frame.len = 61
        DataObj[i].zcanfddata.frame.brs = 0  # 不加速
        for j in range(DataObj[i].zcanfddata.frame.len):
            DataObj[i].zcanfddata.frame.data[j] = j
    ret = zcanlib.TransmitData(handle, DataObj, 10)
    with print_lock:
        print("合并发送--成功发送 %d 条CAN报文" % ret)

# 设置自定义序列号  ---用去区分同型号的CAN卡
def Set_Device_Name(device_handle):
    ret = zcanlib.ZCAN_SetValue(handle, "0/set_cn", "A001".encode("utf-8"))
    if ret == ZCAN_STATUS_OK:
        t = zcanlib.ZCAN_GetValue(handle, "0/get_cn/1")
        print(c_char_p(t).value.decode("utf-8"))
    return

if __name__ == "__main__":
    zcanlib = ZCAN()

    # 打开设备
    handle = zcanlib.OpenDevice(ZCAN_USBCANFD_100U, 0, 0)
    if handle == INVALID_DEVICE_HANDLE:
        print("打开设备失败！")
        exit(0)
    print("设备句柄: %d." % handle)

    # 获取设备信息
    can_number = Read_Device_Info(handle)

    # 设置自定义序列号
    Set_Device_Name(handle)

    # 启动通道
    chn_handles = []
    threads = []
    for i in range(can_number):
        chn_handle = USBCANFD_Start(zcanlib, handle, i)
        if chn_handle is None:
            print("启动通道%d失败！" % i)
            exit(0)
        chn_handles.append(chn_handle)  # 将通道句柄添加到列表中
        print("通道句柄: %d." % chn_handle)

    if enable_merge_receive == 1:   #   若开启合并接收，所有通道都由一个接收线程处理
        thread = threading.Thread(target=receive_thread, args=(handle,chn_handles[0]))    # 开启独立接收线程
        threads.append(thread)
        thread.start()
    else:                           #   若没有开启合并接收，所有通道的数据需要各自开一个线程接收
        for i in range(len(chn_handles)):
            thread = threading.Thread(target=receive_thread, args=(handle, chn_handles[i]))  # 开启独立接收线程
            threads.append(thread)
            thread.start()

    # 发送报文示例
    Transmit_Test(chn_handles[0])

    # 定时发送示例
    # Auto_Send_test(handle,0)

    # 队列发送示例
    # Queue_Transmit_Test(handle,0,chn_handles[0])

    # 合并发送示例
    # Merge_Transmit_Test(handle, 0, chn_handles[0])

    # 回车退出
    input()
    thread_flag = False

    # 关闭 定时发送/队列发送 任务
    Clear_Send_Task(handle, 0)

    # 关闭接收线程
    if enable_merge_receive == 1:
        threads[0].join()
    else:
        for i in range(len(chn_handles)):
            threads[i].join()

    # 关闭通道
    for i in range(len(chn_handles)):
        ret = zcanlib.ResetCAN(chn_handles[i])
        if ret == 1:
            print(f"关闭通道{i}成功")

    # 关闭设备
    ret = zcanlib.CloseDevice(handle)
    if ret == 1:
        print("关闭设备成功")
