from ctypes import *
import platform
import threading
import sys
import os
from pathlib import Path

ZCAN_DEVICE_TYPE = c_uint
ZCAN_RET_STATUS = c_uint
ZCAN_UDS_DATA_DEF = c_uint

INVALID_DEVICE_HANDLE = 0
INVALID_CHANNEL_HANDLE = 0

'''
 Device Type
'''
ZCAN_PCI5121 = ZCAN_DEVICE_TYPE(1)
ZCAN_PCI9810 = ZCAN_DEVICE_TYPE(2)
ZCAN_USBCAN1 = ZCAN_DEVICE_TYPE(3)
ZCAN_USBCAN2 = ZCAN_DEVICE_TYPE(4)
ZCAN_PCI9820 = ZCAN_DEVICE_TYPE(5)
ZCAN_CAN232 = ZCAN_DEVICE_TYPE(6)
ZCAN_PCI5110 = ZCAN_DEVICE_TYPE(7)
ZCAN_CANLITE = ZCAN_DEVICE_TYPE(8)
ZCAN_ISA9620 = ZCAN_DEVICE_TYPE(9)
ZCAN_ISA5420 = ZCAN_DEVICE_TYPE(10)
ZCAN_PC104CAN = ZCAN_DEVICE_TYPE(11)
ZCAN_CANETUDP = ZCAN_DEVICE_TYPE(12)
ZCAN_CANETE = ZCAN_DEVICE_TYPE(12)
ZCAN_DNP9810 = ZCAN_DEVICE_TYPE(13)
ZCAN_PCI9840 = ZCAN_DEVICE_TYPE(14)
ZCAN_PC104CAN2 = ZCAN_DEVICE_TYPE(15)
ZCAN_PCI9820I = ZCAN_DEVICE_TYPE(16)
ZCAN_CANETTCP = ZCAN_DEVICE_TYPE(17)
ZCAN_PCIE_9220 = ZCAN_DEVICE_TYPE(18)
ZCAN_PCI5010U = ZCAN_DEVICE_TYPE(19)
ZCAN_USBCAN_E_U = ZCAN_DEVICE_TYPE(20)
ZCAN_USBCAN_2E_U = ZCAN_DEVICE_TYPE(21)
ZCAN_PCI5020U = ZCAN_DEVICE_TYPE(22)
ZCAN_EG20T_CAN = ZCAN_DEVICE_TYPE(23)
ZCAN_PCIE9221 = ZCAN_DEVICE_TYPE(24)
ZCAN_WIFICAN_TCP = ZCAN_DEVICE_TYPE(25)
ZCAN_WIFICAN_UDP = ZCAN_DEVICE_TYPE(26)
ZCAN_PCIe9120 = ZCAN_DEVICE_TYPE(27)
ZCAN_PCIe9110 = ZCAN_DEVICE_TYPE(28)
ZCAN_PCIe9140 = ZCAN_DEVICE_TYPE(29)
ZCAN_USBCAN_4E_U = ZCAN_DEVICE_TYPE(31)
ZCAN_CANDTU_200UR = ZCAN_DEVICE_TYPE(32)
ZCAN_CANDTU_MINI = ZCAN_DEVICE_TYPE(33)
ZCAN_USBCAN_8E_U = ZCAN_DEVICE_TYPE(34)
ZCAN_CANREPLAY = ZCAN_DEVICE_TYPE(35)
ZCAN_CANDTU_NET = ZCAN_DEVICE_TYPE(36)
ZCAN_CANDTU_100UR = ZCAN_DEVICE_TYPE(37)
ZCAN_PCIE_CANFD_100U = ZCAN_DEVICE_TYPE(38)
ZCAN_PCIE_CANFD_200U = ZCAN_DEVICE_TYPE(39)
ZCAN_PCIE_CANFD_400U = ZCAN_DEVICE_TYPE(40)
ZCAN_USBCANFD_200U = ZCAN_DEVICE_TYPE(41)
ZCAN_USBCANFD_100U = ZCAN_DEVICE_TYPE(42)
ZCAN_USBCANFD_MINI = ZCAN_DEVICE_TYPE(43)
ZCAN_CANFDCOM_100IE = ZCAN_DEVICE_TYPE(44)
ZCAN_CANSCOPE = ZCAN_DEVICE_TYPE(45)
ZCAN_CLOUD = ZCAN_DEVICE_TYPE(46)
ZCAN_CANDTU_NET_400 = ZCAN_DEVICE_TYPE(47)
ZCAN_CANFDNET_200U_TCP = ZCAN_DEVICE_TYPE(48)
ZCAN_CANFDNET_200U_UDP = ZCAN_DEVICE_TYPE(49)
ZCAN_CANFDWIFI_100U_TCP = ZCAN_DEVICE_TYPE(50)
ZCAN_CANFDWIFI_100U_UDP = ZCAN_DEVICE_TYPE(51)
ZCAN_CANFDNET_400U_TCP = ZCAN_DEVICE_TYPE(52)
ZCAN_CANFDNET_400U_UDP = ZCAN_DEVICE_TYPE(53)
ZCAN_CANFDBLUE_200U = ZCAN_DEVICE_TYPE(54)
ZCAN_CANFDNET_100U_TCP = ZCAN_DEVICE_TYPE(55)
ZCAN_CANFDNET_100U_UDP = ZCAN_DEVICE_TYPE(56)
ZCAN_CANFDNET_800U_TCP = ZCAN_DEVICE_TYPE(57)
ZCAN_CANFDNET_800U_UDP = ZCAN_DEVICE_TYPE(58)
ZCAN_USBCANFD_800U = ZCAN_DEVICE_TYPE(59)
ZCAN_PCIE_CANFD_100U_EX = ZCAN_DEVICE_TYPE(60)
ZCAN_PCIE_CANFD_400U_EX = ZCAN_DEVICE_TYPE(61)
ZCAN_PCIE_CANFD_200U_MINI = ZCAN_DEVICE_TYPE(62)
ZCAN_PCIE_CANFD_200U_M2 = ZCAN_DEVICE_TYPE(63)
ZCAN_PCIE_CANFD_200U_EX = ZCAN_DEVICE_TYPE(62)
ZCAN_CANFDDTU_400_TCP = ZCAN_DEVICE_TYPE(64)
ZCAN_CANFDDTU_400_UDP = ZCAN_DEVICE_TYPE(65)
ZCAN_CANFDWIFI_200U_TCP = ZCAN_DEVICE_TYPE(66)
ZCAN_CANFDWIFI_200U_UDP = ZCAN_DEVICE_TYPE(67)
ZCAN_CANFDDTU_800ER_TCP = ZCAN_DEVICE_TYPE(68)
ZCAN_CANFDDTU_800ER_UDP = ZCAN_DEVICE_TYPE(69)
ZCAN_CANFDDTU_800EWGR_TCP = ZCAN_DEVICE_TYPE(70)
ZCAN_CANFDDTU_800EWGR_UDP = ZCAN_DEVICE_TYPE(71)
ZCAN_CANFDDTU_600EWGR_TCP = ZCAN_DEVICE_TYPE(72)
ZCAN_CANFDDTU_600EWGR_UDP = ZCAN_DEVICE_TYPE(73)
ZCAN_CANFDDTU_CASCADE_TCP = ZCAN_DEVICE_TYPE(74)
ZCAN_CANFDDTU_CASCADE_UDP = ZCAN_DEVICE_TYPE(75)
ZCAN_USBCANFD_400U = ZCAN_DEVICE_TYPE(76)
ZCAN_CANFDDTU_200U = ZCAN_DEVICE_TYPE(77)
ZCAN_CANFDBRIDGE_PLUS = ZCAN_DEVICE_TYPE(80)
ZCAN_CANFDDTU_300U = ZCAN_DEVICE_TYPE(81)
ZCAN_VIRTUAL_DEVICE = ZCAN_DEVICE_TYPE(99)

'''
 Interface return status    返回值
'''
ZCAN_STATUS_ERR = 0
ZCAN_STATUS_OK = 1
ZCAN_STATUS_ONLINE = 2
ZCAN_STATUS_OFFLINE = 3
ZCAN_STATUS_UNSUPPORTED = 4

'''
 CAN type   GetReceivenum参数
'''
ZCAN_TYPE_CAN = c_uint(0)
ZCAN_TYPE_CANFD = c_uint(1)
ZCAN_TYPE_MERGE = c_uint(2)

'''
eZCANDataDEF    合并接收结构体类型
'''
# 数据类型
ZCAN_DT_ZCAN_CAN_CANFD_DATA = 1     # CAN/CANFD数据
ZCAN_DT_ZCAN_ERROR_DATA     = 2     #错误数据
ZCAN_DT_ZCAN_GPS_DATA       = 3     #GPS数据
ZCAN_DT_ZCAN_LIN_DATA       = 4     #LIN数据
ZCAN_DT_ZCAN_BUSUSAGE_DATA  = 5     #BusUsage数据
ZCAN_DT_ZCAN_LIN_ERROR_DATA = 6     #LIN错误数据
ZCAN_DT_ZCAN_LIN_EX_DATA    = 7     #LIN扩展数据
ZCAN_DT_ZCAN_LIN_EVENT_DATA = 8     #LIN事件数据

#发送延时单位
ZCAN_TX_DELAY_NO_DELAY   = 0        #无发送延时
ZCAN_TX_DELAY_UNIT_MS    = 1        #发送延时单位毫秒
ZCAN_TX_DELAY_UNIT_100US = 2        #发送延时单位100微秒(0.1毫秒)

'''
 Device information
'''
class ZCAN_DEVICE_INFO(Structure):
    _fields_ = [("hw_Version", c_ushort),
                ("fw_Version", c_ushort),
                ("dr_Version", c_ushort),
                ("in_Version", c_ushort),
                ("irq_Num", c_ushort),
                ("can_Num", c_ubyte),
                ("str_Serial_Num", c_ubyte * 20),
                ("str_hw_Type", c_ubyte * 40),
                ("reserved", c_ushort * 4)]

    def __str__(self):
        return "Hardware Version:%s\nFirmware Version:%s\nDriver Interface:%s\nInterface Interface:%s\nInterrupt Number:%d\nCAN Number:%d\nSerial:%s\nHardware Type:%s\n" % (
            self.hw_version, self.fw_version, self.dr_version, self.in_version, self.irq_num, self.can_num, self.serial,
            self.hw_type)

    def _version(self, version):
        return ("V%02x.%02x" if version // 0xFF >= 9 else "V%d.%02x") % (version // 0xFF, version & 0xFF)

    @property
    def hw_version(self):
        return self._version(self.hw_Version)

    @property
    def fw_version(self):
        return self._version(self.fw_Version)

    @property
    def dr_version(self):
        return self._version(self.dr_Version)

    @property
    def in_version(self):
        return self._version(self.in_Version)

    @property
    def irq_num(self):
        return self.irq_Num

    @property
    def can_num(self):
        return self.can_Num

    @property
    def serial(self):
        serial = ''
        for c in self.str_Serial_Num:
            if c > 0:
                serial += chr(c)
            else:
                break
        return serial

    @property
    def hw_type(self):
        hw_type = ''
        for c in self.str_hw_Type:
            if c > 0:
                hw_type += chr(c)
            else:
                break
        return hw_type


class _ZCAN_CHANNEL_CAN_INIT_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint),
                ("acc_mask", c_uint),
                ("reserved", c_uint),
                ("filter", c_ubyte),
                ("timing0", c_ubyte),
                ("timing1", c_ubyte),
                ("mode", c_ubyte)]


class _ZCAN_CHANNEL_CANFD_INIT_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint),
                ("acc_mask", c_uint),
                ("abit_timing", c_uint),
                ("dbit_timing", c_uint),
                ("brp", c_uint),
                ("filter", c_ubyte),
                ("mode", c_ubyte),
                ("pad", c_ushort),
                ("reserved", c_uint)]


class _ZCAN_CHANNEL_INIT_CONFIG(Union):
    _fields_ = [("can", _ZCAN_CHANNEL_CAN_INIT_CONFIG), ("canfd", _ZCAN_CHANNEL_CANFD_INIT_CONFIG)]


class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [("can_type", c_uint),
                ("config", _ZCAN_CHANNEL_INIT_CONFIG)]


class ZCAN_CHANNEL_ERR_INFO(Structure):
    _fields_ = [("error_code", c_uint),
                ("passive_ErrData", c_ubyte * 3),
                ("arLost_ErrData", c_ubyte)]


class ZCAN_CHANNEL_STATUS(Structure):
    _fields_ = [("errInterrupt", c_ubyte),
                ("regMode", c_ubyte),
                ("regStatus", c_ubyte),
                ("regALCapture", c_ubyte),
                ("regECCapture", c_ubyte),
                ("regEWLimit", c_ubyte),
                ("regRECounter", c_ubyte),
                ("regTECounter", c_ubyte),
                ("Reserved", c_ubyte)]


class ZCAN_CAN_FRAME(Structure):
    _fields_ = [("can_id", c_uint, 32),
                # ("err", c_uint, 1),
                # ("rtr", c_uint, 1),
                # ("eff", c_uint, 1),
                ("can_dlc", c_ubyte),
                ("_pad", c_ubyte),      # 0-开启 1-关闭; bit5 设置发送回显 bit6 设置队列发送时间精度(是否设置0.1ms精度，默认精度是1ms)
                                        # bit7 设置队列发送
                ("_res0", c_ubyte),
                ("_res1", c_ubyte),
                ("data", c_ubyte * 8)]


class ZCAN_CANFD_FRAME(Structure):
    _pack_ = 1
    _fields_ = [("can_id", c_uint),
                # ("err", c_uint, 1),
                # ("rtr", c_uint, 1),
                # ("eff", c_uint, 1),
                ("len", c_ubyte),
                # ("brs", c_ubyte, 1),
                # ("esi", c_ubyte, 1),
                ("flags", c_ubyte),     # 0-开启 1-关闭; bit5 设置发送回显 bit6 设置队列发送时间精度(是否设置0.1ms精度，默认精度是1ms)
                                        # bit7 设置队列发送 bit0 设置加速报文
                ("_res0", c_ubyte),
                ("_res1", c_ubyte),
                ("data", c_ubyte * 64)]


class ZCANdataFlag(Structure):
    _pack_ = 1
    _fields_ = [("frameType", c_uint, 2),
                ("txDelay", c_uint, 2),
                ("transmitType", c_uint, 4),
                ("txEchoRequest", c_uint, 1),
                ("txEchoed", c_uint, 1),
                ("reserved", c_uint, 22),
                ]


class ZCANFDData(Structure):  ##表示 CAN/CANFD 帧结构,目前仅作为 ZCANDataObj 结构的成员使用
    _pack_ = 1
    _fields_ = [("timestamp", c_uint64),
                ("flag", ZCANdataFlag),
                ("extraData", c_ubyte * 4),
                ("frame", ZCAN_CANFD_FRAME), ]


class ZCANDataObj(Structure):
    # _pack_ = 1
    _fields_ = [("dataType", c_ubyte),
                ("chnl", c_ubyte),
                ("flag", c_ushort),
                ("extraData", c_ubyte * 4),
                ("zcanfddata", ZCANFDData),  ##88个字节
                ("reserved", c_ubyte * 4),
                ]

class ZCAN_DYNAMIC_CONFIG_DATA(Structure):
    _pack_ = 1
    _fields_ = [("key",c_char*64),
                ("value",c_char*64)
    ]

class ZCAN_Transmit_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("transmit_type", c_uint)]


class ZCAN_Receive_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("timestamp", c_ulonglong)]


class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("transmit_type", c_uint)]


class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]


class ZCAN_AUTO_TRANSMIT_OBJ(Structure):
    _fields_ = [("enable", c_ushort),
                ("index", c_ushort),
                ("interval", c_uint),
                ("obj", ZCAN_Transmit_Data)]


class ZCANFD_AUTO_TRANSMIT_OBJ(Structure):
    _fields_ = [("enable", c_ushort),
                ("index", c_ushort),
                ("interval", c_uint),
                ("obj", ZCAN_TransmitFD_Data)]


class ZCANFD_AUTO_TRANSMIT_OBJ_PARAM(Structure):  # auto_send delay
    _fields_ = [("indix", c_ushort),
                ("type", c_ushort),
                ("value", c_uint)]


class IProperty(Structure):
    _fields_ = [("SetValue", c_void_p),
                ("GetValue", c_void_p),
                ("GetPropertys", c_void_p)]


'''
// 动态配置 持久配置 BEGIN
#define ZCAN_DYNAMIC_CONFIG_DEVNAME \
    "DYNAMIC_CONFIG_DEVNAME"  // 设备名，最长为32字节（包括’\0’），CANFDNET -
                              // 200U默认值为“CANFDNET - 200U”，CANFDNET -
                              // 100MINI默认值为“CANFDNET - 100MINI”
// CAN的通道配置信息(CAN%d需进行格式化声明通道 范围是0-7)
#define ZCAN_DYNAMIC_CONFIG_CAN_ENABLE \
    "DYNAMIC_CONFIG_CAN%d_ENABLE"  // 通道使能；1：使能，0：失能；CANFDNET系列产品通道默认使能。
#define ZCAN_DYNAMIC_CONFIG_CAN_MODE "DYNAMIC_CONFIG_CAN%d_MODE"  // 工作模式，默认正常模式；0：正常模式；1：只听模式。
#define ZCAN_DYNAMIC_CONFIG_CAN_TXATTEMPTS \
    "DYNAMIC_CONFIG_CAN%d_TXATTEMPTS"  // 发送失败是否重传：0：发送失败不重传1：发送失败重传，直到总线关闭（CANFDNET
                                       // - 100 / 200无此项配置）
#define ZCAN_DYNAMIC_CONFIG_CAN_NOMINALBAUD     "DYNAMIC_CONFIG_CAN%d_NOMINALBAUD"  // CAN波特率或CANFD仲裁域波特率；
#define ZCAN_DYNAMIC_CONFIG_CAN_DATABAUD        "DYNAMIC_CONFIG_CAN%d_DATABAUD"     // CANFD数据域波特率；
#define ZCAN_DYNAMIC_CONFIG_CAN_USERES          "DYNAMIC_CONFIG_CAN%d_USERES"       // 终端电阻开关；0：关闭；1：打开。
#define ZCAN_DYNAMIC_CONFIG_CAN_SNDCFG_INTERVAL "DYNAMIC_CONFIG_CAN%d_SNDCFG_INTERVAL"  // 报文发送间隔，0~255ms
#define ZCAN_DYNAMIC_CONFIG_CAN_BUSRATIO_ENABLE \
    "DYNAMIC_CONFIG_CAN%d_BUSRATIO_ENABLE"  // 总线利用率使能，使能后，将周期发送总线利用率到设定的TCP/UDP连接。1:使能，0：失能
#define ZCAN_DYNAMIC_CONFIG_CAN_BUSRATIO_PERIOD \
    "DYNAMIC_CONFIG_CAN%d_BUSRATIO_PERIOD"  // 总线利用率采集周期，取值200~2000ms
'''

# 设备名，最长为32字节（包括’\0’）
def ZCAN_DYNAMIC_CONFIG_DEVNAME():
    return "DYNAMIC_CONFIG_DEVNAME"

# CAN的通道配置信息(CAN%d需进行格式化声明通道 范围是0-7)
# 通道使能；1：使能，0：失能；CANFDNET系列产品通道默认使能。
def ZCAN_DYNAMIC_CONFIG_CAN_ENABLE(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_ENABLE"

# 工作模式，默认正常模式；0：正常模式；1：只听模式
def ZCAN_DYNAMIC_CONFIG_CAN_MODE(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_MODE"

# 发送失败是否重传：0：发送失败不重传1：发送失败重传，直到总线关闭（CANFDNET- 100 / 200无此项配置）
def ZCAN_DYNAMIC_CONFIG_CAN_TXATTEMPTS(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_TXATTEMPTS"

# CAN波特率或CANFD仲裁域波特率
def ZCAN_DYNAMIC_CONFIG_CAN_NOMINALBAUD(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_NOMINALBAUD"

# CANFD数据域波特率
def ZCAN_DYNAMIC_CONFIG_CAN_DATABAUD(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_DATABAUD"

# 终端电阻开关；0：关闭；1：打开
def ZCAN_DYNAMIC_CONFIG_CAN_USERES(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_USERES"

# 终端电阻开关；0：关闭；1：打开
def ZCAN_DYNAMIC_CONFIG_CAN_USERES(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_USERES"

# 报文发送间隔，0~255ms
def ZCAN_DYNAMIC_CONFIG_CAN_SNDCFG_INTERVAL(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_SNDCFG_INTERVAL"

# 总线利用率使能，使能后，将周期发送总线利用率到设定的TCP/UDP连接。1:使能，0：失能
def ZCAN_DYNAMIC_CONFIG_CAN_BUSRATIO_ENABLE(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_SNDCFG_INTERVAL"

# 总线利用率采集周期，取值200~2000ms
def ZCAN_DYNAMIC_CONFIG_CAN_BUSRATIO_ENABLE(can_id):
    return f"DYNAMIC_CONFIG_CAN{can_id}_SNDCFG_INTERVAL"

class ZCAN(object):
    def __init__(self):
        if platform.system() == "Windows":
            current_dir = Path(__file__).resolve().parent
            dll_path = os.path.join(str(current_dir), "zlgcan.dll")
            self.__dll = windll.LoadLibrary(dll_path)
        else:
            print("No support now!")
            sys.exit(1)
        if self.__dll == None:
            print("DLL couldn't be loaded!")
            sys.exit(1)

    def OpenDevice(self, device_type, device_index, reserved):
        try:
            return self.__dll.ZCAN_OpenDevice(device_type, device_index, reserved)
        except:
            print("Exception on OpenDevice!")
            raise

    def CloseDevice(self, device_handle):
        try:
            return self.__dll.ZCAN_CloseDevice(device_handle)
        except:
            print("Exception on CloseDevice!")
            raise

    def GetDeviceInf(self, device_handle):
        try:
            info = ZCAN_DEVICE_INFO()
            ret = self.__dll.ZCAN_GetDeviceInf(device_handle, byref(info))
            return info if ret == ZCAN_STATUS_OK else None
        except:
            print("Exception on ZCAN_GetDeviceInf")
            raise

    def DeviceOnLine(self, device_handle):
        try:
            return self.__dll.ZCAN_IsDeviceOnLine(device_handle)
        except:
            print("Exception on ZCAN_ZCAN_IsDeviceOnLine!")
            raise

    def InitCAN(self, device_handle, can_index, init_config):
        try:
            return self.__dll.ZCAN_InitCAN(device_handle, can_index, byref(init_config))
        except:
            print("Exception on ZCAN_InitCAN!")
            raise

    def StartCAN(self, chn_handle):
        try:
            return self.__dll.ZCAN_StartCAN(chn_handle)
        except:
            print("Exception on ZCAN_StartCAN!")
            raise

    def ResetCAN(self, chn_handle):
        try:
            return self.__dll.ZCAN_ResetCAN(chn_handle)
        except:
            print("Exception on ZCAN_ResetCAN!")
            raise

    def ClearBuffer(self, chn_handle):
        try:
            return self.__dll.ZCAN_ClearBuffer(chn_handle)
        except:
            print("Exception on ZCAN_ClearBuffer!")
            raise

    def ReadChannelErrInfo(self, chn_handle):
        try:
            ErrInfo = ZCAN_CHANNEL_ERR_INFO()
            ret = self.__dll.ZCAN_ReadChannelErrInfo(chn_handle, byref(ErrInfo))
            return ErrInfo if ret == ZCAN_STATUS_OK else None
        except:
            print("Exception on ZCAN_ReadChannelErrInfo!")
            raise

    def ReadChannelStatus(self, chn_handle):
        try:
            status = ZCAN_CHANNEL_STATUS()
            ret = self.__dll.ZCAN_ReadChannelStatus(chn_handle, byref(status))
            return status if ret == ZCAN_STATUS_OK else None
        except:
            print("Exception on ZCAN_ReadChannelStatus!")
            raise

    def GetReceiveNum(self, chn_handle, can_type=ZCAN_TYPE_CAN):
        try:
            return self.__dll.ZCAN_GetReceiveNum(chn_handle, can_type)
        except:
            print("Exception on ZCAN_GetReceiveNum!")
            raise

    def Transmit(self, chn_handle, std_msg, len):
        try:
            return self.__dll.ZCAN_Transmit(chn_handle, byref(std_msg), len)
        except:
            print("Exception on ZCAN_Transmit!")
            raise

    def Receive(self, chn_handle, rcv_num, wait_time=c_int(-1)):
        try:
            rcv_can_msgs = (ZCAN_Receive_Data * rcv_num)()
            ret = self.__dll.ZCAN_Receive(chn_handle, byref(rcv_can_msgs), rcv_num, wait_time)
            return rcv_can_msgs, ret
        except:
            print("Exception on ZCAN_Receive!")
            raise

    def TransmitFD(self, chn_handle, fd_msg, len):
        try:
            return self.__dll.ZCAN_TransmitFD(chn_handle, byref(fd_msg), len)
        except:
            print("Exception on ZCAN_TransmitFD!")
            raise

    def TransmitData(self, device_handle, msg, len):
        try:
            return self.__dll.ZCAN_TransmitData(device_handle, byref(msg), len)
        except:
            print("Exception on ZCAN_TransmitData!")
            raise

    def ReceiveFD(self, chn_handle, rcv_num, wait_time=c_int(-1)):
        try:
            rcv_canfd_msgs = (ZCAN_ReceiveFD_Data * rcv_num)()
            ret = self.__dll.ZCAN_ReceiveFD(chn_handle, byref(rcv_canfd_msgs), rcv_num, wait_time)
            return rcv_canfd_msgs, ret
        except:
            print("Exception on ZCAN_ReceiveF D!")
            raise

    def ReceiveData(self, device_handle, rcv_num, wait_time=c_int(-1)):
        try:
            rcv_can_data_msgs = (ZCANDataObj * rcv_num)()
            ret = self.__dll.ZCAN_ReceiveData(device_handle, byref(rcv_can_data_msgs), rcv_num, wait_time)
            return rcv_can_data_msgs, ret
        except:
            print("Exception on ZCAN_ReceiveData!")
            raise

    def GetIProperty(self, device_handle):
        try:
            self.__dll.GetIProperty.restype = POINTER(IProperty)
            return self.__dll.GetIProperty(device_handle)
        except:
            print("Exception on ZCAN_GetIProperty!")
            raise

    def SetValue(self, iproperty, path, value):
        try:
            func = CFUNCTYPE(c_uint, c_char_p, c_char_p)(iproperty.contents.SetValue)
            return func(c_char_p(path.encode("utf-8")), c_char_p(value.encode("utf-8")))
        except:
            print("Exception on IProperty SetValue")
            raise

    def SetValue1(self, iproperty, path, value):  #############################
        try:
            func = CFUNCTYPE(c_uint, c_char_p, c_char_p)(iproperty.contents.SetValue)
            return func(c_char_p(path.encode("utf-8")), c_void_p(value))
        except:
            print("Exception on IProperty SetValue")
            raise

    def GetValue(self, iproperty, path):
        try:
            func = CFUNCTYPE(c_char_p, c_char_p)(iproperty.contents.GetValue)
            return func(c_char_p(path.encode("utf-8")))
        except:
            print("Exception on IProperty GetValue")
            raise

    def ReleaseIProperty(self, iproperty):
        try:
            return self.__dll.ReleaseIProperty(iproperty)
        except:
            print("Exception on ZCAN_ReleaseIProperty!")
            raise

    def ZCAN_SetValue(self, device_handle, path, value):
        try:
            self.__dll.ZCAN_SetValue.argtypes = [c_void_p, c_char_p, c_void_p]
            return self.__dll.ZCAN_SetValue(device_handle, path.encode("utf-8"), value)
        except:
            print("Exception on ZCAN_SetValue")
            raise

    def ZCAN_GetValue(self, device_handle, path):
        try:
            self.__dll.ZCAN_GetValue.argtypes = [c_void_p, c_char_p]
            self.__dll.ZCAN_GetValue.restype = c_void_p
            return self.__dll.ZCAN_GetValue(device_handle, path.encode("utf-8"))
        except:
            print("Exception on ZCAN_GetValue")
            raise
