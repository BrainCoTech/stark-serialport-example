import abc
import os
import platform
import numpy as np
from enum import IntEnum  # Enum declarations
from cffi import FFI
import pathlib
from stark_logger import SRLOG
 
ffi = FFI()

def fatal_error(msg):
    SRLOG.LOG_ERROR(f"FATAL_ERROR: {msg}")

def load_library():
    current_dir = pathlib.Path(__file__).resolve()
    parent_dir = current_dir.parent.parent
    lib_dir = os.path.join(parent_dir, "dist")
    SRLOG.LOG_INFO(f"Loading StarkSDK from {lib_dir}")

    # 1. load header
    with open(os.path.join(lib_dir, "include", "stark_sdk.h"), encoding='utf-8') as sdk_header:
        sdk_header = sdk_header.read() \
            .split("CFFI_DEF_START")[1] \
            .split("CFFI_DEF_END")[0] \
            .replace("SDK_EXTERN ", "") \
            .replace("#if defined(_WIN32) || TARGET_OS_OSX", "") \
            .replace("#endif", "")
        ffi.cdef(sdk_header) 

    # 2. find library path and load
    arch = platform.architecture()[0]
    if platform.system() == "Darwin":
        return ffi.dlopen(os.path.join(lib_dir, "mac", "shared", "libstark.dylib"))
    elif platform.system() == "Linux":
        return ffi.dlopen(os.path.join(lib_dir, "linux", "shared", "libstark.so"))
    elif platform.system() == "Windows" and arch == "64bit":
        lib_path = os.path.join(lib_dir, "win", "shared")
        # add path 'python/libstark/' to environment variable 'PATH' to load the dependent DLLs.
        os.environ["PATH"] += os.pathsep + lib_path
        dll_path = os.path.join(lib_path, "stark.dll")
        SRLOG.LOG_INFO(f"""Loading StarkSDK from {dll_path}""")
        return ffi.dlopen(dll_path)
    else:
        raise Exception("Unsupported platform: " + platform.system() + ", arch: " + arch)
    
# load StarkSDK library
libstark = load_library()    
SRLOG.LOG_INFO(f"StarkSDK loaded {libstark}")

class LogLevel(IntEnum):
    debug = 0
    info = 1
    warning = 2
    error = 3
    none = 4

class StarkMessageId(IntEnum):
    MainBoard = 1
    MotorBoard = 2
    App = 3

class FingerId(IntEnum):
    thumb = 1
    thumb_aux = 2
    index = 3
    middle = 4
    ring = 5
    pinky = 6

class HandType(IntEnum):
    medium_right = 1
    medium_left = 2
    small_right = 3
    small_left = 4

class ForceLevel(IntEnum):
    small = 1
    normal = 2
    full = 3

class FingerMovementState(IntEnum):
    idle = 0
    running = 1
    protected_zone = 2
    stall = 3

class LedMode(IntEnum):
    off = 1
    keep = 2
    blink = 3
    one_shot = 4
    blink0_5hz = 5
    blink2hz = 6

class LedColor(IntEnum):
    unchanged = 0
    r = 1
    g = 2
    rg = 3
    b = 4
    rb = 5
    gb = 6
    rgb = 7    
    
class ButtonPressState(IntEnum):
    pressing = 1
    not_pressing = 2

class StarkErrorCode(IntEnum):
    none = 0
    unknown = -1
    invalid_params = -2
    invalid_data = -3
    parse_failed = -4

class StarkError:
    def __init__(self, code=None, message=None):
        if code is not None:
            self.code = code
            if message is not None:
                self.message = message
            else: 
                c_msg = libstark.stark_err_code_to_msg(code)
                self.message = ffi.string(c_msg).decode("utf-8")
        else:
            raise ValueError("Either code or message must be provided")
class StarkSDK:
    __on_error = None

    @staticmethod
    def set_log_level(level=LogLevel.info):
        return libstark.stark_set_log_level(level.value)

    @staticmethod
    def set_error_callback(cb):   
        StarkSDK.__on_error = cb    

    @staticmethod 
    def run_error_callback(error):
        if StarkSDK.__on_error is not None:
            StarkSDK.__on_error(error)    
    
    @staticmethod
    def dispose():
        ffi.dlclose(libstark)
        # os.kill(os.getpid(), signal.SIGKILL)

    @staticmethod
    def get_sdk_version():
        return ffi.string(libstark.stark_get_sdk_version()).decode("utf-8")
    
    @staticmethod
    def gen_msg_id():
        return libstark.stark_gen_msg_id()     

    @staticmethod
    def set_write_data_callback(cb):   
        StarkSDK.__on_write_data = cb
        libstark.stark_set_write_data_callback(StarkSDK.__on_write_data_internal);  

    @staticmethod
    @ffi.callback("int(char*, uint8_t*, int)")
    def __on_write_data_internal(uuid_ptr, data, length):
        if StarkSDK.__on_write_data is not None:
            # 使用 ffi.buffer 创建一个引用C内存区域的缓冲区对象并转换为新的Python字节数组
            value = ffi.buffer(data, length)[:]
            return StarkSDK.__on_write_data(value)
        return -1 
    
    @staticmethod
    def set_finger_positions(finger_positions):
        if not isinstance(finger_positions, list) or len(finger_positions) != 6:
            StarkSDK.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid finger_positions:" + str(finger_positions)))
            return
        # check if all values are in range
        for pos in finger_positions:
            if pos < 0 or pos > 100:
                StarkSDK.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid finger_positions:" + str(finger_positions)))
                return
        libstark.stark_group_set_finger_positions(ffi.new("int[]", finger_positions)) 
        
    @staticmethod
    def set_finger_speeds(finger_speeds):
        if not isinstance(finger_speeds, list) or len(finger_speeds) != 6:
            StarkSDK.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid finger_speeds:" + str(finger_speeds)))
            return
        # check if all values are in range
        for speed in finger_speeds:
            if speed < -100 or speed > 100:
                StarkSDK.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid finger_speeds:" + str(finger_speeds)))
                return
        libstark.stark_group_set_finger_speeds(ffi.new("int[]", finger_speeds)) 

SRLOG.LOG_INFO(f"StarkSDK Version: v{StarkSDK.get_sdk_version()}")        

class SerialPortCfg:  
    serial_device_id = None
    baudrate = None

    def __init__(self, c_cfg):
        self.serial_device_id = c_cfg.serial_device_id
        self.baudrate = c_cfg.baudrate

class MotorboardInfo:
    hand_type = None
    sn = None
    fw_version = None

    def __init__(self, c_info):
        self.hand_type = HandType(c_info.hand_type)
        self.sn = ffi.string(c_info.sn, 40).decode("utf-8")
        self.fw_version = ffi.string(c_info.fw_version, 40).decode("utf-8")

class StarkFingerStatus:
    finger_positions = None
    finger_speeds = None
    finger_currents = None
    finger_pwms = None

    def __init__(self, c_status):
        self.finger_positions = ffi.unpack(c_status.finger_positions, 6)
        self.finger_speeds = ffi.unpack(c_status.finger_speeds, 6)
        self.finger_currents = ffi.unpack(c_status.finger_currents, 6)
        self.finger_pwms = ffi.unpack(c_status.finger_pwms, 6)    

    def __str__(self):
        return f"finger_positions: {self.finger_positions}, finger_speeds: {self.finger_speeds}, finger_currents: {self.finger_currents}, finger_pwms: {self.finger_pwms}"    

class StarkFingerStatusData:
    n_finger_status = 0        
    finger_status = []   

    def __init__(self, c_data):
        self.n_finger_status = c_data.n_finger_status
        self.finger_status = []
        for i in range(self.n_finger_status):
            self.finger_status.append(StarkFingerStatus(c_data.finger_status[i]))

    def __str__(self):
        if self.n_finger_status > 0:
            return f"n_finger_status: {self.n_finger_status}, finger_status: {self.finger_status[0]}"     
        return f"n_finger_status: {self.n_finger_status}, finger_status: []"   

class StarkFingerMovementData:
    n_finger_movement_status = 0        
    finger_movement_status = []   

    def __init__(self, c_data):
        self.n_finger_movement_status = c_data.n_finger_movement_status
        self.finger_movement_status = ffi.unpack(c_data.finger_movement_status, 6)      

    def __str__(self) -> str:
        return f"n_finger_movement_status: {self.n_finger_movement_status}, finger_movement_status: {self.finger_movement_status}"          
class LedInfo:
    led_color = None
    led_mode = None

    def __init__(self, c_info):
        self.led_color = LedColor(c_info.led_color)
        self.led_mode = LedMode(c_info.led_mode)

class ButtonPressEvent:
    timestamp = None
    button_id = None
    press_status = None

    def __init__(self, c_info):
        self.timestamp = c_info.timestamp
        self.button_id = c_info.button_id
        self.press_status = ButtonPressState(c_info.press_status)     

    def __str__(self):
        return f"timestamp: {self.timestamp}, button_id: {self.button_id}, is_pressing: {self.press_status == ButtonPressState.pressing.value}" 

class StarkDeviceListener(abc.ABC):
    def on_error(self, error):
        pass
    def on_serialport_cfg(self, serialport_cfg):
        pass
    def on_hand_type(self, hand_type):
        pass
    def on_force_level(self, force_level):
        pass
    def on_motorboard_info(self, motorboard_info):
        pass
    def on_limit_current(self, limit_current):
        pass    
    def on_voltage(self, voltage):
        pass
    def on_finger_status(self, finger_status):
        pass
    def on_finger_movement_status(self, finger_movement_status):
        pass
    def on_button_event(self, button_event):
        pass       

class StarkDevice(StarkDeviceListener):
    @classmethod
    def create_serial_device(cls, id, name):
        uuid = str(id)
        if uuid in cls._device_map:
            device = cls._device_map.get(uuid)
            return device
        device = StarkDevice(uuid, name, id)
        cls._device_map[uuid] = device
        
        device_ptr = libstark.stark_create_device(uuid.encode('utf-8'), id)
        if device_ptr is not ffi.NULL:
            StarkDevice._device_pointer_map[uuid] = device_ptr
            libstark.stark_set_error_callback(device_ptr, StarkDevice.__on_error_internal)
        return device

    _device_pointer_map = {}
    _device_map = {}
    _config_response_callbacks = {}

    __address = 0
    __uuid = None
    __name = None
    __listener = None

    def __init__(self, uuid, name, address):
        self.__uuid = uuid
        self.__name = name
        self.__address = address

    @property
    def uuid(self):
        return self.__uuid

    @property
    def addr(self):
        return self.__address

    @property
    def name(self):
        return self.__name

    def run_error_callback(self, error):
        if self.__listener is not None and self.__listener.on_error is not None:
            self.__listener.on_error(error)

    def set_listener(self, listener):
        if isinstance(listener, StarkDeviceListener):
            self.__listener = listener        

    def did_receive_data(self, data):  
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_did_receive_data(StarkDevice._device_pointer_map[self.__uuid], ffi.new("uint8_t[]", data), len(data))        

    def get_serialport_cfg(self, cb=None):
        self.__on_serialport_cfg = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_serialport_cfg(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_serialport_cfg_internal)

    def set_serialport_cfg(self, baudrate=115200):
        # 115200/57600/19200
        if baudrate < 19200 or baudrate > 115200:
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid baudrate:" + str(baudrate)))
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_serialport_cfg(StarkDevice._device_pointer_map[self.__uuid], baudrate)           

    def set_serial_device_id(self, device_id):
        if device_id < 10 or device_id > 253:
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid device_id:" + str(device_id)))
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_serial_device_id(StarkDevice._device_pointer_map[self.__uuid], device_id)

    def factory_set_device_sn(self, operation_key, board_sn):
        if not isinstance(operation_key, str) or operation_key == '':
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid operation_key:" + operation_key))
            return
        if not isinstance(board_sn, str) or board_sn == '':
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid board_sn:" + board_sn))
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.factory_set_device_sn(StarkDevice._device_pointer_map[self.__uuid], operation_key.encode('utf-8'), board_sn.encode('utf-8'))        
    
    def factory_set_hand_type(self, operation_key, hand_type=HandType.medium_left):
        if not isinstance(operation_key, str) or operation_key == '':
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid operation_key:" + operation_key))
            return
        if not isinstance(hand_type, HandType):
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid hand_type:" + str(hand_type)))
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.factory_set_hand_type(StarkDevice._device_pointer_map[self.__uuid], operation_key.encode('utf-8'), hand_type.value)

    def get_hand_type(self, cb=None):
        self.__on_hand_type = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_hand_type(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_hand_type_internal)        

    def get_motorboard_info(self, cb=None):
        self.__on_motorboard_info = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_motorboard_info(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_motorboard_info_internal)

    def get_voltage(self, cb=None):
        self.__on_voltage = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_voltage(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_voltage_internal)

    def get_max_current(self, cb=None):
        self.__on_limit_current = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_max_current(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_limit_current_internal)

    def set_max_current(self, max_current):
        # max value is 2000mA
        if max_current < 0 or max_current > 2000000:
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid max_current:" + str(max_current)))
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_max_current(StarkDevice._device_pointer_map[self.__uuid], max_current)

    def get_force_level(self, cb=None):
        self.__on_force_level = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_force_level(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_force_level_internal)                     

    def set_force_level(self, force_level=ForceLevel.normal):
        if not isinstance(force_level, ForceLevel):
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid force_level:" + str(force_level)))
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_force_level(StarkDevice._device_pointer_map[self.__uuid], force_level.value)
    
    def get_finger_status(self, cb=None):
        self.__on_finger_status = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_finger_status(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_finger_status_internal)

    def get_finger_movement_status(self, cb=None):
        self.__on_finger_movement_status = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_finger_movement_status(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_finger_movement_status_internal)

    def reset_finger_positions(self):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_reset_finger_positions(StarkDevice._device_pointer_map[self.__uuid])

    def set_finger_position(self, finger_position):
        # 0 表示张开手指，100 表示闭合手指
        if finger_position < 0 or finger_position > 100:
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid finger_position:" + str(finger_position)))
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_position(StarkDevice._device_pointer_map[self.__uuid], finger_position)

    def set_finger_positions(self, finger_positions):
        if not isinstance(finger_positions, list) or len(finger_positions) != 6:
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid finger_positions:" + str(finger_positions)))
            return
        # check if all values are in range
        for pos in finger_positions:
            if pos < 0 or pos > 100:
                self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid finger_positions:" + str(finger_positions)))
                return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_positions(StarkDevice._device_pointer_map[self.__uuid], ffi.new("int[]", finger_positions))

    def set_finger_speed(self, finger_speed):
        if finger_speed < -100 or finger_speed > 100:
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid finger_speed:" + str(finger_speed)))
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_speed(StarkDevice._device_pointer_map[self.__uuid], finger_speed)

    def set_finger_speeds(self, finger_speeds):
        if not isinstance(finger_speeds, list) or len(finger_speeds) != 6:
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid finger_speeds:" + str(finger_speeds)))
            return
        # check if all values are in range
        for speed in finger_speeds:
            if speed < -100 or speed > 100:
                self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid finger_speeds:" + str(finger_speeds)))
                return 
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_speeds(StarkDevice._device_pointer_map[self.__uuid], ffi.new("int[]", finger_speeds))

    def set_led_info(self, mode=LedMode.blink, color=LedColor.unchanged):
        if not isinstance(mode, LedMode) or not isinstance(color, LedColor):
            self.run_error_callback(StarkError(StarkErrorCode.invalid_params, "Invalid mode or color"))
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_led_info(StarkDevice._device_pointer_map[self.__uuid], mode.value, color.value)

    def get_button_event(self, cb=None):
        self.__on_button_event = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_button_event(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_button_event_internal)

    @staticmethod
    @ffi.callback("void(char*, int)")  
    def __on_error_internal(uuid_ptr, c_error):
        error = StarkError(c_error)
        if uuid_ptr is None:
            StarkSDK.run_error_callback(error)
            return

        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            device.run_error_callback(error)
        else:    
            fatal_error("__on_error_internal:device unavailable for:" + uuid)      

    @staticmethod
    @ffi.callback("void(char*, MotorboardInfo*)")         
    def __on_motorboard_info_internal(uuid_ptr, c_info):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            info = MotorboardInfo(c_info)
            if device.__on_motorboard_info is not None:
                device.__on_motorboard_info(info)
            if device.__listener is not None and device.__listener.on_motorboard_info is not None:
                device.__listener.on_motorboard_info(info)
        else:
            fatal_error("__on_motorboard_info_internal:device unavailable for:" + uuid)  

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_hand_type_internal(uuid_ptr, c_hand_type):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            hand_type = HandType(c_hand_type)
            if device.__on_hand_type is not None:
                device.__on_hand_type(hand_type)
            if device.__listener is not None and device.__listener.on_hand_type is not None:
                device.__listener.on_hand_type(hand_type)
        else:
            fatal_error("__on_hand_type_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, SerialPortCfg*)")
    def __on_serialport_cfg_internal(uuid_ptr, c_cfg):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            serialport_cfg = SerialPortCfg(c_cfg)
            if device.__on_serialport_cfg is not None:
                device.__on_serialport_cfg(serialport_cfg)
            if device.__listener is not None and device.__listener.on_serialport_cfg is not None:
                device.__listener.on_serialport_cfg(serialport_cfg)
        else:
            fatal_error("__on_serialport_cfg_internal:device unavailable for:" + uuid)                

    @staticmethod
    @ffi.callback("void(char*, float)")
    def __on_voltage_internal(uuid_ptr, voltage):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_voltage is not None:
                device.__on_voltage(voltage)
            if device.__listener is not None and device.__listener.on_voltage is not None:
                device.__listener.on_voltage(voltage)
        else:
            fatal_error("__on_voltage_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_limit_current_internal(uuid_ptr, limit_current):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_limit_current is not None:
                device.__on_limit_current(limit_current)
            if device.__listener is not None and device.__listener.on_limit_current is not None:
                device.__listener.on_limit_current(limit_current)
        else:
            fatal_error("__on_limit_current_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_force_level_internal(uuid_ptr, force_level):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_force_level is not None:
                device.__on_force_level(force_level)
            if device.__listener is not None and device.__listener.on_force_level is not None:
                device.__listener.on_force_level(force_level)
        else:
            fatal_error("__on_force_level_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, StarkFingerStatusData*)")
    def __on_finger_status_internal(uuid_ptr, c_data):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            data = StarkFingerStatusData(c_data)
            if device.__on_finger_status is not None:
                device.__on_finger_status(data)        
            if device.__listener is not None and device.__listener.on_finger_status is not None:
                device.__listener.on_finger_status(data)
        else:
            fatal_error("__on_finger_status_internal:device unavailable for:" + uuid)    

    @staticmethod
    @ffi.callback("void(char*, StarkFingerMovementData*)")
    def __on_finger_movement_status_internal(uuid_ptr, c_data):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            data = StarkFingerMovementData(c_data)
            if device.__on_finger_movement_status is not None:
                device.__on_finger_movement_status(data)        
            if device.__listener is not None and device.__listener.on_finger_movement_status is not None:
                device.__listener.on_finger_movement_status(data)
        else:
            fatal_error("__on_finger_movement_status_internal:device unavailable for:" + uuid)        

    @staticmethod
    @ffi.callback("void(char*, LedInfo*)")
    def __on_led_info_internal(uuid_ptr, c_info):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            info = LedInfo(c_info)
            if device.__on_led_info is not None:
                device.__on_led_info(info)
            if device.__listener is not None and device.__listener.on_led_info is not None:
                device.__listener.on_led_info(info)
        else:
            fatal_error("__on_led_info_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, ButtonPressEvent*)")
    def __on_button_event_internal(uuid_ptr, c_event):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            event = ButtonPressEvent(c_event)
            if device.__on_button_event is not None:
                device.__on_button_event(event)
            if device.__listener is not None and device.__listener.on_button_event is not None:
                device.__listener.on_button_event(event)
        else:
            fatal_error("__on_button_event_internal:device unavailable for:" + uuid)          

