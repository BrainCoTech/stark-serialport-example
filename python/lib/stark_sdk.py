import abc
import logging
import os
import pathlib
import platform
import time
import numpy as np
from enum import IntEnum  # Enum declarations
from cffi import FFI
from lib.stark_logging import *

ffi = FFI()


def fatal_error(msg):
    SKLog.error(f"FATAL_ERROR: {msg}")


def load_library():
    current_dir = pathlib.Path(__file__).resolve()
    parent_dir = current_dir.parent.parent.parent
    lib_dir = os.path.join(parent_dir, "dist")
    SKLog.info(f"Loading StarkSDK from {lib_dir}")

    # 1. load header
    with open(
        os.path.join(lib_dir, "include", "stark_sdk.h"), encoding="utf-8"
    ) as sdk_header:
        sdk_header = (
            sdk_header.read()
            .split("CFFI_DEF_START")[1]
            .split("CFFI_DEF_END")[0]
            .replace("SDK_EXTERN ", "")
            .replace("#if defined(_WIN32) || TARGET_OS_OSX", "")
            .replace("#endif", "")
        )
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
        SKLog.info(f"""Loading StarkSDK from {dll_path}""")
        return ffi.dlopen(dll_path)
    else:
        raise Exception(
            "Unsupported platform: " + platform.system() + ", arch: " + arch
        )


# load StarkSDK library
libstark = load_library()
print(f"StarkSDK loaded from {libstark}")


class LogLevel(IntEnum):
    debug = 0
    info = 1
    warning = 2
    error = 3
    none = 4


class BaudRate(IntEnum):
    baudrate_19200 = 19200
    baudrate_57600 = 57600
    baudrate_115200 = 115200
    baudrate_460800 = 460800


class FingerId(IntEnum):
    thumb = 1
    thumb_aux = 2
    index = 3
    middle = 4
    ring = 5
    pinky = 6


class HandType(IntEnum):
    unknown = 0
    medium_right = 1
    medium_left = 2
    small_right = 3
    small_left = 4


class ForceLevel(IntEnum):
    unknown = 0
    small = 1
    normal = 2
    full = 3


class MotorState(IntEnum):
    unknown = 0
    idle = 1
    running = 2
    protected_zone = 3
    stall = 4


class LedMode(IntEnum):
    unknown = 0
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
    unknown = 0
    pressing = 1
    not_pressing = 2


class ActionSequenceId(IntEnum):
    # 内置手势1~6：张开、握拳、两只捏、三只捏、侧边捏、单指点
    default_gesture_open = 1
    default_gesture_fist = 2
    default_gesture_pinch_two = 3
    default_gesture_pinch_three = 4
    default_gesture_pinch_side = 5
    default_gesture_point = 6
    # 自定义手势6个: 10~15
    custom_gesture_1 = 10
    custom_gesture_2 = 11
    custom_gesture_3 = 12
    custom_gesture_4 = 13
    custom_gesture_5 = 14
    custom_gesture_6 = 15


class StarkErrorCode(IntEnum):
    none = 0
    unknown = -1
    invalid_params = -2
    invalid_data = -3
    parse_failed = -4
    alloc_failed = -5
    read_failed = -6
    operation_failed = -7
    not_supported = -8


class StarkDfuState(IntEnum):
    idle = 0
    enabling = 1
    started = 2
    transfer = 3
    completed = 4
    aborted = 5


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
    isModbus = False
    __on_error = None
    on_read_data = None

    @staticmethod
    def init(
        isModbus: bool = False,
        log_level=logging.INFO,
        log_file_name: str = None,
        c_log_level: LogLevel = LogLevel.warning,
    ):
        StarkSDK.isModbus = isModbus
        StarkSDK.set_error_callback(
            lambda error: SKLog.error(f"Error: {error.message}")
        )
        if log_level == logging.DEBUG and c_log_level == LogLevel.warning:
            c_log_level = LogLevel.info
        StarkSDK.apply_logging_config(
            log_level, log_file_name=log_file_name, c_log_level=c_log_level
        )
        if libstark is None:
            fatal_error("Failed to load StarkSDK")
            return
        SKLog.info(f"StarkSDK Version: v{StarkSDK.get_sdk_version()}")
        if isModbus:
            from pymodbus.logging import pymodbus_apply_logging_config
            pymodbus_apply_logging_config(log_level)

    @staticmethod
    def apply_logging_config(
        log_level=logging.INFO, log_file_name: str = None, c_log_level: LogLevel = None
    ):
        if c_log_level is not None:
            StarkSDK.set_log_level(c_log_level)
            StarkSDK.set_log_callback(StarkSDK.log_message)

        SKLog.apply_logging_config(log_level, log_file_name=log_file_name)

    @staticmethod
    def log_message(level, message):
        if level == LogLevel.debug:
            SKLog.debug(message)
        elif level == LogLevel.info:
            SKLog.info(message)
        elif level == LogLevel.warning:
            SKLog.warning(message)
        elif level == LogLevel.error:
            SKLog.error(message)

    @staticmethod
    def set_log_level(level=LogLevel.info):
        return libstark.stark_set_log_level(level.value)

    @staticmethod
    def set_log_callback(cb):
        StarkSDK.__on_log_message = cb
        libstark.stark_set_log_callback(StarkSDK.__on_log_message_internal)

    @staticmethod
    @ffi.callback("void(int, char*)")
    def __on_log_message_internal(level, msg):
        if StarkSDK.__on_log_message is not None:
            try:
                message = ffi.string(msg).decode("utf-8")
                StarkSDK.__on_log_message(LogLevel(level), message)
            except Exception as e:
                SKLog.warning(f"Failed to decode log message: {e}")

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
    def dfu_state_to_string(dfu_state: int):
        return ffi.string(libstark.stark_dfu_state_to_string(dfu_state)).decode("utf-8")

    @staticmethod
    def set_write_data_callback(cb):
        StarkSDK.__on_write_data = cb
        libstark.stark_set_write_data_callback(StarkSDK.__on_write_data_internal)

    @staticmethod
    def set_read_data_callback(cb):
        StarkSDK.on_read_data = cb

    @staticmethod
    @ffi.callback("int(char*, uint8_t*, int)")
    def __on_write_data_internal(_, data, length):
        if StarkSDK.__on_write_data is not None:
            # 使用 ffi.buffer 创建一个引用C内存区域的缓冲区对象并转换为新的Python字节数组
            value = ffi.buffer(data, length)[:]  # bytes(data)
            StarkSDK.__on_write_data(list(value))
            return 0
        return -1

    @staticmethod
    def set_finger_positions(finger_positions):
        if StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Modbus device not support set_finger_positions",
                )
            )
            return
        if not isinstance(finger_positions, list) or len(finger_positions) != 6:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid finger_positions:" + str(finger_positions),
                )
            )
            return
        # check if all values are in range
        for pos in finger_positions:
            if pos < 0 or pos > 100:
                StarkSDK.run_error_callback(
                    StarkError(
                        StarkErrorCode.invalid_params,
                        "Invalid finger_positions:" + str(finger_positions),
                    )
                )
                return
        libstark.stark_group_set_finger_positions(ffi.new("int[]", finger_positions))

    @staticmethod
    def set_finger_speeds(finger_speeds):
        if StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Modbus device not support set_finger_speeds",
                )
            )
            return
        if not isinstance(finger_speeds, list) or len(finger_speeds) != 6:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid finger_speeds:" + str(finger_speeds),
                )
            )
            return
        # check if all values are in range
        for speed in finger_speeds:
            if speed < -100 or speed > 100:
                StarkSDK.run_error_callback(
                    StarkError(
                        StarkErrorCode.invalid_params,
                        "Invalid finger_speeds:" + str(finger_speeds),
                    )
                )
                return
        libstark.stark_group_set_finger_speeds(ffi.new("int[]", finger_speeds))


class SerialPortCfg:
    def __init__(self, c_cfg):
        self.serial_device_id = c_cfg.serial_device_id
        self.baudrate = c_cfg.baudrate

    def __str__(self):
        return f"SerialPortCfg(serial_device_id={self.serial_device_id}, baudrate={self.baudrate})"


class MotorboardInfo:
    def __init__(self, c_info):
        self.hand_type = HandType.unknown
        if (
            isinstance(c_info.hand_type, int)
            and c_info.hand_type >= 0
            and c_info.hand_type < 5
        ):
            self.hand_type = HandType(c_info.hand_type)
        self.sn = ffi.string(c_info.sn, 20).decode("utf-8")
        self.fw_version = ffi.string(c_info.fw_version, 20).decode("utf-8")

    def __str__(self):
        return (
            f"MotorboardInfo(hand_type={self.hand_type.name}, "
            f"sn={self.sn}, fw_version={self.fw_version})"
        )


class TouchStatus:
    def __init__(self, c_status):
        self.normal_force1 = c_status.normal_force1
        self.normal_force2 = c_status.normal_force2
        self.normal_force3 = c_status.normal_force3
        self.tangential_force1 = c_status.tangential_force1
        self.tangential_force2 = c_status.tangential_force2
        self.tangential_force3 = c_status.tangential_force3
        self.tangential_direction1 = c_status.tangential_direction1
        self.tangential_direction2 = c_status.tangential_direction2
        self.tangential_direction3 = c_status.tangential_direction3
        self.self_close1 = c_status.self_close1
        self.self_close2 = c_status.self_close2
        self.mutual_close = c_status.mutual_close
        self.status = c_status.status

    def __str__(self):
        return (
            f"\tnormal_force1: {self.normal_force1}, normal_force2: {self.normal_force2}, normal_force3: {self.normal_force3},\n"
            f"\ttangential_force1: {self.tangential_force1}, tangential_force2: {self.tangential_force2},\n"
            f"\ttangential_force3: {self.tangential_force3}, tangential_direction1: {self.tangential_direction1},\n"
            f"\ttangential_direction2: {self.tangential_direction2}, tangential_direction3: {self.tangential_direction3},\n"
            f"\tself_close1: {self.self_close1}, self_close2: {self.self_close2},\n"
            f"\tmutual_close: {self.mutual_close}, status: {self.status}"
        )


class TouchStatusData:
    def __init__(self, c_data):
        self.n_touch_status = c_data.n_touch_status
        self.touch_status = []
        for i in range(self.n_touch_status):
            self.touch_status.append(TouchStatus(c_data.touch_status[i]))

    def __str__(self):
        return (
            f"n_touch_status: {self.n_touch_status},\n\n"
            f"touch_status[0]: \n{self.touch_status[0]},\n\n"
            f"touch_status[1]: \n{self.touch_status[1]},\n\n"
            f"touch_status[2]: \n{self.touch_status[2]},\n\n"
            f"touch_status[3]: \n{self.touch_status[3]},\n\n"
            f"touch_status[4]: \n{self.touch_status[4]}"
        )

class TouchRawData:
    def __init__(self, c_data):
        # 通道数, 每个通道的数据类型为uint32_t, 有效值为24 bits
        # 7 + 11 + 11 + 11 + 7
        self.thumb = ffi.unpack(c_data.thumb, 7)
        self.index = ffi.unpack(c_data.index, 11)
        self.middle = ffi.unpack(c_data.middle, 11)
        self.ring = ffi.unpack(c_data.ring, 11)
        self.pinky = ffi.unpack(c_data.pinky, 7)
        
    def __str__(self):
        return (
            f"TouchRawData:\n"
            f"\t thumb: {self.thumb}, \n"
            f"\t index: {self.index}, \n"
            f"\t middle: {self.middle}, \n"
            f"\t ring: {self.ring}, \n"
            f"\t pinky: {self.pinky}"
        )    


class StarkFingerStatus:
    def __init__(self, c_status):
        self.finger_positions = ffi.unpack(c_status.finger_positions, 6)
        self.finger_speeds = ffi.unpack(c_status.finger_speeds, 6)
        self.finger_currents = ffi.unpack(c_status.finger_currents, 6)

        # protobuf firmware only
        self.finger_pwms = ffi.unpack(c_status.finger_pwms, 6)

        # Modbus firmware only
        self.finger_states = []
        values = ffi.unpack(c_status.finger_states, 6)
        for value in values:
            self.finger_states.append(motorStateFromValue(value))

    def __str__(self):
        if StarkSDK.isModbus:
            return (
                f"\tpositions: {self.finger_positions},\n"
                f"\tspeeds: {self.finger_speeds},\n"
                f"\tcurrents: {self.finger_currents},\n"
                f"\tstates: {[state.name for state in self.finger_states]}"
            )
        return (
            f"\tpositions: {self.finger_positions},\n"
            f"\tspeeds: {self.finger_speeds},\n"
            f"\tcurrents: {self.finger_currents},\n"
            f"\tpwms: {self.finger_pwms}"
        )

    @property
    def is_idle(self):
        # fmt: off
        if StarkSDK.isModbus and self.finger_currents == [0] * 6 and self.finger_states == [MotorState.idle] * 6:
            return True
        if not StarkSDK.isModbus and self.finger_currents == [0] * 6 and self.finger_pwms == [0] * 6:
            return True
        return False

    @property
    def in_open_position(self):
        required_positions = [0] * 6
        # fmt: off
        for finger_position, required_position in zip(self.finger_positions, required_positions):
            if finger_position > required_position: return False
        return True

    @property
    def in_close_position(self):
        # fmt: off
        required_positions = [55, 55, 99, 99, 99, 85] # 考虑到惯性及结构误差，这里的值可以根据实际情况调整

        for finger_position, required_position in zip(self.finger_positions, required_positions):
            if finger_position < required_position: return False
        return True


class FingerStatusData:
    def __init__(self, c_data):
        self.n_finger_status = c_data.n_finger_status
        self.finger_status = []
        for i in range(self.n_finger_status):
            self.finger_status.append(StarkFingerStatus(c_data.finger_status[i]))

    def __str__(self):
        if self.n_finger_status > 0:
            return f"n_finger_status: {self.n_finger_status}, finger_status:\n{self.finger_status[self.n_finger_status - 1]}"
        return f"n_finger_status: {self.n_finger_status}, finger_status is empty"

    @property
    def last_finger_status(self):
        if self.n_finger_status > 0:
            return self.finger_status[self.n_finger_status - 1]
        return None

    @property
    def is_idle(self):
        status = self.last_finger_status
        return status is not None and status.is_idle

    @property
    def is_opened(self):
        status = self.last_finger_status
        return status is not None and status.is_idle and status.in_open_position

    @property
    def is_closed(self):
        status = self.last_finger_status
        return status is not None and status.is_idle and status.in_close_position


class ActionSequenceData:
    def __init__(self, c_data):
        self.index = c_data.index
        self.duration = c_data.duration
        self.positions = ffi.unpack(c_data.motor_positions, 6)
        self.speeds = ffi.unpack(c_data.motor_speeds, 6)
        self.strengths = ffi.unpack(c_data.motor_strengths, 6)


class ActionSequence:
    def __init__(self, c_seq):
        self.action_id = c_seq.action_id
        self.action_num = c_seq.action_num
        self.action_sequences = []
        for i in range(self.action_num):
            self.action_sequences.append(ActionSequenceData(c_seq.data[i]))

    def __str__(self):
        return f"action_id: {self.action_id}, action_num: {self.action_num}"


def motorStateFromValue(value):
    if value not in MotorState._value2member_map_:
        state = MotorState.idle
    else:
        if StarkSDK.isModbus:
            state = MotorState.idle
            if value == 1:
                state = MotorState.running
            elif value == 2:
                state = MotorState.stall
            # elif value != 0:
            #     SKLog.warning("Invalid motor state value: " + str(value))
        else:
            state = MotorState(value)

    return state


class MotorStatusData:
    def __init__(self, c_data):
        self.n_motor_status = c_data.n_motor_status
        values = ffi.unpack(c_data.motor_status, self.n_motor_status)
        self.motor_statuses = []
        for value in values:
            self.motor_statuses.append(motorStateFromValue(value))

    def __str__(self) -> str:
        return f"motor_statuses: {', '.join([status.name for status in self.motor_statuses])}"


class LedInfo:
    def __init__(self, c_info):
        self.led_color = LedColor(c_info.led_color)
        self.led_mode = LedMode(c_info.led_mode)

    __str__ = (
        lambda self: f"LedInfo(led_color={self.led_color.name}, led_mode={self.led_mode.name})"
    )


class ButtonPressEvent:
    def __init__(self, c_info):
        self.timestamp = c_info.timestamp
        self.button_id = c_info.button_id
        self.press_status = ButtonPressState(c_info.press_status)

    def __str__(self):
        return f"timestamp: {self.timestamp}, button_id: {self.button_id}, press_status: {self.press_status.name}"


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

    def on_motor_status(self, motor_status):
        pass

    def on_button_event(self, button_event):
        pass

    def on_led_info(self, led_info):
        pass

    def on_turbo_mode(self, turbo_mode):
        pass

    def on_turbo_conf(self, turbo_interval, turbo_duration):
        pass

    def on_auto_calibration(self, auto_calibration_enabled):
        pass
    
    def on_touch_sensor_enabled(self, finger_bits):
        pass

    def on_touch_sensor_raw_data(self, touch_raw_data):
        pass
    
    def on_touch_sensor_status(self, touch_status):
        pass

    def on_touch_sensor_firmware(self, versions):
        pass

    def on_finger_positions(self, finger_positions):
        pass

    def on_finger_speeds(self, finger_speeds):
        pass

    def on_finger_currents(self, finger_currents):
        pass

    def on_action_sequence(self, action_sequence):
        pass

    def on_stall_durations(self, stall_durations):
        pass

    def on_stall_currents(self, stall_currents):
        pass

    def on_finger_pwms(self, finger_pwms):
        pass

    def on_dfu_state(self, dfu_state):
        pass

    def on_dfu_progress(self, dfu_progress):
        pass


data_array_store = []


class StarkDevice(StarkDeviceListener):
    @classmethod
    def create_device(cls, id: int, name: str):
        uuid = str(id)
        if uuid in cls._device_map:
            device = cls._device_map.get(uuid)
            return device
        device = StarkDevice(uuid, name, address=id)
        cls._device_map[uuid] = device

        uuid_str = uuid.encode("utf-8")
        device_ptr = (
            libstark.stark_create_modbus_device(uuid_str, id)
            if StarkSDK.isModbus
            else libstark.stark_create_serial_device(uuid_str, id)
        )
        if device_ptr is not ffi.NULL:
            StarkDevice._device_pointer_map[uuid] = device_ptr
            libstark.stark_set_error_callback(
                device_ptr, StarkDevice.__on_error_internal
            )
        return device

    _device_pointer_map = {}
    _device_map = {}
    _config_response_callbacks = {}

    __address = 0
    __uuid = None
    __name = None
    __listener = None

    def __init__(self, uuid: str, name: str, address: int):
        self.__uuid = uuid
        self.__name = name
        self.__address = address

    @property
    def uuid(self):
        return self.__uuid

    @property
    def address(self):
        return self.__address

    @property
    def slave(self):
        return self.__address

    @property
    def slave_id(self):
        return self.__address

    @property
    def name(self):
        return self.__name

    def hello(self):
        if self.__uuid in StarkDevice._device_pointer_map:
            device_ptr = StarkDevice._device_pointer_map[self.__uuid]
            libstark.hello(device_ptr)

    def run_error_callback(self, error):
        StarkSDK.run_error_callback(error)
        if self.__listener is not None and self.__listener.on_error is not None:
            self.__listener.on_error(error)

    def set_listener(self, listener):
        if isinstance(listener, StarkDeviceListener):
            self.__listener = listener

    def serial_receive_data(self):
        # I want to sleep smaller time to get data faster
        # time.sleep(0.02) # 会导致个别读取到的响应包不完整, 如get_force_level
        time.sleep(0.05)
        if StarkSDK.on_read_data is not None:
            data = StarkSDK.on_read_data()
            if data is not None and len(data) > 0:
                self.did_receive_data(data)

    def did_receive_data(self, data):
        if data is None or len(data) == 0:
            SKLog.warning(f"Invalid data received")
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            device_ptr = StarkDevice._device_pointer_map[self.__uuid]
            data_view = memoryview(data)
            data_array = ffi.cast("uint8_t*", ffi.from_buffer(data_view))
            libstark.stark_did_receive_data(device_ptr, data_array, len(data_view))
        else:
            fatal_error(f"UUID {self.__uuid} not found in device pointer map")

    def set_write_registers_callback(self, cb):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support set_write_registers_callback",
                )
            )
            return
        self.__on_write_registers = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.modbus_set_write_registers_callback(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_write_registers_internal,
            )

    def set_read_holding_registers_callback(self, cb):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support set_read_holding_registers_callback",
                )
            )
            return
        self.__on_read_holding_registers = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.modbus_set_read_holding_register_callback(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_read_holding_register_internal,
            )
            libstark.modbus_set_read_holding_registers_callback(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_read_holding_registers_internal,
            )

    def set_read_input_registers_callback(self, cb):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support set_read_input_registers_callback",
                )
            )
            return
        self.__on_read_input_registers = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.modbus_set_read_input_register_callback(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_read_input_register_internal,
            )
            libstark.modbus_set_read_input_registers_callback(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_read_input_registers_internal,
            )

    def set_dfu_write_callback(self, cb):
        StarkSDK.set_write_data_callback(cb)

    def set_dfu_read_callback(self, cb):
        self.__on_dfu_read = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_dfu_read_callback(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_dfu_read_internal,
            )

    def set_dfu_state_callback(self, cb):
        self.__on_dfu_state = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_dfu_state_callback(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_dfu_state_internal,
            )

    def set_dfu_progress_callback(self, cb):
        self.__on_dfu_progress = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_dfu_progress_callback(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_dfu_progress_internal,
            )

    def set_dfu_cfg(
        self, dfu_enabling_delay=8, dfu_enabling_interval=10, dfu_applying_delay=10
    ):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_dfu_cfg(
                StarkDevice._device_pointer_map[self.__uuid],
                dfu_enabling_delay,
                dfu_enabling_interval,
                dfu_applying_delay,
            )

    def start_dfu(self, dfu_file_path: str):
        SKLog.info(f"start_dfu: {dfu_file_path}")
        if not os.path.exists(dfu_file_path):
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid dfu_file_path:" + dfu_file_path,
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_start_dfu(
                StarkDevice._device_pointer_map[self.__uuid],
                dfu_file_path.encode("utf-8"),
            )

    def abort_dfu(self):
        SKLog.info(f"abort_dfu")
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_abort_dfu(StarkDevice._device_pointer_map[self.__uuid])

    def get_serialport_cfg(self, cb=None):
        self.__on_serialport_cfg = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_serialport_cfg(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_serialport_cfg_internal,
            )
        if not StarkSDK.isModbus:
            self.serial_receive_data()

    def set_serial_baudrate(self, baudrate: BaudRate):
        if isinstance(baudrate, BaudRate) is False:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params, "Invalid baudrate:" + str(baudrate)
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_serial_baudrate(
                StarkDevice._device_pointer_map[self.__uuid], baudrate.value
            )

    def set_serial_device_id(self, device_id):
        if (not StarkSDK.isModbus and device_id < 10 or device_id > 253) or (
            StarkSDK.isModbus and device_id < 0 or device_id > 254
        ):
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params, "Invalid device_id:" + str(device_id)
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_serial_device_id(
                StarkDevice._device_pointer_map[self.__uuid], device_id
            )

    def factory_set_device_sn(self, operation_key, board_sn):
        if not isinstance(operation_key, str) or operation_key == "":
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid operation_key:" + operation_key,
                )
            )
            return
        if not isinstance(board_sn, str) or board_sn == "":
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params, "Invalid board_sn:" + board_sn
                )
            )
            return
        if len(board_sn) != 17:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid board_sn length:" + str(len(board_sn)),
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.factory_set_device_sn(
                StarkDevice._device_pointer_map[self.__uuid],
                operation_key.encode("utf-8"),
                board_sn.encode("utf-8"),
            )

    def factory_set_hand_type(self, operation_key, hand_type=HandType.medium_left):
        if not isinstance(operation_key, str) or operation_key == "":
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid operation_key:" + operation_key,
                )
            )
            return
        if not isinstance(hand_type, HandType):
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params, "Invalid hand_type:" + str(hand_type)
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.factory_set_hand_type(
                StarkDevice._device_pointer_map[self.__uuid],
                operation_key.encode("utf-8"),
                hand_type.value,
            )

    def factory_get_stall_durations(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support factory_get_stall_durations",
                )
            )
            return
        self.__on_stall_durations = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.factory_get_stall_durations(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_stall_durations_internal,
            )

    def factory_set_stall_durations(self, operation_key, durations):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support factory_set_stall_durations",
                )
            )
            return
        if not isinstance(operation_key, str) or operation_key == "":
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid operation_key:" + operation_key,
                )
            )
            return
        if durations is None or len(durations) != 6:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid stall_durations:" + str(durations),
                )
            )
            return
        # check if all values are in range
        for duration in durations:
            if duration < 100 or duration > 10000:
                self.run_error_callback(
                    StarkError(
                        StarkErrorCode.invalid_params,
                        "Invalid stall_durations:" + str(durations),
                    )
                )
                return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.factory_set_stall_durations(
                StarkDevice._device_pointer_map[self.__uuid],
                operation_key.encode("utf-8"),
                durations,
            )

    def factory_set_stall_currents(self, operation_key, level: ForceLevel, currents):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support factory_set_stall_currents",
                )
            )
            return
        if not isinstance(operation_key, str) or operation_key == "":
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid operation_key:" + operation_key,
                )
            )
            return
        if not isinstance(currents, list) or len(currents) != 6:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid finger_currents:" + str(currents),
                )
            )
            return
        # check if all values are in range
        for current in currents:
            if current < 50 or current > 600:
                self.run_error_callback(
                    StarkError(
                        StarkErrorCode.invalid_params,
                        "Invalid finger_currents:" + str(currents),
                    )
                )
                return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.factory_set_stall_currents(
                StarkDevice._device_pointer_map[self.__uuid],
                operation_key.encode("utf-8"),
                level.value,
                ffi.new("int[]", currents),
            )

    def factory_set_finger_pwms(self, operation_key, level: ForceLevel, pwms):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support factory_set_finger_pwms",
                )
            )
            return
        if not isinstance(pwms, list) or len(pwms) != 6:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params, "Invalid finger_pwms:" + str(pwms)
                )
            )
            return
        if not isinstance(operation_key, str) or operation_key == "":
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid operation_key:" + operation_key,
                )
            )
            return
        # check if all values are in range
        for pwm in pwms:
            if pwm < 0 or pwm > 100:
                self.run_error_callback(
                    StarkError(
                        StarkErrorCode.invalid_params,
                        "Invalid finger_pwms:" + str(pwms),
                    )
                )
                return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.factory_set_finger_pwms(
                StarkDevice._device_pointer_map[self.__uuid],
                operation_key.encode("utf-8"),
                level.value,
                ffi.new("int[]", pwms),
            )

    def factory_get_stall_currents(self, level: ForceLevel, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support factory_get_stall_currents",
                )
            )
            return
        self.__on_stall_currents = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.factory_get_stall_currents(
                StarkDevice._device_pointer_map[self.__uuid],
                level.value,
                StarkDevice.__on_stall_currents_internal,
            )

    def factory_get_finger_pwms(self, level: ForceLevel, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support factory_get_finger_pwms",
                )
            )
            return
        self.__on_finger_pwms = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.factory_get_finger_pwms(
                StarkDevice._device_pointer_map[self.__uuid],
                level.value,
                StarkDevice.__on_finger_pwms_internal,
            )

    def get_hand_type(self, cb=None):
        self.__on_hand_type = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_hand_type(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_hand_type_internal,
            )
        if not StarkSDK.isModbus:
            self.serial_receive_data()

    def get_motorboard_info(self, cb=None):
        self.__on_motorboard_info = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_motorboard_info(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_motorboard_info_internal,
            )
        if not StarkSDK.isModbus:
            self.serial_receive_data()

    def get_led_info(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_led_info",
                )
            )
            return
        self.__on_led_info = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_led_info(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_led_info_internal,
            )
        if not StarkSDK.isModbus:
            self.serial_receive_data()

    def get_voltage(self, cb=None):
        self.__on_voltage = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_voltage(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_voltage_internal,
            )
        if not StarkSDK.isModbus:
            self.serial_receive_data()

    def get_max_current(self, cb=None):
        if StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Modbus device not support get_max_current",
                )
            )
            return
        self.__on_limit_current = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_max_current(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_limit_current_internal,
            )
        if not StarkSDK.isModbus:
            self.serial_receive_data()

    def set_max_current(self, max_current):
        if StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Modbus device not support set_max_current",
                )
            )
            return
        # max value is 2000mA
        if max_current < 0 or max_current > 2000000:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid max_current:" + str(max_current),
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_max_current(
                StarkDevice._device_pointer_map[self.__uuid], max_current
            )

    def get_force_level(self, cb=None):
        self.__on_force_level = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_force_level(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_force_level_internal,
            )
        if not StarkSDK.isModbus:
            self.serial_receive_data()

    def set_force_level(self, force_level=ForceLevel.normal):
        if not isinstance(force_level, ForceLevel):
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid force_level:" + str(force_level),
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_force_level(
                StarkDevice._device_pointer_map[self.__uuid], force_level.value
            )

    def get_turbo_mode(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_turbo_mode",
                )
            )
            return
        self.__on_turbo_mode = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_turbo_mode(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_turbo_mode_internal,
            )

    def set_turbo_mode(self, turbo_mode: bool):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support set_turbo_mode",
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_turbo_mode(
                StarkDevice._device_pointer_map[self.__uuid], turbo_mode
            )

    # 单位：ms
    def set_turbo_conf(self, turbo_interval: int = 500, turbo_duration: int = 500):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support set_turbo_config",
                )
            )
            return
        if turbo_interval <= 50 or turbo_duration <= 50:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid turbo_interval or turbo_duration",
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_turbo_conf(
                StarkDevice._device_pointer_map[self.__uuid],
                turbo_interval,
                turbo_duration,
            )

    def get_turbo_conf(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_turbo_config",
                )
            )
            return
        self.__on_turbo_conf = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_turbo_conf(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_turbo_conf_internal,
            )

    def set_auto_calibration(self, enabled: bool):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support set_auto_calibration",
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_auto_calibration(
                StarkDevice._device_pointer_map[self.__uuid], enabled
            )

    def get_auto_calibration(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_auto_calibration",
                )
            )
            return
        self.__on_auto_calibration = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_auto_calibration(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_auto_calibration_internal,
            )

    def send_manual_calibration_cmd(self):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support send_manual_calibration",
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_manual_calibration(
                StarkDevice._device_pointer_map[self.__uuid]
            )

    def get_touch_sensor_firmware(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_touch_sensor_firmware",
                )
            )
            return
        self.__on_touch_sensor_firmware = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_touch_sensor_firmware(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_touch_sensor_firmware_internal,
            )

    def get_touch_sensor_enabled(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_touch_sensor_enabled",
                )
            )
            return
        self.__on_touch_sensor_enabled = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_touch_sensor_enabled(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_touch_sensor_enabled_internal,
            )

    def set_touch_sensor_enabled(self, finger_bits: int = 0b11111):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support set_touch_sensor_enabled",
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_touch_sensor_enabled(
                StarkDevice._device_pointer_map[self.__uuid], finger_bits
            )
        
    # 0b 11111 all fingers
    def touch_sensor_reset(self, finger_bits: int = 0b11111):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support touch_sensor_reset",
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_touch_sensor_reset(
                StarkDevice._device_pointer_map[self.__uuid],
                finger_bits,
            )

    # 0x1F: 11111 all fingers
    def touch_sensor_calibrate(self, finger_bits: int = 0b11111):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support touch_sensor_calibrate",
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_touch_sensor_calibrate(
                StarkDevice._device_pointer_map[self.__uuid],
                finger_bits,
            )

    def get_touch_sensor_status(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_touch_sensor_status",
                )
            )
            return
        self.__on_touch_sensor_status = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_touch_sensor_status(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_touch_sensor_status_internal,
            )
            
    def get_touch_sensor_raw_data(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_touch_sensor_raw_data",
                )
            )
            return
        self.__on_touch_sensor_raw_data = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_touch_sensor_raw_data(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_touch_sensor_raw_data_internal,
            )

    def get_finger_status(self, cb=None):
        self.__on_finger_status = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_finger_status(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_finger_status_internal,
            )
        if not StarkSDK.isModbus:
            self.serial_receive_data()

    def get_motor_status(self, cb=None):
        self.__on_motor_status = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_motor_status(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_motor_status_internal,
            )
        if not StarkSDK.isModbus:
            self.serial_receive_data()

    # 实际位置值为 0-100，0 表示张开手指，100 表示闭合手指
    def get_finger_positions(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_finger_positions",
                )
            )
            return
        self.__on_finger_positions = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_finger_positions(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_finger_positions_internal,
            )

    # 实际速度值为 -100-100，-100 表示最大张开速度，100 表示最大闭合速度
    def get_finger_speeds(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_finger_speeds",
                )
            )
            return
        self.__on_finger_speeds = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_finger_speeds(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_finger_speeds_internal,
            )

    # 实际电流值为 -100-100，-100 表示最小电流，100 表示最大电流
    def get_finger_currents(self, cb=None):
        if not StarkSDK.isModbus:
            StarkSDK.run_error_callback(
                StarkError(
                    StarkErrorCode.not_supported,
                    "Protobuf device not support get_finger_currents",
                )
            )
            return
        self.__on_finger_currents = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_finger_currents(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_finger_currents_internal,
            )

    def reset_finger_positions(self):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_reset_finger_positions(
                StarkDevice._device_pointer_map[self.__uuid]
            )

    def set_finger_position(self, finger_position):
        # 0 表示张开手指，100 表示闭合手指
        if finger_position < 0 or finger_position > 100:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid finger_position:" + str(finger_position),
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_position(
                StarkDevice._device_pointer_map[self.__uuid], finger_position
            )

    def set_finger_positions(self, finger_positions):
        if not isinstance(finger_positions, list) or len(finger_positions) != 6:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid finger_positions:" + str(finger_positions),
                )
            )
            return
        # check if all values are in range
        for pos in finger_positions:
            if pos < 0 or pos > 100:
                self.run_error_callback(
                    StarkError(
                        StarkErrorCode.invalid_params,
                        "Invalid finger_positions:" + str(finger_positions),
                    )
                )
                return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_positions(
                StarkDevice._device_pointer_map[self.__uuid],
                ffi.new("int[]", finger_positions),
            )

    def set_finger_speed(self, finger_speed):
        if finger_speed < -100 or finger_speed > 100:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid finger_speed:" + str(finger_speed),
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_speed(
                StarkDevice._device_pointer_map[self.__uuid], finger_speed
            )

    def set_finger_speeds(self, finger_speeds):
        if not isinstance(finger_speeds, list) or len(finger_speeds) != 6:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid finger_speeds:" + str(finger_speeds),
                )
            )
            return
        # check if all values are in range
        for speed in finger_speeds:
            if speed < -100 or speed > 100:
                self.run_error_callback(
                    StarkError(
                        StarkErrorCode.invalid_params,
                        "Invalid finger_speeds:" + str(finger_speeds),
                    )
                )
                return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_speeds(
                StarkDevice._device_pointer_map[self.__uuid],
                ffi.new("int[]", finger_speeds),
            )

    def set_led_info(self, mode=LedMode.blink, color=LedColor.unchanged):
        if not isinstance(mode, LedMode) or not isinstance(color, LedColor):
            self.run_error_callback(
                StarkError(StarkErrorCode.invalid_params, "Invalid mode or color")
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_led_info(
                StarkDevice._device_pointer_map[self.__uuid], mode.value, color.value
            )

    def get_button_event(self, cb=None):
        self.__on_button_event = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_button_event(
                StarkDevice._device_pointer_map[self.__uuid],
                StarkDevice.__on_button_event_internal,
            )
        if not StarkSDK.isModbus:
            self.serial_receive_data()

    def save_action_sequence(self, action_id: ActionSequenceId):
        if action_id.value < ActionSequenceId.custom_gesture_1.value:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid action_id:" + str(action_id.name),
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_save_action_sequence(
                StarkDevice._device_pointer_map[self.__uuid], action_id.value
            )

    def transfer_action_sequence(
        self, action_id: ActionSequenceId, action_sequences: list
    ):
        if action_id.value < ActionSequenceId.custom_gesture_1.value:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid action_id:" + str(action_id.name),
                )
            )
            return
        if (
            not isinstance(action_sequences, list)
            or len(action_sequences) < 0
            or len(action_sequences) > 32
        ):
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid action_sequences:" + str(action_sequences),
                )
            )
            return
        action_sequences = np.array(list(action_sequences), dtype=np.uint16)
        rows, cols = action_sequences.shape
        if cols != 20:
            self.run_error_callback(
                StarkError(
                    StarkErrorCode.invalid_params,
                    "Invalid action_sequences cols:" + str(cols),
                )
            )
            return
        if self.__uuid in StarkDevice._device_pointer_map:
            ptr_array = ffi.new("uint16_t*[]", rows)
            for i in range(rows):
                ptr_array[i] = ffi.cast(
                    "uint16_t*", ffi.from_buffer(action_sequences[i])
                )
            libstark.stark_transfer_action_sequence(
                StarkDevice._device_pointer_map[self.__uuid],
                action_id.value,
                rows,
                ffi.cast("uint16_t**", ptr_array),
            )

    def run_action_sequence(self, action_id: ActionSequenceId):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_run_action_sequence(
                StarkDevice._device_pointer_map[self.__uuid], action_id.value
            )

    def get_action_sequence(self, action_id: ActionSequenceId, cb=None):
        self.__on_action_sequence = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_action_sequence(
                StarkDevice._device_pointer_map[self.__uuid],
                action_id.value,
                StarkDevice.__on_action_sequence_internal,
            )

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
            if (
                device.__listener is not None
                and device.__listener.on_motorboard_info is not None
            ):
                device.__listener.on_motorboard_info(info)
        else:
            fatal_error("__on_motorboard_info_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_hand_type_internal(uuid_ptr, c_hand_type):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if c_hand_type == 0:
                SKLog.warning("Hand type is not set")
                return
            if c_hand_type > 0 and c_hand_type < 5:
                hand_type = HandType(c_hand_type)
            if device.__on_hand_type is not None:
                device.__on_hand_type(hand_type)
            if (
                device.__listener is not None
                and device.__listener.on_hand_type is not None
            ):
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
            if (
                device.__listener is not None
                and device.__listener.on_serialport_cfg is not None
            ):
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
            if (
                device.__listener is not None
                and device.__listener.on_voltage is not None
            ):
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
            if (
                device.__listener is not None
                and device.__listener.on_limit_current is not None
            ):
                device.__listener.on_limit_current(limit_current)
        else:
            fatal_error("__on_limit_current_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_force_level_internal(uuid_ptr, force_level):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            level = ForceLevel(force_level)
            if device.__on_force_level is not None:
                device.__on_force_level(level)
            if (
                device.__listener is not None
                and device.__listener.on_force_level is not None
            ):
                device.__listener.on_force_level(level)
        else:
            fatal_error("__on_force_level_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_turbo_mode_internal(uuid_ptr, turbo_mode):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            in_turbo_mode = turbo_mode == 1
            if device.__on_turbo_mode is not None:
                device.__on_turbo_mode(in_turbo_mode)
            if (
                device.__listener is not None
                and device.__listener.on_turbo_mode is not None
            ):
                device.__listener.on_turbo_mode(in_turbo_mode)
        else:
            fatal_error("__on_turbo_mode_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, int, int)")
    def __on_turbo_conf_internal(uuid_ptr, turbo_interval, turbo_duration):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_turbo_conf is not None:
                device.__on_turbo_conf(turbo_interval, turbo_duration)
            if (
                device.__listener is not None
                and device.__listener.on_turbo_conf is not None
            ):
                device.__listener.on_turbo_conf(turbo_interval, turbo_duration)
        else:
            fatal_error("__on_turbo_conf_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, bool)")
    def __on_auto_calibration_internal(uuid_ptr, auto_calibration):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            auto_calibration_enabled = auto_calibration == 1
            if device.__on_auto_calibration is not None:
                device.__on_auto_calibration(auto_calibration_enabled)
            if (
                device.__listener is not None
                and device.__listener.on_auto_calibration is not None
            ):
                device.__listener.on_auto_calibration(auto_calibration_enabled)
        else:
            fatal_error("__on_auto_calibration_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, const char*[])")
    def __on_touch_sensor_firmware_internal(uuid_ptr, versions):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            try:
                SKLog.debug("versions:" + str(versions))
                versions_array = [
                    ffi.string(versions[i], 20).decode("utf-8") for i in range(5)
                ]
                if device.__on_touch_sensor_firmware is not None:
                    device.__on_touch_sensor_firmware(versions_array)
                if (
                    device.__listener is not None
                    and device.__listener.on_touch_sensor_firmware is not None
                ):
                    device.__listener.on_touch_sensor_firmware(versions_array)
            except Exception as e:
                SKLog.warning(
                    f"__on_touch_sensor_firmware_internal, failed to parse versions: {e}"
                )
        else:
            fatal_error(
                "__on_touch_sensor_firmware_internal, device unavailable for:" + uuid
            )

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_touch_sensor_enabled_internal(uuid_ptr, finger_bits):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_touch_sensor_enabled is not None:
                device.__on_touch_sensor_enabled(finger_bits)
            if (
                device.__listener is not None
                and device.__listener.on_touch_sensor_enabled is not None
            ):
                device.__listener.on_touch_sensor_enabled(finger_bits)
        else:
            fatal_error("__on_touch_sensor_enabled_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, TouchRawData*)")
    def __on_touch_sensor_raw_data_internal(uuid_ptr, c_data):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            data = TouchRawData(c_data)
            if device.__on_touch_sensor_raw_data is not None:
                device.__on_touch_sensor_raw_data(data)
            if (
                device.__listener is not None
                and device.__listener.on_touch_sensor_raw_data is not None
            ):
                device.__listener.on_touch_sensor_raw_data(data)
        else:
            fatal_error("__on_touch_sensor_raw_data_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, TouchStatusData*)")
    def __on_touch_sensor_status_internal(uuid_ptr, c_data):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            data = TouchStatusData(c_data)
            if device.__on_touch_sensor_status is not None:
                device.__on_touch_sensor_status(data)
            if (
                device.__listener is not None
                and device.__listener.on_touch_sensor_status is not None
            ):
                device.__listener.on_touch_sensor_status(data)
        else:
            fatal_error("__on_finger_status_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, FingerStatusData*)")
    def __on_finger_status_internal(uuid_ptr, c_data):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            data = FingerStatusData(c_data)
            if device.__on_finger_status is not None:
                device.__on_finger_status(data)
            if (
                device.__listener is not None
                and device.__listener.on_finger_status is not None
            ):
                device.__listener.on_finger_status(data)
        else:
            fatal_error("__on_finger_status_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, ActionSequence*)")
    def __on_action_sequence_internal(uuid_ptr, c_action_sequence):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            action_sequence = ActionSequence(c_action_sequence)
            if device.__on_action_sequence is not None:
                device.__on_action_sequence(action_sequence)
            if (
                device.__listener is not None
                and device.__listener.on_action_sequence is not None
            ):
                device.__listener.on_action_sequence(action_sequence)
        else:
            fatal_error("__on_action_sequence_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, MotorStatusData*)")
    def __on_motor_status_internal(uuid_ptr, c_data):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            data = MotorStatusData(c_data)
            if device.__on_motor_status is not None:
                device.__on_motor_status(data)
            if (
                device.__listener is not None
                and device.__listener.on_motor_status is not None
            ):
                device.__listener.on_motor_status(data)
        else:
            fatal_error("__on_motor_status_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, uint16_t*)")
    def __on_finger_positions_internal(uuid_ptr, c_positions):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            positions = ffi.unpack(c_positions, 6)
            if device.__on_finger_positions is not None:
                device.__on_finger_positions(positions)
            if (
                device.__listener is not None
                and device.__listener.on_finger_positions is not None
            ):
                device.__listener.on_finger_positions(positions)
        else:
            fatal_error("__on_finger_positions_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, uint16_t*)")
    def __on_finger_speeds_internal(uuid_ptr, c_speeds):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            speeds = ffi.unpack(c_speeds, 6)
            if device.__on_finger_speeds is not None:
                device.__on_finger_speeds(speeds)
            if (
                device.__listener is not None
                and device.__listener.on_finger_speeds is not None
            ):
                device.__listener.on_finger_speeds(speeds)
        else:
            fatal_error("__on_finger_speeds_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, uint16_t*)")
    def __on_finger_currents_internal(uuid_ptr, c_currents):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            currents = ffi.unpack(c_currents, 6)
            if device.__on_finger_currents is not None:
                device.__on_finger_currents(currents)
            if (
                device.__listener is not None
                and device.__listener.on_finger_currents is not None
            ):
                device.__listener.on_finger_currents(currents)
        else:
            fatal_error("__on_finger_currents_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, uint16_t*)")
    def __on_stall_durations_internal(uuid_ptr, c_durations):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            durations = ffi.unpack(c_durations, 6)
            if device.__on_stall_durations is not None:
                device.__on_stall_durations(durations)
            if (
                device.__listener is not None
                and device.__listener.on_stall_durations is not None
            ):
                device.__listener.on_stall_durations(durations)
        else:
            fatal_error("__on_stall_durations_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, uint16_t*)")
    def __on_stall_currents_internal(uuid_ptr, c_currents):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            currents = ffi.unpack(c_currents, 6)
            if device.__on_stall_currents is not None:
                device.__on_stall_currents(currents)
            if (
                device.__listener is not None
                and device.__listener.on_stall_currents is not None
            ):
                device.__listener.on_stall_currents(currents)
        else:
            fatal_error("__on_stall_currents_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, uint16_t*)")
    def __on_finger_pwms_internal(uuid_ptr, c_pwms):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            pwms = ffi.unpack(c_pwms, 6)
            if device.__on_finger_pwms is not None:
                device.__on_finger_pwms(pwms)
            if (
                device.__listener is not None
                and device.__listener.on_finger_pwms is not None
            ):
                device.__listener.on_finger_pwms(pwms)
        else:
            fatal_error("__on_finger_pwms_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, LedInfo*)")
    def __on_led_info_internal(uuid_ptr, c_info):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            info = LedInfo(c_info)
            if device.__on_led_info is not None:
                device.__on_led_info(info)
            if (
                device.__listener is not None
                and device.__listener.on_led_info is not None
            ):
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
            if (
                device.__listener is not None
                and device.__listener.on_button_event is not None
            ):
                device.__listener.on_button_event(event)
        else:
            fatal_error("__on_button_event_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("int(char*, int, uint16_t*, int)")
    def __on_write_registers_internal(uuid_ptr, register_address, c_values, count):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_write_registers is not None:
                values = ffi.unpack(c_values, count)
                device.__on_write_registers(register_address, values)
                return 0
        else:
            fatal_error("__on_write_registers_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("uint16_t*(char*, int, int)")
    def __on_read_holding_registers_internal(uuid_ptr, register_address, count):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_read_holding_registers is not None:
                data = device.__on_read_holding_registers(register_address, count)
                if data is None or len(data) == 0:
                    SKLog.warning(f"Invalid data received")
                    return ffi.NULL
                data_array = ffi.new("uint16_t[]", data)
                data_array_store.clear()
                # keep reference to data_array to avoid garbage collection
                data_array_store.append(data_array)
                return data_array
        else:
            fatal_error(
                "__on_read_holding_registers_internal:device unavailable for:" + uuid
            )

    @staticmethod
    @ffi.callback("int(char*, int)")
    def __on_read_holding_register_internal(uuid_ptr, register_address):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_read_holding_registers is not None:
                data = device.__on_read_holding_registers(register_address, count=1)
                if data is None or len(data) == 0:
                    SKLog.warning(f"Invalid data received")
                    return -1
                return data[0]
        else:
            fatal_error(
                "__on_read_holding_register_internal:device unavailable for:" + uuid
            )

    @staticmethod
    @ffi.callback("uint16_t*(char*, int, int)")
    def __on_read_input_registers_internal(uuid_ptr, register_address, count):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_read_input_registers is not None:
                data = device.__on_read_input_registers(register_address, count)
                if data is None or len(data) == 0:
                    SKLog.warning(f"Invalid data received")
                    return ffi.NULL
                data_array = ffi.new("uint16_t[]", data)
                data_array_store.clear()
                # keep reference to data_array to avoid garbage collection
                data_array_store.append(data_array)
                return data_array
        else:
            fatal_error(
                "__on_read_input_registers_internal:device unavailable for:" + uuid
            )

    @staticmethod
    @ffi.callback("int(char*, int)")
    def __on_read_input_register_internal(uuid_ptr, register_address):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_read_input_registers is not None:
                data = device.__on_read_input_registers(register_address, count=1)
                if data is None or len(data) == 0:
                    SKLog.warning(f"Invalid data received")
                    return -1
                return data[0]
        else:
            fatal_error(
                "__on_read_input_register_internal:device unavailable for:" + uuid
            )

    @staticmethod
    @ffi.callback("void(char*)")
    def __on_dfu_read_internal(uuid_ptr):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_dfu_read is not None:
                device.__on_dfu_read()
        else:
            fatal_error("__on_dfu_read_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_dfu_state_internal(uuid_ptr, state):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_dfu_state is not None:
                device.__on_dfu_state(StarkDfuState(state))
            if (
                device.__listener is not None
                and device.__listener.on_dfu_state is not None
            ):
                device.__listener.on_dfu_state(StarkDfuState(state))
        else:
            fatal_error("__on_dfu_state_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, float)")
    def __on_dfu_progress_internal(uuid_ptr, progress):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_dfu_progress is not None:
                device.__on_dfu_progress(progress)
            if (
                device.__listener is not None
                and device.__listener.on_dfu_progress is not None
            ):
                device.__listener.on_dfu_progress(progress)
        else:
            fatal_error("__on_dfu_progress_internal:device unavailable for:" + uuid)
