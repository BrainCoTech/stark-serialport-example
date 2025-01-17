from enum import IntEnum  # Enum declarations

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