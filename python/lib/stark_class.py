from lib.stark_enums import *
from cffi import FFI

ffi = FFI()

class StarkConf:
    isModbus = False

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
        if StarkConf.isModbus:
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
        if StarkConf.isModbus and self.finger_currents == [0] * 6 and self.finger_states == [MotorState.idle] * 6:
            return True
        if not StarkConf.isModbus and self.finger_currents == [0] * 6 and self.finger_pwms == [0] * 6:
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
        if StarkConf.isModbus:
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