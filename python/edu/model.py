class EMGData:
    @staticmethod
    def from_data(arr: list):
        return EMGData(arr[0], arr[1], arr[2:])

    def __init__(self, seq_num, lead_off_bits, channel_values):
        self.seq_num = seq_num
        self.lead_off_bits = lead_off_bits # 8个通道的导联脱落状态
        self.channel_values = channel_values  # 8 * 20, 8个通道，每个通道20个数据点

    def __repr__(self):
        return f"EMG seq_num={self.seq_num}, lead_off_bits={self.lead_off_bits}, len={len(self.channel_values)}, values={self.channel_values[:3]}"

class FlexData:
    @staticmethod
    def from_data(arr: list):
        return FlexData(arr[0], arr[1:])

    def __init__(self, seq_num, channel_values):
        self.seq_num = seq_num
        self.channel_values = channel_values  # 6个通道

    def __repr__(self):
        return f"FlexData seq_num={self.seq_num}, len={len(self.channel_values)}"

class IMUCord:

    def __init__(self, cord_x, cord_y, cord_z):
        self.cord_x = cord_x
        self.cord_y = cord_y
        self.cord_z = cord_z

    @staticmethod
    def from_json(json_obj):
        return IMUCord(
            json_obj["cordX"],
            json_obj["cordY"],
            json_obj["cordZ"],
        )

    def __repr__(self):
        return f"(x={self.cord_x}, y={self.cord_y}, z={self.cord_z})"


class IMUData:
    @staticmethod
    def from_data(arr: list):
        return IMUData(arr[0], arr[1:4], arr[4:7])

    def __init__(self, seqnum, acc, gyro):
        self.seqnum = seqnum & 0xFFFF
        self.acc = IMUCord(acc[0], acc[1], acc[2])
        self.gyro = IMUCord(gyro[0], gyro[1], gyro[2])

    def __repr__(self):
        return f"IMU seqnum={self.seqnum}, acc={self.acc}, gyro={self.gyro}"


class MagData:
    @staticmethod
    def from_data(arr: list):
        return MagData(arr[0], arr[1:4])

    def __init__(self, seqnum, acc):
        self.seqnum = seqnum & 0xFFFF
        self.data = IMUCord(acc[0], acc[1], acc[2])

    def __repr__(self):
        return f"MAG seqnum={self.seqnum}, data={self.data}"
