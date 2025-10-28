# data_reader.py
import asyncio
import time
import os
import csv
from collections import deque
import numpy as np

def save_csv(data, csv_path, csv_name, header):
    if not os.path.exists(csv_path):
        os.mkdir(csv_path)
    nf_file = open(os.path.join(csv_path, csv_name), 'w', newline='')
    nf_writer = csv.writer(nf_file)
    nf_writer.writerow(header)
    for i in range(data.shape[0]):
        data_w = data[i, :, :].flatten()
        data_w = np.insert(data_w, 0, i)
        nf_writer.writerow(data_w)
    nf_file.close()
    print("采样结束，文件已保存")

class DataReader:
    """读数据模块，异步读取触觉数据到固定长度 buffer
    in: client, slave_id, buffer_length
    """
    def __init__(self, client, slave_id, sleep_time: float, history_len: int=5):
        self.client = client
        self.slave_id = slave_id
        self.buffer = deque(maxlen=history_len)
        self.running = False
        self.finger_list = [0, 1, 2, 3, 4]
        self.force_list = [0, 1]  # 决定每个模组采集什么通道
        self.sleep_time = sleep_time

    async def start(self):
        self.running = True
        print('read_start')
        while self.running:
            try:
                _new_data = await self.parse_touch_status(self.slave_id)
                # print(_new_data)
                timestamp = time.time()
                self.buffer.append((timestamp, _new_data))
                # print(_new_data[:][:])
            except Exception as e:
                print("[DataReader] Error:", e)
            await asyncio.sleep(5e-3)

    def stop(self):
        self.running = False

    def get_buffer(self):
        return list(self.buffer)

    async def parse_touch_status(self, slave_id):
        """
        如果 client.get_touch_sensor_status 返回的是复杂对象，解析为 normal list/tangential list。
        假设 touch_status[i] 有 normal_force1, tangential_force1 属性（和你原来一致）。
        """
        try:
            # initialize data matrix
            _force_attr = ["normal_force1", "tangential_force1", "tangential_direction1", "self_proximity1"]
            _finger_attr = ["thumb", "index", "middle", "ring", "pinky"]
            _data = []
            touch_data = await self.client.get_touch_sensor_status(slave_id)
            touch_data_raw = await self.client.get_touch_sensor_raw_data(slave_id)

            # record data
            for finger in self.finger_list:
                if finger not in range(5):
                    raise ValueError(f"Invalid finger number {finger}.")
                _data.append([])
                for channel in self.force_list:
                    if channel in range(4):
                        _data[finger].append(getattr(touch_data[finger], _force_attr[channel]))
                    elif channel in range(4, 10):
                        _data[finger].append(getattr(touch_data_raw, _finger_attr[finger])[channel - 4])
                    else:
                        raise ValueError(f"Invalid channel number {channel}.")
            # _data = np.array(_data, dtype=np.int32)
            await asyncio.sleep(self.sleep_time)

        except Exception as e:
            _data = [[0.0] * len(self.force_list)] * len(self.finger_list)
            print(f"Error: {e}")

        return _data  # 5x4 list