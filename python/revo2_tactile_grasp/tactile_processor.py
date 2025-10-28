"""
电容式传感器-触觉处理器模块
实现接触检测、滑动检测、刚度识别等触觉感知算法
"""
import numpy as np
import asyncio
import pickle
import torch
import logging
import warnings

from stiffness_classifier import StiffnessClassifier
from logger import getLogger
from datetime import datetime

class AlgorithmModule:
    """算法模块：异步计算 contact / sliding / stiffness"""
    def __init__(self, data_reader, slip_threshold=1000, contact_threshold=10, device='cpu',
                 stiffness_detect_step=25):
        self.data_reader = data_reader
        self.contact_threshold = contact_threshold
        self.sliding_threshold = slip_threshold
        self.device = device
        self.stiffness_detect_step = stiffness_detect_step

        slip_model_path = 'models/slip_detection_model.pkl'
        stiffness_model_path = 'models/stiffness_detect_model.pth'

        # 加载所有检测模型
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", category=UserWarning, module="sklearn")
            with open(slip_model_path, "rb") as f_s:
                self.slip_detect_model = pickle.load(f_s)
        self.stiffness_detect_model = StiffnessClassifier(
            input_size=6, hidden_size=64, num_classes=12, bidirectional=False)
        self.stiffness_detect_model.load_state_dict(torch.load(stiffness_model_path, weights_only=True))

        # 初始化其他模块
        self._stiffness = [3.5, 12.5, 26.5, 39.5, 50, 68, 80, 60, 65, 78.5, 83, 88.5]
        self.finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        self.latest_result = {
            "status": "Collecting",
            # "contact": {},
            "sliding": {},
            "stiffness": None,
            "hard_released": True,
            "time": datetime.now().strftime("%H:%M:%S.") + f"{datetime.now().microsecond:06d}"
        }
        self.logger = getLogger(logging.INFO)
        self.running = False

    """简单 hardness(stiffness) 判断模型"""
    async def start(self):
        """异步循环，持续读取数据并分析"""
        self.running = True
        stiffness_ = []
        while self.running:
            buffer = self.data_reader.get_buffer()
            # print(buffer)
            # 如果 buffer 还没满，继续收集
            if len(buffer) < self.data_reader.buffer.maxlen:
                self.latest_result = {
                    "status": "Collecting",
                    "contact": {},
                    "sliding": {},
                    "stiffness": None,
                    "hand_released": True,
                    "time:": datetime.now().strftime("%H:%M:%S.") + f"{datetime.now().microsecond:06d}"
                }
            else:
                self.latest_result, stiffness_ = self._analyze(buffer, stiffness_)

            await asyncio.sleep(0.01477)  # 让出事件循环

    def stop(self):
        """停止算法循环"""
        self.running = False

    def _analyze(self, buffer, stiffness_):

        """
        分析 buffer 计算 contact / sliding / stiffness
        data.shape = (buffer_length, num_fingers, num_channels)
        """

        data = np.array([forces for _, forces in buffer])
        # buffer_length = data.shape[0]
        contact_info = {}
        sliding_info = {}
        stiffness = None

        """简单 contact 判断"""
        for i, finger in enumerate(self.finger_names):
            latest = data[-1, i, 0] # 法向力
            # 简单阈值检测，接触检测使用法向力
            if latest > self.contact_threshold:
                contact_info[finger] = {"force": int(latest)}

        """简单 sliding 判断模型"""
        # avg_force = np.mean(data, axis=0)
        # 基于力方差的滑动检测
        var_force = np.var(data.reshape((data.shape[0], -1)), axis=0)
        for i, finger in enumerate(self.finger_names):
            latest = data[-1, i, 1] # 切向力
            if latest > self.contact_threshold:
                contact_info[finger] = {"force": round(latest, 2)}
            # 切向力 + 方差分析 → 滑动检测    
            if var_force[2*i] > self.sliding_threshold and finger in contact_info:
                sliding_info[finger] = {"score": round(var_force[2*i], 2)}

        # 状态判断
        if contact_info:
            if sliding_info:
                status = "CONTACT_SLIDING"
            else:
                status = "CONTACT"
            if len(stiffness_) < self.stiffness_detect_step:
                stiffness = self._analyze_stiffness(data, self.stiffness_detect_model)
                stiffness_.append(stiffness)
                print(f"Stiffness:{stiffness_[-1]}")
                # 事件驱动：立刻通知控制器
                if self.controller:
                    self.controller.on_stiffness_update(stiffness)

        else:
            status = "NO_CONTACT"
            stiffness_ = []
            stiffness = None
            sliding_info = None

        # ---- 新增 hand_released 标志 ----
        hand_released = not bool(contact_info) and not bool(sliding_info)

        return {
            "status": status,
            "contact": contact_info,
            "sliding": sliding_info,
            "stiffness": stiffness,
            "hand_released": hand_released,
            "time": datetime.now().strftime("%H:%M:%S.") + f"{datetime.now().microsecond:06d}"
        }, stiffness_

    def get_result(self):
        """获取最新计算结果"""
        return self.latest_result

    def _analyze_stiffness(self, data, model):
        with torch.no_grad():
            data = data[:, :3, :2]  # 所有时间戳、前三个手指、前两个通道
            output = model(torch.tensor(data, dtype=torch.float32).unsqueeze(0))
            label_stiffness = torch.argmax(output, dim=1).item()
        stiffness = self._stiffness[label_stiffness]
        return stiffness
