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
from collections import defaultdict
from typing import Callable, Any

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
        self.num_fingers = 0
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
            "hand_released": True,
            "contact": {},
            "sliding": {},
            "stiffness": None,
            "time": datetime.now().strftime("%H:%M:%S.") + f"{datetime.now().microsecond:06d}"
        }
        self.logger = getLogger(logging.INFO)
        self.running = False

        # Callback registry: event_name -> list[Callable]
        self._callbacks = defaultdict(list)

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
                    "hand_released": True,
                    "contact": {},
                    "sliding": {},
                    "stiffness": None,
                    "time:": datetime.now().strftime("%H:%M:%S.") + f"{datetime.now().microsecond:06d}"
                }
            else:
                self.latest_result, stiffness_ = self._analyze(buffer, stiffness_)

            await asyncio.sleep(0.01477)  # 让出事件循环

    def stop(self):
        """停止算法循环"""
        self.running = False

    # ---------- Callback management ----------
    def register_callback(self, event: str, fn: Callable[[Any], Any]):
        """Register a callback for a given event name (e.g., 'stiffness')."""
        if not callable(fn):
            raise TypeError("callback must be callable")
        self._callbacks[event].append(fn)

    def unregister_callback(self, event: str, fn: Callable[[Any], Any]):
        """Remove a previously registered callback for an event."""
        if event in self._callbacks and fn in self._callbacks[event]:
            self._callbacks[event].remove(fn)
            if not self._callbacks[event]:
                del self._callbacks[event]

    def _emit(self, event: str, payload: Any):
        """Invoke all callbacks registered to an event; sync only for now."""
        callbacks = list(self._callbacks.get(event, []))
        for cb in callbacks:
            try:
                cb(payload)
            except Exception as e:
                self.logger.error(f"Callback error on event '{event}': {e}")

    def _analyze(self, buffer, stiffness_):
        """
        分析 buffer 计算 contact / sliding / stiffness
        data.shape = (buffer_length, num_fingers, num_channels)
        channel 0 = normal force
        channel 1 = tangential force
        """

        # ===== 构造数据 =====
        data = np.array([forces for _, forces in buffer])
        contact_info = {}
        sliding_info = {}
        stiffness = None

        # 防御式检查
        if data.ndim != 3:
            raise ValueError(f"Invalid data shape: {data.shape}, expect (T, F, C)")

        T, F, C = data.shape

        # 至少需要 2 个通道（法向 + 切向）
        if C < 2:
            raise ValueError("num_channels must be >= 2 (normal + tangential)")

        # ===== 当前帧数据 =====
        latest_forces = data[-1]           # shape: (num_fingers, num_channels)
        normal_forces = latest_forces[:, 0]
        tangential_forces = latest_forces[:, 1]

        # ===== 接触检测：只使用【法向力】=====
        contact_mask = normal_forces > self.contact_threshold

        for i, (finger, is_contact) in enumerate(zip(self.finger_names, contact_mask)):
            if is_contact:
                contact_info[finger] = {
                    "force": round(float(normal_forces[i]), 2)
                }

        # ===== 如果没有接触，直接返回 =====
        if not contact_info:
            return {
                "status": "NO_CONTACT",
                "hand_released": True,
                "stiffness": None,
                "sliding": None,
                "contact": {},
                "time": datetime.now().strftime("%H:%M:%S.") + f"{datetime.now().microsecond:06d}"
            }, stiffness_

        # ===== 滑动检测：只使用【切向力的方差】=====
        # variance shape = (num_fingers, num_channels)
        variance = np.var(data, axis=0)

        for i, finger in enumerate(self.finger_names):
            # 只有接触中的手指才可能滑动
            if finger in contact_info:
                tangential_var = variance[i, 1]   # 切向力方差
                if tangential_var > self.sliding_threshold:
                    sliding_info[finger] = {
                        "score": round(float(tangential_var), 4)
                    }

        # ===== 判断完整接触区域（用于 stiffness）=====
        finger_indices = {
            2: slice(0, 2),  # 拇指 + 食指
            3: slice(0, 3),  # 拇指 + 食指 + 中指
            5: slice(None)   # 全部
        }.get(self.num_fingers, slice(0))

        full_contact = bool(np.all(contact_mask[finger_indices]))

        # ===== 刚度检测 =====
        if full_contact and len(stiffness_) < self.stiffness_detect_step:
            stiffness = self._analyze_stiffness(data, self.stiffness_detect_model)
            stiffness_.append(stiffness)

            # Emit stiffness event (replaces direct controller reference)
            self._emit("stiffness", stiffness)

        # ===== 状态判断 =====
        status = "CONTACT_SLIDING" if sliding_info else "CONTACT"

        return {
            "status": status,
            "hand_released": False,
            "stiffness": stiffness,
            "sliding": sliding_info if sliding_info else None,
            "contact": contact_info,
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
