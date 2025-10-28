"""
抓握控制器模块
基于电容式传感器-触觉反馈的自适应机械手抓握控制系统
"""
import asyncio
import logging
from logger import getLogger
from typing import List

class TactileGraspController:
    """Controller 模块：接收算法结果，控制仿生手"""
    def __init__(self, client, slave_id: int, algorithm, enable_recording: bool = True):
        self.client = client
        self.slave_id = slave_id
        self.algorithm = algorithm
        self.running = False
        self.control_mode = "AUTO"
        self.finger_num = 0  # 控制的手指数量，5=握拳 / 3=三指捏 / 2=两指捏
        # 6 个关节，第0是 base，1~5 对应手指
        self.current_currents = [0] * 6
        self.latest_commanded_currents = [0] * 6
        self.current_set = False  # 标志位：基础电流是否已设置

        self._finger_map = {"Thumb": 0, "Index": 2, "Middle": 3, "Ring": 4, "Pinky": 5}
        self.enable_recording = enable_recording
        self.data_buffer = []  # 每行: [status, thumb, index, middle, ring, pinky]
        self.logger = getLogger(logging.INFO)
        self.contacted = False
        self.current_ctrl_list = [0] * 6


    def set_control_mode(self, mode: str):
        self.control_mode = mode.upper()
        print(f"[Controller] control_mode <- {self.control_mode}")

    def update_commanded_currents(self, currents: List[int]):
        self.latest_commanded_currents = currents.copy()
        self.current_currents = currents.copy()

    @staticmethod
    def adjust_currents(cur_currents, base_current, finger_num):
        """
        根据 finger_num 设置指定手指的电流为 base_current
        cur_currents: 当前电流列表，长度必须为6
        base_current: 基础电流值
        finger_num: 要控制的手指数量（2~5）
        """
        if len(cur_currents) != 6:
            raise ValueError("currents 必须有 6 个值")
        
        if finger_num < 2 or finger_num > 5:
            print(f"finger_num 必须是2,3,4,5之一, 当前值为: {finger_num}")
            raise ValueError(f"finger_num 必须是2,3,4,5之一, 当前值为: {finger_num}")

        # 根据 finger_num 决定哪些手指需要赋值
        finger_map = {
            2: [0, 2],
            3: [0, 2, 3],
            4: [0, 2, 3, 4],
            5: [0, 2, 3, 4, 5]
        }

        target_indices = finger_map[finger_num]

        new_currents = []
        for i, val in enumerate(cur_currents):
            if i == 1:  # 第二个值保持为0
                new_currents.append(100)
            elif i in target_indices:
                new_currents.append(base_current)
            else:
                new_currents.append(0)

        return new_currents

    async def grasp_with_current(self, finger_mask: List[int]):
        """
        基于电流控制的抓握方法
        
        参数:
            finger_mask: 长度6的列表，对应 [Thumb_Base, Thumb_Flex, Index, Middle, Ring, Pinky]
            # 含义说明：
            # - Thumb_Base : 拇指根部弯曲（掌指关节）
            # - Thumb_Flex : 拇指指节弯曲（指间关节）
            # - Index      : 食指弯曲
            # - Middle     : 中指弯曲
            # - Ring       : 无名指弯曲
            # - Pinky      : 小指弯曲
            >0 表示抓握动作
            <0 表示放开动作
            0 表示保持不动
        """
        if len(finger_mask) != 6:
            raise ValueError("finger_mask must be length 6")

        # -------- 其他手指电流控制 --------
        currents_initial = []
        currents_run = []
        for i, val in enumerate(finger_mask):
            if i == 1:  # Thumb_Flex
                currents_initial.append(500)
                currents_run.append(500)
            else:
                if val > 0:     # 抓握
                    currents_initial.append(100)
                    currents_run.append(100)
                elif val < 0:   # 放开
                    currents_initial.append(-1000)
                    currents_run.append(-1000)
                else:           # 保持
                    currents_initial.append(0)
                    currents_run.append(0)

        await self.client.set_finger_currents(self.slave_id, currents_initial)  # 初始电流
        await asyncio.sleep(0.05)
        await self.client.set_finger_currents(self.slave_id, currents_run)      # 运行电流

        self.update_commanded_currents(currents_run)

    async def release_all_fingers(self):
        """
        释放全部手指
        """
        await self.client.set_finger_positions_and_speeds(self.slave_id, [0, 0, 0, 0, 0, 0], [300]*6)
        # await asyncio.sleep(0.05)

    async def grasp_with_speed(
            self,
            num_of_fingers: int = 5,
            position: List[int] = None,
            speed: int = 100,
            thumb_base_position: int = 500,
            thumb_flex_position: int = 650
    ):
        """
        基于位置和速度控制的抓握方法
        
        参数:
            num_of_fingers: 控制的手指数量（2-5）
            position: 目标位置列表
            speed: 运动速度
            thumb_position: 拇指位置
        """
        self.finger_num = num_of_fingers
        position_all_default = [thumb_base_position, thumb_flex_position, 850, 1000, 1000, 1000]

        if num_of_fingers == 0:
            # 所有手指归零（除了拇指位置仍然控制位置）
            position_all = [0, 0, 0, 0, 0, 0]
            # thumb_position = 0
        else:
            if position is None:
                position_all = position_all_default[:num_of_fingers + 1] + [0] * (5 - num_of_fingers)
            elif len(position) != num_of_fingers:
                raise ValueError("position must be equal to number of fingers")
            else:
                position_all = position.copy()
                position_all.insert(1, int(thumb_flex_position))
                position_all += [0] * (5 - num_of_fingers)

        # 速度列表
        speed_list = [int(speed), int(speed) * 2, int(speed), int(speed) * 2] + [int(speed)] * 2

        # current_ctrl_list 可以用来标记动作方向
        self.current_ctrl_list = [(i > 0) - (i < 0) for i in position_all]

        # -------- Thumb_FLEX 位置控制 --------
        await self.client.set_finger_positions_and_speeds(self.slave_id, [0, int(thumb_flex_position / 10), 0, 0, 0, 0], [500]*6)
        await asyncio.sleep(0.05)

        # -------- 其他手指速度控制 --------
        await self.client.set_finger_positions_and_speeds(self.slave_id, position_all, speed_list)
        
        # TODO: 物理碰撞检测及处理

    def compute_current(self, stiffness: float):
        """根据 stiffness 计算电流"""
        if stiffness < 10:
            return 0
        elif stiffness > 60:
            return 500
        else:
            return 100

    def on_stiffness_update(self, stiffness: float):
        if not self.running or self.finger_num == 0:
            return
        
        """事件驱动回调：stiffness 一算出就立刻执行"""
        if not self.current_set:
            base_current = self.compute_current(stiffness)
            try: 
                self.current_currents = self.adjust_currents(self.current_currents, base_current, self.finger_num)
                self.client.set_finger_currents(self.slave_id, self.current_currents)
                self.logger.info(f"[Controller] detected stiffness: {stiffness}")
                self.logger.info(f"[Controller] set base current: {self.current_currents}")
                self.current_set = True
                self.latest_commanded_currents = self.current_currents.copy()
                asyncio.sleep(1.5)  # 停止1s，不影响滑动检测的情况
            except Exception as e:
                print(f"Error in on_stiffness_update: {e}")

    async def start(self):
        """启动控制循环"""
        # base_current = 0
        self.running = True
        # self._control_task = asyncio.create_task(self._control_loop())
        print("[Controller] control loop started")
        while self.running:
            result = self.algorithm.get_result()  # 可以一直读取
            # print("Controller read data from algorithm")
            # print(result)
            if result and self.control_mode == "AUTO":
                contact_info = result.get("contact", {})
                sliding_info = result.get("sliding", {})
                stiffness_level = result["stiffness"]
                hand_released = result.get("hand_released", True)

                # ---- 松手检测 ----
                if hand_released:
                    await asyncio.sleep(1)  # 防止误判
                    if not self.current_set:  # 如果当前已经是松手状态，无需重复操作
                        await asyncio.sleep(0.01)
                        continue
                    self.current_set = False
                    self.contacted = False
                    self.current_currents = [0] * 6
                    print("[Controller] hand released, ready for next grasp")
                    continue

                # ---- 接触检测阶段 ----
                if contact_info and not self.contacted:
                    self.contacted = True
                    print("Contacted for the first time")


            await asyncio.sleep(0.001)


    async def stop(self):
        """停止控制循环"""
        self.running = False
        if hasattr(self, "_control_task"):
            await self._control_task
        print("[Controller] control loop stopped")
