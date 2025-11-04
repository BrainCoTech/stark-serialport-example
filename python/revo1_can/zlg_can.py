import asyncio
import sys

from zlg_win import *
from can_utils import *


class Revo1CanController:
    """Revo1 CAN通信控制器"""

    def __init__(self, master_id: int = 1, slave_id: int = 1):
        self.master_id = master_id
        self.slave_id = slave_id
        self.client = libstark.PyDeviceContext()

    async def initialize(self):
        """初始化CAN连接"""
        try:
            # 初始化ZLG CAN设备
            zlgcan_open()

            # 设置回调函数
            libstark.set_can_tx_callback(self._can_send)
            libstark.set_can_rx_callback(self._can_read)

            logger.info(
                f"CAN连接初始化成功 - Master ID: {self.master_id}, Slave ID: {self.slave_id}"
            )
            return True

        except Exception as e:
            logger.error(f"CAN连接初始化失败: {e}")
            return False

    def _can_send(self, _slave_id: int, can_id: int, data: list) -> bool:
        """CAN消息发送"""
        try:
            if not zlgcan_send_message(can_id, bytes(data)):
                logger.error("发送CAN消息失败")
                return False
            return True
        except Exception as e:
            logger.error(f"CAN发送异常: {e}")
            return False

    def _can_read(self, _slave_id: int) -> tuple:
        """CAN消息接收"""
        try:
            recv_msg = zlgcan_receive_message()
            if recv_msg is None:
                return 0, bytes([])

            can_id, data = recv_msg
            # 可选：启用详细调试日志
            # logger.debug(f"接收CAN - ID: {can_id:029b}, Data: {bytes(data).hex()}")
            return can_id, data

        except Exception as e:
            logger.error(f"CAN接收异常: {e}")
            return 0, bytes([])

    async def get_device_info(self):
        """获取设备信息"""
        try:
            device_info = await self.client.get_device_info(self.slave_id)
            logger.info(f"设备信息: {device_info.description}")
            return device_info
        except Exception as e:
            logger.error(f"获取设备信息失败: {e}")
            return None

    async def change_slave_id(self, new_slave_id: int):
        """修改设备从站ID（谨慎使用，设备会重启）"""
        try:
            await self.client.set_slave_id(self.slave_id, new_slave_id)
            logger.info(f"从站ID已修改为 {new_slave_id}，设备将重启...")
            return True
        except Exception as e:
            logger.error(f"修改从站ID失败: {e}")
            return False

    async def configure_device(self):
        """配置设备参数"""
        try:
            # 选项: 修改从站ID（如果需要，取消注释并设置新ID）
            # WARNING: 设备会重启，程序将退出
            # if await self.change_slave_id(new_slave_id=2):
            #     sys.exit(0)

            # 禁用自动校准并执行手动校准
            # await self.client.set_auto_calibration(self.slave_id, False) # 禁用开机后自自动校准
            # await self.client.calibrate_position(self.slave_id)
            # await self.client.set_auto_calibration(self.slave_id, True)  # 启用开机后自动校准
            # auto_calibration_enabled = await self.client.get_auto_calibration_enabled(self.slave_id)
            # logger.info(f"开机自动校准: {auto_calibration_enabled}")

            # 配置：Turbo模式（可选，取消注释以启用）
            # await self.configure_turbo_mode()

            return True
        except Exception as e:
            logger.error(f"配置设备失败: {e}")
            return False

    async def configure_turbo_mode(self):
        """配置Turbo模式"""
        try:
            # 启用Turbo模式
            await self.client.set_turbo_mode_enabled(self.slave_id, True)

            # 设置Turbo参数
            turbo_interval = 200  # 握紧间隔时间（毫秒）
            turbo_duration = 300  # 握紧持续时间（毫秒）
            turbo_conf = libstark.TurboConfig(turbo_interval, turbo_duration)
            await self.client.set_turbo_config(self.slave_id, turbo_conf)

            # 验证配置
            turbo_mode_enabled = await self.client.get_turbo_mode_enabled(self.slave_id)
            turbo_config = await self.client.get_turbo_config(self.slave_id)

            logger.info(f"Turbo模式: {turbo_mode_enabled}")
            logger.info(
                f"Turbo配置 - 间隔: {turbo_config.interval}ms, 持续: {turbo_config.duration}ms"
            )

        except Exception as e:
            logger.error(f"配置Turbo模式失败: {e}")

    async def finger_position_examples(self):
        """手指位置控制示例"""
        logger.info("=== 手指位置控制示例 ===")

        # 示例1: 逐个握拳动作
        logger.info("示例1: 逐个手指握拳")
        positions = [
            [20, 0, 0, 0, 0, 0],  # 拇指
            [20, 30, 0, 0, 0, 0],  # 拇指+食指
            [20, 30, 50, 0, 0, 0],  # +中指
            [20, 30, 50, 70, 0, 0],  # +无名指
            [20, 30, 50, 70, 80, 0],  # +小指
            [20, 30, 50, 70, 80, 90],  # +手腕
        ]

        for i, pos in enumerate(positions):
            logger.info(f"步骤 {i+1}: {pos}")
            await self.client.set_finger_positions(self.slave_id, pos)
            await asyncio.sleep(0.8)

        await asyncio.sleep(1.0)

        # 示例2: 预设手势
        logger.info("示例2: 预设手势")
        gestures = {
            "张开手": [0, 0, 0, 0, 0, 0],
            "指向": [0, 30, 0, 0, 0, 0],
            "胜利手势": [0, 30, 80, 0, 0, 0],
            "OK手势": [80, 30, 80, 0, 0, 0],
            "握拳": [80, 30, 100, 100, 100, 100],
        }

        for gesture_name, positions in gestures.items():
            logger.info(f"执行手势: {gesture_name} - {positions}")
            await self.client.set_finger_positions(self.slave_id, positions)
            await asyncio.sleep(1.5)

        # 示例3: 抓取动作模拟
        logger.info("示例3: 抓取动作模拟")
        grab_sequence = [
            ([0, 0, 0, 0, 0, 0], "初始位置"),
            ([30, 40, 60, 80, 80, 0], "抓取准备"),
            ([60, 70, 90, 100, 100, 0], "抓取完成"),
        ]

        for positions, description in grab_sequence:
            logger.info(f"{description}: {positions}")
            await self.client.set_finger_positions(self.slave_id, positions)
            await asyncio.sleep(1.0)

    async def finger_speed_examples(self):
        """手指速度控制示例"""
        logger.info("=== 手指速度控制示例 ===")

        # 停止所有运动
        await self.client.set_finger_speeds(self.slave_id, [0] * 6)
        logger.info("所有手指停止运动")

    async def single_finger_control_example(self, finger_id: libstark.FingerId):
        """单个手指控制示例"""
        logger.info(f"=== 单个手指控制示例 - {finger_id} ===")

        # 位置控制
        logger.info("位置控制测试")
        await self.client.set_finger_position(self.slave_id, finger_id, 100)  # 最大位置
        await asyncio.sleep(1.0)
        await self.client.set_finger_position(self.slave_id, finger_id, 0)  # 初始位置
        await asyncio.sleep(1.0)

        # 速度控制
        logger.info("速度控制测试")
        await self.client.set_finger_speed(
            self.slave_id, finger_id, 100
        )  # 正向最大速度
        await asyncio.sleep(1.0)
        await self.client.set_finger_speed(
            self.slave_id, finger_id, -100
        )  # 反向最大速度
        await asyncio.sleep(1.0)
        await self.client.set_finger_speed(self.slave_id, finger_id, 0)  # 停止

    async def get_motor_status(self) -> libstark.MotorStatusData:
        """获取电机状态"""
        try:
            status = await self.client.get_motor_status(self.slave_id)
            # 可选：启用详细状态日志
            # logger.info(f"电机状态: {status.description}")
            return status
        except Exception as e:
            logger.error(f"获取电机状态失败: {e}")
            return None  # type: ignore

    async def monitor_motor_status(self, interval: float = 0.001):
        """持续监控电机状态"""
        logger.info(f"开始监控设备 {self.slave_id:02x} 的电机状态")

        while True:
            try:
                await self.get_motor_status()
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                logger.info("电机状态监控已取消")
                break
            except Exception as e:
                logger.error(f"电机状态监控异常: {e}")
                await asyncio.sleep(1)

    async def demo_task(self):
        """演示任务"""
        # 获取设备信息
        await self.get_device_info()

        # 配置设备参数，根据需要启用
        # await self.configure_device()

        # 根据需要启用以下演示
        await self.finger_position_examples()
        await asyncio.sleep(1.0)

        await self.finger_speed_examples()
        await asyncio.sleep(1.0)

        await self.single_finger_control_example(libstark.FingerId.Pinky)

        # 设置最终手势（装箱手势）
        # await self.client.set_finger_positions(
        #     self.slave_id, [80, 30, 100, 100, 100, 100]
        # )

    def cleanup(self):
        """清理资源"""
        try:
            zlgcan_close()
            logger.info("CAN连接已关闭")
        except Exception as e:
            logger.error(f"清理资源时出错: {e}")


async def main():
    """主函数"""
    controller = Revo1CanController(master_id=1, slave_id=1)

    try:
        # 初始化连接
        if not await controller.initialize():
            logger.error("初始化失败，程序退出")
            return

        # 设置关闭事件监听
        shutdown_event = setup_shutdown_event(logger)

        # 创建任务
        tasks = []

        # 启动演示任务
        demo_task = asyncio.create_task(controller.demo_task())
        tasks.append(demo_task)

        # 可选：启动电机状态监控
        monitor_task = asyncio.create_task(controller.monitor_motor_status())
        tasks.append(monitor_task)

        # 等待关闭信号
        await shutdown_event.wait()
        logger.info("收到关闭信号，停止所有任务...")

        # 取消所有任务
        for task in tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    except Exception as e:
        logger.error(f"程序执行异常: {e}")

    finally:
        controller.cleanup()
        sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
