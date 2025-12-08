"""
Revo2 触觉灵巧手抓握示例
基于电容式传感器-触觉反馈灵巧手演示抓握动作
"""
import asyncio
from revo2_utils import *
from data_reader import DataReader
from tactile_processor import AlgorithmModule
from grasp_controller import TactileGraspController

async def main():
    client, slave_id = await open_modbus_revo2()

    # 模块初始化
    sleep_time = 0.0095
    buffer_length = 10  # 存10帧进行计算
    data_reader = DataReader(client, slave_id, sleep_time, history_len=buffer_length)
    
    algorithm = AlgorithmModule(data_reader, slip_threshold=1000)
    controller = TactileGraspController(client, slave_id, algorithm, enable_recording=True)
    controller.set_control_mode("AUTO")
    
    # 通过回调显式注入：只订阅 stiffness 事件
    algorithm.register_callback("stiffness", controller.on_stiffness_update)

    # 启动异步任务
    asyncio.create_task(data_reader.start())
    asyncio.create_task(algorithm.start())
    asyncio.create_task(controller.start())

    # 演示抓握动作之前，先释放手指
    controller.set_control_mode("AUTO")
    await controller.release_all_fingers()
    await asyncio.sleep(2)

    print('演示两指抓握')
    await controller.grasp_with_speed(num_fingers=2)
    await asyncio.sleep(3)
    print('释放')
    await controller.release_all_fingers()
    await asyncio.sleep(2)
    
    print('演示三指抓握')
    await controller.grasp_with_speed(num_fingers=3)
    await asyncio.sleep(3)
    print('释放')
    await controller.release_all_fingers()
    await asyncio.sleep(2)
    
    print('演示五指抓握')
    await controller.grasp_with_speed(num_fingers=5)
    await asyncio.sleep(5)
    print('释放')
    await controller.release_all_fingers()
    await asyncio.sleep(2)
    
    # 停止并清理资源
    data_reader.stop()
    algorithm.stop()
    await controller.stop()
    logger.info("Controller stopped")
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    logger.info("Program ended")

# 触觉传感器校准
# async def touch_sensor_calibrate(client, slave_id):
#     await client.touch_sensor_reset(slave_id, 0x1f)
#     await client.touch_sensor_calibrate(slave_id, 0x1f) # 校准零漂
#     await asyncio.sleep(1)

if __name__ == "__main__":
    # warnings.filterwarnings("ignore")
    asyncio.run(main())
