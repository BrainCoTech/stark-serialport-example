"""
Revo2灵巧手固件升级(DFU)示例

本示例演示如何对Revo2灵巧手进行固件升级，包括：
- 自动检测设备类型和协议
- 固件文件的验证和路径配置
- 执行固件升级过程和进度监控
- 处理升级状态和结果反馈

重要警告：
- 不同硬件版本必须使用对应的固件文件
- 使用错误的固件可能导致设备无法启动，需要拆机重新烧录
- 升级过程中请勿断开设备连接
- Revo2设备仅支持基础版Modbus固件
"""

import asyncio
import sys
import pathlib
import os
import time

from utils import setup_shutdown_event
from revo2_utils import *

# 重要！！！：不同硬件需使用相应固件，否则需要拆设备重新烧录
# 固件升级文件路径配置
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
logger.info(f"parent_dir: {parent_dir}")

# Revo2灵巧手基础版Modbus固件路径
# 二代灵巧手仅支持基础版固件
revo2_basic_ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "stark2",
    "stark2_fw_V0.0.10_20250714180644.bin",
)

# 全局变量用于异步事件处理
shutdown_event = None
main_loop = None


def on_dfu_state(_slave_id, state):
    """
    DFU状态变化回调函数

    在固件升级过程中，当状态发生变化时被调用。

    Args:
        _slave_id: 设备ID（未使用）
        state: DFU状态枚举值
    """
    logger.info(f"DFU STATE: {libstark.DfuState(state)}")
    dfu_state = libstark.DfuState(state)

    # 当升级完成或中止时，设置关闭事件
    if (
        dfu_state == libstark.DfuState.Completed
        or dfu_state == libstark.DfuState.Aborted
    ):
        if main_loop and shutdown_event:
            if not shutdown_event.is_set():
                logger.info("Using call_soon_threadsafe to set event")
                main_loop.call_soon_threadsafe(shutdown_event.set)


def on_dfu_progress(_slave_id, progress):
    """
    DFU进度更新回调函数

    在固件升级过程中，定期报告升级进度。

    Args:
        _slave_id: 设备ID（未使用）
        progress: 升级进度，范围0.0-1.0
    """
    logger.info(f"progress: {progress * 100.0 :.2f}%")


async def main():
    """
    主函数：执行固件升级流程
    """
    global main_loop
    main_loop = asyncio.get_running_loop()

    # 自动检测设备协议和连接参数
    (protocol, port_name, baudrate, slave_id) = await libstark.auto_detect_device("/dev/ttyUSB0") # 替换为实际的串口名称, 传None会尝试自动检测
    logger.info(
        f"Detected protocol: {protocol}, port: {port_name}, baudrate: {baudrate}, slave_id: {slave_id}"
    )

    # 使用Revo2基础版固件
    ota_bin_path = revo2_basic_ota_bin_path

    # 验证固件文件是否存在
    if not os.path.exists(ota_bin_path):
        logger.warning(f"OTA文件不存在: {ota_bin_path}")
        exit(0)
    else:
        logger.info(f"OTA文件路径: {ota_bin_path}")

    # 执行固件升级
    await start_dfu(port_name, baudrate, slave_id, ota_bin_path)

    # 可选：测试再次升级
    # await asyncio.sleep(5)
    # await start_dfu(port_name, baudrate, slave_id, ota_bin_path)

    sys.exit(0)


async def start_dfu(port_name, baudrate, slave_id, ota_bin_path):
    """
    启动固件升级过程

    Args:
        port_name: 串口名称
        baudrate: 波特率
        slave_id: 设备ID
        ota_bin_path: 固件文件路径
    """
    global shutdown_event
    shutdown_event = setup_shutdown_event(logger)

    # 建立Modbus连接
    client: libstark.PyDeviceContext = await libstark.modbus_open(port_name, baudrate)

    # 获取设备信息
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    start_time = time.perf_counter()

    # 开始固件升级
    wait_seconds = 5  # 等待设备进入DFU模式的时间
    await client.start_dfu(
        slave_id,
        ota_bin_path,
        wait_seconds,
        on_dfu_state,  # 状态变化回调
        on_dfu_progress,  # 进度更新回调
    )

    # 等待升级完成
    logger.info("Revo2 Modbus DFU, Waiting for DFU to complete...")
    await shutdown_event.wait()
    logger.info("DFU completed, shutdown event received!")
    elapsed = time.perf_counter() - start_time
    logger.info(f"Elapsed: {elapsed:.1f}s")

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")


if __name__ == "__main__":
    asyncio.run(main())
