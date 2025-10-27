"""
Revo1灵巧手固件升级(DFU)示例

本示例演示如何对Revo1灵巧手进行固件升级，包括：
- 自动检测设备类型和协议
- 根据设备类型选择对应的固件文件
- 执行固件升级过程和进度监控
- 处理升级状态和结果反馈

重要警告：
- 不同硬件版本必须使用对应的固件文件
- 使用错误的固件可能导致设备无法启动，需要拆机重新烧录
- 升级过程中请勿断开设备连接
"""

import asyncio
import sys
import pathlib
import os
from utils import setup_shutdown_event
from revo1_utils import *

# 重要！！！：不同硬件需使用相应固件，否则需要拆设备重新烧录
# 固件升级文件路径配置
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
logger.info(f"parent_dir: {parent_dir}")

# Revo1触觉版固件路径
# 下载链接：https://app.brainco.cn/universal/bc-stark-sdk/firmware/touch/FW_MotorController_Release_SecureOTA_V1.8.53.F.ota
revo1_touch_ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "touch",
    "FW_MotorController_Release_SecureOTA_V1.8.53.F.ota",
)

# Revo1基础版Modbus固件路径
# 下载链接：https://app.brainco.cn/universal/bc-stark-sdk/firmware/modbus/FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota
revo1_basic_ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "modbus",
    # "FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota",
    "FW_MotorController_Release_SecureOTA_0.3.1.C.ota",
)

# Revo1 Protobuf协议固件路径
revo1_protobuf_ota_bin_path = os.path.join(
    parent_dir,
    "ota_bin",
    "protobuf",
    "FW_MotorController_Release_SecureOTA_485_9.2.7.ota",
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
    if state == libstark.DfuState.Completed or dfu_state == libstark.DfuState.Aborted:
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
    global shutdown_event, main_loop
    main_loop = asyncio.get_running_loop()
    shutdown_event = setup_shutdown_event(logger)

    # 自动检测设备协议和连接参数
    (protocol, port_name, baudrate, slave_id) = await libstark.auto_detect_device()
    logger.info(
        f"Detected protocol: {protocol}, port: {port_name}, baudrate: {baudrate}, slave_id: {slave_id}"
    )

    # 建立Modbus连接
    client: libstark.PyDeviceContext = await libstark.modbus_open(port_name, baudrate)

    # 根据设备类型选择对应的固件文件
    ota_bin_path = ""

    # 重要！！！：不同硬件需使用相应固件，否则需要拆设备重新烧录
    if protocol == libstark.StarkProtocolType.Modbus:
        # 获取设备信息以确定硬件类型
        device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
        logger.info(f"Device info: {device_info.description}")

        if device_info.is_revo1():
            if baudrate != libstark.Baudrate.Baud115200:
                logger.warning(
                    f"非115200波特率可能无法升级，请先将波特率设置为115200后再升级，当前波特率: {baudrate}"
                )

            if device_info.is_revo1_touch():
                # 触觉版使用触觉固件
                ota_bin_path = revo1_touch_ota_bin_path
            else:
                # 基础版使用Modbus固件
                ota_bin_path = revo1_basic_ota_bin_path
                # 可选：使用Protobuf固件
                # ota_bin_path = revo1_protobuf_ota_bin_path
    elif protocol == libstark.StarkProtocolType.Protobuf:
        # Protobuf协议设备使用Modbus基础版固件
        ota_bin_path = revo1_basic_ota_bin_path

    # 验证固件文件是否存在
    if not os.path.exists(ota_bin_path):
        logger.warning(f"OTA文件不存在: {ota_bin_path}")
        exit(0)
    else:
        logger.info(f"OTA文件路径: {ota_bin_path}")

    import time

    start_time = time.perf_counter()

    # 开始固件升级
    logger.info("start_dfu")
    wait_seconds = 5  # 等待设备进入DFU模式的时间
    await client.start_dfu(
        slave_id,
        ota_bin_path,
        wait_seconds,
        on_dfu_state,  # 状态变化回调
        on_dfu_progress,  # 进度更新回调
    )

    # 等待升级完成或关闭事件
    await shutdown_event.wait()
    elapsed = time.perf_counter() - start_time
    logger.info(f"Elapsed: {elapsed:.1f}s")

    # 关闭资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
