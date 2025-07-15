import asyncio
import sys
import signal
import os
import platform
import pathlib
import logging
from logger import getLogger
# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

from bc_stark_sdk import main_mod
libethercat = main_mod.ethercat
libstark = main_mod.stark
libstark.init_config(libstark.StarkHardwareType.Revo2Basic, libstark.StarkProtocolType.EtherCAT)

def setup_shutdown_event(logger):
    # 创建一个事件用于关闭
    shutdown_event = asyncio.Event()

    def shutdown_handler():
        logger.info("Shutdown signal received")
        shutdown_event.set()

    # 注册信号处理器
    loop = asyncio.get_running_loop()
    if platform.system() != "Windows":
        loop.add_signal_handler(signal.SIGINT, shutdown_handler)
        loop.add_signal_handler(signal.SIGTERM, shutdown_handler)

    return shutdown_event
