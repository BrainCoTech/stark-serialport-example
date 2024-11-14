import asyncio
import platform
import signal
from lib.stark_logging import SKLog


def setup_shutdown_event():
    # 创建一个事件用于关闭
    shutdown_event = asyncio.Event()

    def shutdown_handler():
        SKLog.info("Shutdown signal received")
        shutdown_event.set()

    # 注册信号处理器
    loop = asyncio.get_running_loop()
    if platform.system() != "Windows":
        loop.add_signal_handler(signal.SIGINT, shutdown_handler)
        loop.add_signal_handler(signal.SIGTERM, shutdown_handler)

    return shutdown_event
