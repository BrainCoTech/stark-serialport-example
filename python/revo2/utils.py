import asyncio
import platform
import signal
import sys

def setup_shutdown_event(logger=None):
    # 创建一个事件用于关闭
    shutdown_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    def shutdown_handler():
        if logger is not None:
            logger.info("Shutdown signal received")
        else:
            print("Shutdown signal received")
        # if not shutdown_event.is_set():
        #     print("Using call_soon_threadsafe to set event")
        #     loop.call_soon_threadsafe(shutdown_event.set)
        sys.exit(0)

    if platform.system() != "Windows":
        # 注册信号处理器
        loop.add_signal_handler(signal.SIGINT, shutdown_handler)
        loop.add_signal_handler(signal.SIGTERM, shutdown_handler)
    else:
        # Windows下使用不同的方式处理信号
        signal.signal(signal.SIGINT, lambda s, f: shutdown_handler())
        signal.signal(signal.SIGTERM, lambda s, f: shutdown_handler())

    return shutdown_event

