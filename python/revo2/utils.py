import asyncio
import platform
import signal
import inspect


def inspect_class(cls):
    print(f"{cls} methods and attributes:")
    for name, member in inspect.getmembers(cls):
        if inspect.isfunction(member):
            print(f"Method: {name}")
        elif not name.startswith("__"):
            print(f"Attribute: {name} = {member}")
        # else:
        #     print(f"name: {name}, member:{member}")


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


