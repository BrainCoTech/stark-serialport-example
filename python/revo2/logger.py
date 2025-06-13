import logging
import colorlog

# 定义日志格式
# FORMAT = "%(log_color)s%(levelname)s %(name)s %(asctime)-15s %(filename)s:%(lineno)d %(message)s"
FORMAT = "%(log_color)s%(levelname)s %(asctime)-15s %(filename)s:%(lineno)d %(message)s"

# 创建一个 colorlog 的日志格式化器
formatter = colorlog.ColoredFormatter(
    FORMAT,
    datefmt=None,
    reset=True,
    log_colors={
        "DEBUG": "cyan",
        "INFO": "green",
        "WARNING": "yellow",
        "ERROR": "red",
        "CRITICAL": "red,bg_white",
    },
    secondary_log_colors={},
    style="%",
)

# 获取根日志记录器
logger = logging.getLogger()

# 创建一个控制台处理器
console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)

# 将处理器添加到日志记录器
logger.addHandler(console_handler)
logger.setLevel(logging.INFO)

# 示例日志消息
# logger.debug("This is a debug message")
# logger.info("This is an info message")
# logger.warning("This is a warning message")
# logger.error("This is an error message")
# logger.critical("This is a critical message")


# python
def getLogger(level=logging.INFO):
    logger.setLevel(level)
    return logger
