import logging
import colorlog
import datetime
import os

# 自定义时间格式化器，输出RFC3339格式
class RFC3339Formatter(colorlog.ColoredFormatter):
    def formatTime(self, record, datefmt=None):
        # 创建RFC3339格式的时间戳，例如: 2025-07-24T02:04:23.466388Z
        dt = datetime.datetime.fromtimestamp(record.created)
        return dt.strftime('%Y-%m-%dT%H:%M:%S.%fZ')

# 创建文件格式化器（不带颜色）
class PlainRFC3339Formatter(logging.Formatter):
    def formatTime(self, record, datefmt=None):
        dt = datetime.datetime.fromtimestamp(record.created)
        return dt.strftime('%Y-%m-%dT%H:%M:%S.%fZ')

# 控制台日志格式（带颜色，显示完整路径）
CONSOLE_FORMAT = "%(asctime)s[%(log_color)s%(levelname)s%(reset)s][%(pathname)s:%(lineno)d] %(message)s"

# 文件日志格式（不带颜色，显示完整路径）
FILE_FORMAT = "%(asctime)s[%(levelname)s][%(pathname)s:%(lineno)d] %(message)s"

# 创建控制台格式化器（带颜色）
console_formatter = RFC3339Formatter(
    CONSOLE_FORMAT,
    datefmt=None,
    reset=True,
    log_colors={
        "DEBUG": "cyan",
        "INFO": "green",
        "WARNING": "yellow",
        "ERROR": "red",
        "CRITICAL": "red,bg_white",
    },
     secondary_log_colors={
        'module_color': {
            'DEBUG': 'blue',
            'INFO': 'blue',
            'WARNING': 'blue',
            'ERROR': 'blue',
            'CRITICAL': 'blue'
        },
        'file_color': {
            'DEBUG': 'purple',
            'INFO': 'purple',
            'WARNING': 'purple',
            'ERROR': 'purple',
            'CRITICAL': 'purple'
        }
    },
    style="%",
)

# 文件日志格式（不带颜色）
file_formatter = PlainRFC3339Formatter(FILE_FORMAT)

# 获取根日志记录器
logger = logging.getLogger()

# 创建一个控制台处理器
console_handler = logging.StreamHandler()
console_handler.setFormatter(console_formatter)

# 创建文件处理器
# 确保logs目录存在
os.makedirs('logs', exist_ok=True)

# 生成带时间戳的日志文件名
log_filename = f"logs/python_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.log"
file_handler = logging.FileHandler(log_filename, encoding='utf-8')
file_handler.setFormatter(file_formatter)

# 将处理器添加到日志记录器
logger.addHandler(console_handler)
logger.addHandler(file_handler)
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
