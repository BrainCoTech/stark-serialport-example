import logging
import colorlog
import datetime
import os

# Custom time formatter, outputs RFC3339 format
class RFC3339Formatter(colorlog.ColoredFormatter):
    def formatTime(self, record, datefmt=None):
        # Create RFC3339 format timestamp, e.g.: 2025-07-24T02:04:23.466388Z
        dt = datetime.datetime.fromtimestamp(record.created)
        return dt.strftime('%Y-%m-%dT%H:%M:%S.%fZ')

# Create file formatter (without color)
class PlainRFC3339Formatter(logging.Formatter):
    def formatTime(self, record, datefmt=None):
        dt = datetime.datetime.fromtimestamp(record.created)
        return dt.strftime('%Y-%m-%dT%H:%M:%S.%fZ')

# Console log format (with color, shows full path)
CONSOLE_FORMAT = "%(asctime)s[%(log_color)s%(levelname)s%(reset)s][%(pathname)s:%(lineno)d] %(message)s"

# File log format (without color, shows full path)
FILE_FORMAT = "%(asctime)s[%(levelname)s][%(pathname)s:%(lineno)d] %(message)s"

# Create console formatter (with color)
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

# File log formatter (without color)
file_formatter = PlainRFC3339Formatter(FILE_FORMAT)

# Get root logger
logger = logging.getLogger()

# Create console handler
console_handler = logging.StreamHandler()
console_handler.setFormatter(console_formatter)

# Create file handler
# Ensure logs directory exists
os.makedirs('logs', exist_ok=True)

# Generate log filename with timestamp
log_filename = f"logs/python_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.log"
file_handler = logging.FileHandler(log_filename, encoding='utf-8')
file_handler.setFormatter(file_formatter)

# Add handlers to logger
logger.addHandler(console_handler)
logger.addHandler(file_handler)
logger.setLevel(logging.INFO)

# Example log messages
# logger.debug("This is a debug message")
# logger.info("This is an info message")
# logger.warning("This is a warning message")
# logger.error("This is an error message")
# logger.critical("This is a critical message")

# python
def getLogger(level=logging.INFO):
    logger.setLevel(level)
    return logger
