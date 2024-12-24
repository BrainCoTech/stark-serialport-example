import inspect
import logging
import os
from binascii import b2a_hex

class CustomFormatter(logging.Formatter):
    def __init__(self, formater, stackIndex=9, addColor=False):
        super().__init__(formater)
        self.stackIndex = stackIndex
        self.addColor = addColor

    def format(self, record:logging.LogRecord):
        # 获取调用者信息
        frame = inspect.stack()[self.stackIndex] # 前面的栈帧是logging和函数本身
        module = inspect.getmodule(frame[0])
        # record.calling_file = module.__file__ if module else 'unknown'
        record.calling_file = getattr(module, "__file__", "unknown")
        record.calling_line = frame[2]
        if self.addColor:
            record.color_msg = colors_map[record.levelname.upper()] + record.msg + '\033[0m'
        return super().format(record)

class SKLog(object):
    """Class to hide logging complexity.

    :meta private:
    """

    _logger = logging.getLogger(__name__)

    @classmethod
    def apply_logging_config(cls, level, log_file_name):
        """Apply basic logging configuration."""
        if level == logging.NOTSET:
            level = cls._logger.getEffectiveLevel()
        if isinstance(level, str):
            level = level.upper()

        if cls._logger.hasHandlers():
            cls._logger.handlers.clear()

        log_formatter = CustomFormatter('%(asctime)s [%(levelname)s] %(calling_file)s:%(calling_line)d, %(color_msg)s', addColor=True)
        log_stream_handler = logging.StreamHandler()
        log_stream_handler.setFormatter(log_formatter)
        cls._logger.addHandler(log_stream_handler)

        if log_file_name:
            if os.path.exists(log_file_name):
                os.remove(log_file_name)
            log_file_formatter = CustomFormatter('%(asctime)s [%(levelname)s] %(calling_file)s:%(calling_line)d, %(message)s', 10)
            log_file_handler = logging.FileHandler(log_file_name)
            log_file_handler.setFormatter(log_file_formatter)
            cls._logger.addHandler(log_file_handler)
        cls.setLevel(level)

    @classmethod
    def setLevel(cls, level):
        """Apply basic logging level."""
        cls._logger.setLevel(level)

    @classmethod
    def build_msg(cls, txt, *args):
        """Build message."""
        string_args = []
        count_args = len(args) - 1
        skip = False
        for i in range(count_args + 1):
            if skip:
                skip = False
                continue
            if (
                i < count_args
                and isinstance(args[i + 1], str)
                and args[i + 1][0] == ":"
            ):
                if args[i + 1] == ":hex":
                    string_args.append(hexlify_packets(args[i]))
                elif args[i + 1] == ":str":
                    string_args.append(str(args[i]))
                elif args[i + 1] == ":b2a":
                    string_args.append(b2a_hex(args[i]))
                skip = True
            else:
                string_args.append(args[i])
        return txt.format(*string_args)

    @classmethod
    def debug(cls, txt, *args):
        """Log debug messages."""
        if cls._logger.isEnabledFor(logging.DEBUG):
            cls._logger.debug(cls.build_msg(txt, *args))

    @classmethod
    def info(cls, txt, *args):
        """Log info messages."""
        if cls._logger.isEnabledFor(logging.INFO):
            cls._logger.info(cls.build_msg(txt, *args))

    @classmethod
    def warning(cls, txt, *args):
        """Log warning messages."""
        if cls._logger.isEnabledFor(logging.WARNING):
            cls._logger.warning(cls.build_msg(txt, *args))

    @classmethod
    def error(cls, txt, *args):
        """Log error messages."""
        if cls._logger.isEnabledFor(logging.ERROR):
            cls._logger.error(cls.build_msg(txt, *args))

    @classmethod
    def critical(cls, txt, *args):
        """Log critical messages."""
        if cls._logger.isEnabledFor(logging.CRITICAL):
            cls._logger.critical(cls.build_msg(txt, *args))

class LCOLORS(object):
    toggle = False
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

colors_map = {
    'DEBUG': LCOLORS.OKCYAN,
    'INFO': LCOLORS.OKBLUE,
    'WARNING': LCOLORS.WARNING,
    'ERROR': LCOLORS.FAIL,
    'CRITICAL': LCOLORS.FAIL
}

def hexlify_packets(packet) -> str:
    if not packet:
        return ""
    return ",".join([hex(int(x)) for x in packet])

def hexlify_packets_without_0x(packet) -> str:
    if not packet:
        return ""
    if isinstance(packet, (list, tuple, bytes)):
        return " ".join(['{:02X}'.format(int(x)) for x in packet])
    if isinstance(packet, int):
        return '{:02X}'.format(packet)
    return ""
