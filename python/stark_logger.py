import datetime
import functools
import inspect
import json
import logging
import os
import sys
import time
import traceback
from copy import copy

import six


class ST(object):
    DEBUG = False
    LOG_DIR = None
    LOG_FILE = f"log_{datetime.date.today().strftime('%y%m%d')}.txt"
    PROJECT_ROOT = os.environ.get("PROJECT_ROOT", "")


class Starkogger(object):
    def __init__(self, logfile):
        super(Starkogger, self).__init__()
        self.running_stack = []
        self.logfile = None
        self.logfd = None
        self.set_logfile(logfile)

    def set_logfile(self, logfile):
        if logfile:
            self.logfile = os.path.realpath(logfile)
            self.logfd = open(self.logfile, "w")
        else:
            self.logfile = None
            if self.logfd:
                self.logfd.close()
                self.logfd = None

    @staticmethod
    def _dumper(obj):
        if hasattr(obj, "to_json"):
            return obj.to_json()
        try:
            d = copy(obj.__dict__)
            try:
                d["__class__"] = obj.__class__.__name__
            except AttributeError:
                pass
            return d
        except AttributeError:
            return repr(obj)

    def log(self, tag, data, depth=None, timestamp=None):
        # TODO Thread safe
        # LOGGING.debug("%s: %s" % (tag, data))
        if depth is None:
            depth = len(self.running_stack)
        if self.logfd:
            timestamp = float(timestamp) if timestamp is not None else time.time()
            try:
                log_data = json.dumps({'tag': tag, 'depth': depth, 'time': timestamp,
                                       'data': data}, default=self._dumper)
            except UnicodeDecodeError:
                # PY2
                log_data = json.dumps({'tag': tag, 'depth': depth, 'time': timestamp,
                                       'data': data}, default=self._dumper, ensure_ascii=False)
            self.logfd.write(log_data + '\n')
            self.logfd.flush()

    def handle_stacked_log(self):
        while self.running_stack:
            log_stacked = self.running_stack[-1]
            self.log("function", log_stacked)
            self.running_stack.pop()


def Logwrap(f, logger):
    @functools.wraps(f)
    def wrapper(*args, **kwargs):
        # py3 only: def wrapper(*args, depth=None, **kwargs):
        depth = kwargs.pop('depth', None)  # For compatibility with py2
        start = time.time()
        m = inspect.getcallargs(f, *args, **kwargs)
        fndata = {'name': f.__name__, 'call_args': m, 'start_time': start}
        logger.running_stack.append(fndata)
        try:
            res = f(*args, **kwargs)
        except Exception as e:
            data = {"traceback": traceback.format_exc(), "end_time": time.time()}
            fndata.update(data)
            raise
        else:
            fndata.update({'ret': res, "end_time": time.time()})
        finally:
            logger.log('function', fndata, depth=depth)
            logger.running_stack.pop()
        return res

    return wrapper


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


class SRLOG(object):
    """Represent the globals variables"""
    BASEDIR = []
    USE_COLOR = True
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s][Zen Python Binding:%(pathname)s:%(lineno)d]>%(message)s", datefmt="%Y-%m-%d %H:%M:%S")
    LOGGER = Starkogger(None)
    LOGGING = logging.getLogger("Stark python binding")
    LOGGING.setLevel(logging.DEBUG)
    # LOGGING.addHandler(logging.StreamHandler(sys.stdout))

    # Create an instance

    COLOR = {logging.DEBUG: LCOLORS.OKCYAN, logging.INFO: LCOLORS.OKBLUE, logging.WARNING: LCOLORS.WARNING, logging.ERROR: LCOLORS.FAIL}

    @classmethod
    def LOG_COLOR(cls, level=logging.INFO, msg=None):
        try:
            if not cls.USE_COLOR:
                raise ValueError
            cls.LOGGING.log(level=level, msg=f'{cls.COLOR[level]} {msg}\033[0m')
        except (ValueError, AttributeError) as e:
            cls.LOGGING.log(level=level, msg=msg)

    @classmethod
    def LOG_DEBUG(cls, msg):
        cls.LOG_COLOR(level=logging.DEBUG, msg=msg)

    @classmethod
    def LOG_INFO(cls, msg):
        cls.LOG_COLOR(level=logging.INFO, msg=msg)

    @classmethod
    def LOG_WARNING(cls, msg):
        cls.LOG_COLOR(level=logging.WARNING, msg=msg)

    @classmethod
    def LOG_ERROR(cls, msg):
        cls.LOG_COLOR(level=logging.ERROR, msg=msg)


def set_logdir(dirpath):
    """set log dir for logfile and screenshots.

    Args:
        dirpath: directory to save logfile and screenshots

    Returns:

    """
    if not os.path.exists(dirpath):
        os.mkdir(dirpath)
    ST.LOG_DIR = dirpath
    SRLOG.LOGGER.set_logfile(os.path.join(ST.LOG_DIR, ST.LOG_FILE))


def log(arg, timestamp=None, desc=""):
    if SRLOG.LOGGER:
        depth = 0
        if isinstance(arg, Exception):
            if hasattr(arg, "__traceback__"):
                # in PY3, arg.__traceback__ is traceback object
                trace_msg = ''.join(traceback.format_exception(type(arg), arg, arg.__traceback__))
            else:
                trace_msg = arg.message  # PY2
            SRLOG.LOGGER.log("info", {
                "name": desc or arg.__class__.__name__,
                "traceback": trace_msg,
            }, depth=depth, timestamp=timestamp)
            SRLOG.LOGGING.error(trace_msg)
        elif isinstance(arg, six.string_types):
            SRLOG.LOGGER.log("info", {"name": desc or arg, "traceback": None, "log": arg}, depth=depth, timestamp=timestamp)
            SRLOG.LOGGING.info(arg)
        else:
            SRLOG.LOGGER.log("info", {"name": desc or repr(arg), "traceback": None, "log": repr(arg)}, depth=depth,
                            timestamp=timestamp)
            SRLOG.LOGGING.info(repr(arg))


def logwrap(f):
    return Logwrap(f, SRLOG.LOGGER)


def using(path):
    if not os.path.isabs(path):
        abspath = os.path.join(ST.PROJECT_ROOT, path)
        if os.path.exists(abspath):
            path = abspath
    SRLOG.LOGGING.debug("using path: %s", path)
    if path not in sys.path:
        sys.path.append(path)
    SRLOG.BASEDIR.append(path)
