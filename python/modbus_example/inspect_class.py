from utils import inspect_class
from revo2_utils import libstark, logger

inspect_class(libstark.DeviceInfo)
logger.info(libstark.DfuState(1))
inspect_class(libstark.DfuState)
