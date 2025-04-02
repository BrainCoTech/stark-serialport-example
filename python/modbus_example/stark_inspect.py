from utils import inspect_class
from stark_utils import libstark, logger

inspect_class(libstark.DeviceInfo)
inspect_class(libstark.ForceLevel)

logger.info(libstark.ForceLevel(1))
logger.info(libstark.ForceLevel.Small.int_value)

inspect_class(libstark.DfuState)
