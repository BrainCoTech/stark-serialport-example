import sys
import os

# Add parent directory to path to import logger and common utilities
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import logging
from logger import getLogger
from common_utils import setup_shutdown_event

# logger = getLogger(logging.DEBUG)
logger = getLogger(logging.INFO)

from bc_stark_sdk import main_mod

libstark = main_mod
libstark.init_config(libstark.StarkProtocolType.EtherCAT)
