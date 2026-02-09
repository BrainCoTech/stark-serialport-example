import sys
import os

# Import from common_imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_imports import logger, libstark
from common_utils import (
    setup_shutdown_event,
    format_direction,
    print_finger_touch_data_revo1,
    print_finger_touch_data_revo2,
)
