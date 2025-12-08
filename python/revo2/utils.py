"""Revo2 utilities - imports common utilities from parent directory"""

import sys
import os

# Add parent directory to path to import common utilities
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_utils import setup_shutdown_event

# Re-export for backward compatibility
__all__ = ['setup_shutdown_event']

