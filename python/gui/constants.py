"""Shared constants for GUI panels"""

import sys
from pathlib import Path

# Add parent directory to path for SDK import
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import sdk


# =============================================================================
# Motor Constants (6 DOF)
# =============================================================================

# Map finger index to FingerId enum
FINGER_IDS = [
    sdk.FingerId.Thumb,
    sdk.FingerId.ThumbAux,
    sdk.FingerId.Index,
    sdk.FingerId.Middle,
    sdk.FingerId.Ring,
    sdk.FingerId.Pinky,
] if sdk else []

MOTOR_NAMES_EN = ['Thumb', 'ThumbAux', 'Index', 'Middle', 'Ring', 'Pinky']
MOTOR_NAMES_ZH = ['拇指', '拇指辅助', '食指', '中指', '无名指', '小指']

MOTOR_COLORS = [
    (255, 107, 107),  # Thumb - Red
    (255, 182, 193),  # ThumbAux - Light Pink
    (78, 205, 196),   # Index - Teal
    (69, 183, 209),   # Middle - Blue
    (255, 160, 122),  # Ring - Orange
    (152, 216, 200),  # Pinky - Green
]

MOTOR_COUNT = 6


# =============================================================================
# Touch Sensor Constants (5 fingers, no ThumbAux)
# =============================================================================

TOUCH_NAMES_EN = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
TOUCH_NAMES_ZH = ['拇指', '食指', '中指', '无名指', '小指']

TOUCH_COLORS = [
    (255, 107, 107),  # Thumb - Red
    (78, 205, 196),   # Index - Teal
    (69, 183, 209),   # Middle - Blue
    (255, 160, 122),  # Ring - Orange
    (152, 216, 200),  # Pinky - Green
]

TOUCH_COUNT = 5

# Touch sensor configuration per finger (Revo1AdvancedTouch)
# Each finger has different number of force sensors
# Format: (force_groups, self_proximity_groups, mutual_proximity_groups)
TOUCH_SENSOR_CONFIG = {
    'Thumb':  (2, 1, 0),  # 2 force groups, 1 self-proximity, 0 mutual
    'Index':  (3, 2, 1),  # 3 force groups, 2 self-proximity, 1 mutual
    'Middle': (3, 2, 1),  # 3 force groups
    'Ring':   (3, 2, 1),  # 3 force groups
    'Pinky':  (2, 1, 0),  # 2 force groups, 1 self-proximity, NO mutual
}

# Default force sensor index to display per finger
# Use the first sensor (index 0) for all fingers by default
TOUCH_DEFAULT_FORCE_INDEX = [0, 0, 0, 0, 0]  # force1 for all


# =============================================================================
# Buffer Sizes
# =============================================================================

MOTOR_BUFFER_SIZE = 1000
TOUCH_BUFFER_SIZE = 500


# =============================================================================
# Utility Functions
# =============================================================================

def format_motor_state(state) -> str:
    """Format motor state for display"""
    state_name = state.name if hasattr(state, 'name') else str(state)
    return state_name.replace('MotorState.', '')


def get_motor_names(language: str = 'en') -> list:
    """Get motor names in specified language"""
    return MOTOR_NAMES_ZH if language == 'zh' else MOTOR_NAMES_EN


def get_touch_names(language: str = 'en') -> list:
    """Get touch sensor names in specified language"""
    return TOUCH_NAMES_ZH if language == 'zh' else TOUCH_NAMES_EN


def get_force_count(finger_name: str) -> int:
    """Get number of force sensors for a finger"""
    config = TOUCH_SENSOR_CONFIG.get(finger_name, (1, 0, 0))
    return config[0]


def get_touch_force_value(touch_item, force_index: int = 0, force_type: str = 'normal'):
    """Get force value from TouchFingerItem
    
    Args:
        touch_item: TouchFingerItem object
        force_index: 0, 1, or 2 (which sensor)
        force_type: 'normal' or 'tangential'
    
    Returns:
        Force value (int)
    """
    if not touch_item:
        return 0
    
    attr_name = f"{force_type}_force{force_index + 1}"
    return getattr(touch_item, attr_name, 0)
