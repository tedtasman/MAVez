from enum import Enum

class RepositionYawMode(Enum):
    """Enum for MAVLink reposition yaw modes.
    """
    CLOCKWISE = 0
    COUNTER_CLOCKWISE = 1
    USE_YAW = 2
    NONE = 3


