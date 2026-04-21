from enum import Enum

class PrecisionLandMode(Enum):
    """Enum for MAVLink precision land modes.
    """
    DISABLED = 0
    OPPORTUNISTIC = 1
    REQUIRED = 2

