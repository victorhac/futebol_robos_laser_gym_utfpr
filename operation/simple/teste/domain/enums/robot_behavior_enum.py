from enum import Enum

class RobotBehaviorEnum(Enum):
    NONE = 0,
    STOPPED = 1,
    MANUAL = 2,
    BALL_TRACKER = 3,
    POSITIONING = 4