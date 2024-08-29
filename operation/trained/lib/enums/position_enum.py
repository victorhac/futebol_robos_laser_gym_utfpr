from enum import Enum

class PositionEnum(Enum):
    NONE = 0
    OWN_AREA = 1
    GOAL_AREA = 2
    OWN_AREA_EXCEPT_GOAL_AREA = 3
    OPPONENT_AREA = 4
    OPPONENT_GOAL_AREA = 5
    OPPONENT_AREA_EXCEPT_GOAL_AREA = 6
    RELATIVE_TO_BALL = 7