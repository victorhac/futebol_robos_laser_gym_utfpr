import random
import math

from ..geometry.geometry_utils import GeometryUtils

class FieldHelper:
    @staticmethod
    def isLeftTeam(isYellowTeam: bool, isYellowLeftTeam: bool):
        return (isYellowLeftTeam and isYellowTeam) or (not isYellowLeftTeam and not isYellowTeam)
    
    @staticmethod
    def isInsideField(
        x: float, y: float,
        fieldWidth: float,
        fieldHeight: float
    ):
        
        return abs(x) < fieldWidth / 2 and abs(y) < fieldHeight / 2
    
    @staticmethod
    def getOpponentGoalPosition(
        fieldLength: float,
        isLeftTeam: bool
    ):
        if isLeftTeam:
            return (fieldLength / 2, 0)
        else:
            return (-fieldLength / 2, 0)
    
    @staticmethod
    def getOwnGoalPosition(
        fieldLength: float,
        isLeftTeam: bool
    ):
        if isLeftTeam:
            return (-fieldLength / 2, 0)
        else:
            return (fieldLength / 2, 0)
        
    @staticmethod
    def getOwnPenaltyPosition(
        fieldLength: float,
        penaltyLength: float,
        isLeftTeam: bool
    ):
        if isLeftTeam:
            return (-fieldLength / 2 + penaltyLength, 0)
        else:
            return (fieldLength / 2 - penaltyLength, 0)
        
    # deprecated
    @staticmethod
    def getFieldRandomPosition(fieldLength: float, fieldWidth: float):
        return random.uniform(-fieldLength / 2, fieldLength / 2), random.uniform(-fieldWidth / 2, fieldWidth / 2)
    
    @staticmethod
    def getRandomTheta():
        return random.uniform(-math.pi, math.pi)
    
    @staticmethod
    def getMaxDistanceToPoint(
        position: tuple[float, float],
        fieldLength: float,
        fieldWidth: float
    ):
        distances = [
            GeometryUtils.distance(position, (fieldLength / 2, fieldWidth / 2)),
            GeometryUtils.distance(position, (fieldLength / 2, -fieldWidth / 2)),
            GeometryUtils.distance(position, (-fieldLength / 2, fieldWidth / 2)),
            GeometryUtils.distance(position, (-fieldLength / 2, -fieldWidth / 2))
        ]
        
        return max(distances)
    
    @staticmethod
    def get_biggest_distance(
        field_length: float,
        field_width: float,
    ):
        return math.sqrt(field_length ** 2 + field_width ** 2) / 2
    
    @staticmethod
    def is_inside_own_goal_area(
        position: tuple[float, float],
        field_length: float,
        penalty_length: float,
        penalty_width: float,
        is_left_team: bool
    ):
        x, y = position

        if is_left_team:
            return x < -field_length / 2 + penalty_length and abs(y) < penalty_width / 2
        else:
            return x > field_length / 2 - penalty_length and abs(y) < penalty_width / 2
        
    @staticmethod
    def is_inside_opponent_area(
        position: tuple[float, float],
        is_left_team: bool
    ):
        x, _ = position

        if is_left_team:
            return x > 0
        else:
            return x < 0
        
    @staticmethod
    def is_touching(
        position1: tuple[float, float],
        radius1: float,
        position2: tuple[float, float],
        radius2: float,
        threshold: float
    ):
        return GeometryUtils.circunferences_intersect(
            position1,
            radius1 + threshold,
            position2,
            radius2 + threshold
        )
    
    @staticmethod
    def get_random_position_inside_field(
        field_length: float,
        field_width: float
    ):
        max_x = field_length / 2
        max_y = field_width / 2
        margin = 0.15

        return \
            random.uniform(-max_x + margin, max_x - margin), \
            random.uniform(-max_y + margin, max_y - margin)
    
    @staticmethod
    def get_random_position_inside_own_area(
        field_length: float,
        field_width: float,
        is_left_team: bool,
    ):
        max_x = field_length / 2
        max_y = field_width / 2
        margin = 0.15

        y = random.uniform(-max_y + margin, max_y - margin)

        if is_left_team:
            return random.uniform(-max_x + margin, 0), y
        else:
            return random.uniform(0, max_x - margin), y
        
    @staticmethod
    def get_random_position_inside_opponent_area(
        field_length: float,
        field_width: float,
        is_left_team: bool,
    ):
        return FieldHelper.get_random_position_inside_own_area(
            field_length,
            field_width,
            not is_left_team
        )