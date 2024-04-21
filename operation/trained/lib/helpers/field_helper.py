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