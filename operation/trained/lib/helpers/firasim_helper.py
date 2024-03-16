import math

class FIRASimHelper:
    @staticmethod
    def normalizeAngle(angle: float):
        if angle < 0:
            return angle + math.pi
        elif angle > 0:
            return angle - math.pi
        return angle
    
    @staticmethod
    def normalizePosition(
        x: float,
        y: float,
        isLeftTeam: bool):
        
        if isLeftTeam:
            return x, y 
        
        return -x, -y
    
    @staticmethod
    def normalizeSpeed(
        x: float,
        y: float,
        isLeftTeam: bool):

        if isLeftTeam:
            return x, y 
        
        return -x, -y