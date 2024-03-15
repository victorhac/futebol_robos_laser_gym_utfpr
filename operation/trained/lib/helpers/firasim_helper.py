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
    def normalizePosition(position: float):
        return -position