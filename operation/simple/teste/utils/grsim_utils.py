import math

class GrSimUtils:
    @staticmethod
    def normalize_angle(angle: float):
        if angle < 0:
            return angle + math.pi
        elif angle > 0:
            return angle - math.pi
        return angle
    
    @staticmethod
    def normalize_position(
        x: float,
        y: float,
        is_left_team: bool):
        
        if is_left_team:
            return x, y 
        
        return -x, -y
    
    @staticmethod
    def normalize_speed(
        x: float,
        y: float,
        isLeftTeam: bool):

        if isLeftTeam:
            return x, y 
        
        return -x, -y