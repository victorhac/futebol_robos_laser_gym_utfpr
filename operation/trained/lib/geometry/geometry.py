import math

class Geometry:
    @staticmethod
    def smallestAngleDiff(angle1: float, angle2: float):
        PI = math.pi
        angle = (angle2 - angle1) % (2 * PI)
        if angle >= PI:
            return angle - (2 * PI)
        elif angle < -PI:
            return angle + (2 * PI)
        return angle

    @staticmethod
    def normalizeAngle(value: float,
                    center: float,
                    amplitude: float):
        value = value % (2 * amplitude)
        if value < -amplitude + center:
            value += 2 * amplitude
        elif value > amplitude + center:
            value -= 2 * amplitude
        return value

    @staticmethod
    def normalizeInPI(radians: float):
        PI = math.pi
        return Geometry.normalizeAngle(radians, 0, PI)

    @staticmethod
    def distance(position1: tuple[float, float],
                position2: tuple[float, float]):
        position1x, position1y = position1
        position2x, position2y = position2

        return math.sqrt((position2x - position1x) ** 2 + (position2y - position1y) ** 2)

    @staticmethod
    def isClose(position1: tuple[float, float], 
            position2: tuple[float, float],
            tolerance: float):
        return Geometry.distance(position1, position2) < tolerance