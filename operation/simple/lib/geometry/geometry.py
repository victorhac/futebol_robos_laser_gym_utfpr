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
    def normalizeAngle(
        value: float,
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
    def distance(position1: 'tuple[float, float]', position2: 'tuple[float, float]'):
        
        deltx = position1[0] - position2[0]
        delty = position1[1] - position2[1]
        
        deltx = deltx * deltx
        delty = delty * delty

        dist = math.sqrt((deltx + delty))
        
        return dist

    @staticmethod
    def isClose(
        position1: tuple[float, float], 
        position2: tuple[float, float],
        tolerance: float):

        return Geometry.distance(position1, position2) < tolerance
    
    @staticmethod
    def findIntersection(line1: tuple[float, float, float], line2: tuple[float, float, float]):
        """
        Extract coefficients (a, b, c) from line equations (ax + by = c)
        """
        a1, b1, c1 = line1
        a2, b2, c2 = line2

        determinant = a1 * b2 - a2 * b1

        if determinant == 0:
            return None

        x = (c1 * b2 - c2 * b1) / determinant
        y = (a1 * c2 - a2 * c1) / determinant

        return x, y
    
    def lineEquation(point1: tuple[float, float], point2: tuple[float, float]):
        """
        Return the equation of the line passing through two points in the form (a, b, c): (ax + by = c).
        """
        x1, y1 = point1
        x2, y2 = point2
        
        if x2 - x1 != 0:
            m = (y2 - y1) / (x2 - x1)
        else:
            return None
        
        b = y1 - m * x1

        return -m, 1, b
