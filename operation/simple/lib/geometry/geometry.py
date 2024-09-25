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

        return math.sqrt((deltx + delty))
        

    @staticmethod
    def isClose(
        position1: 'tuple[float, float]', 
        position2: 'tuple[float, float]',
        tolerance: float):
        return Geometry.distance(position1, position2) < tolerance
    
    @staticmethod
    def findIntersection(line1: 'tuple[float, float, float]', line2: 'tuple[float, float, float]'):

        a1, b1, c1 = line1
        a2, b2, c2 = line2

        determinant = a1 * b2 - a2 * b1

        if determinant == 0:
            return None

        x = (c1 * b2 - c2 * b1) / determinant
        y = (a1 * c2 - a2 * c1) / determinant

        return (x, y)

    
    @staticmethod
    def lineEquation(point1: 'tuple[float, float]', point2: 'tuple[float, float]'):
        """
        Return the equation of the line passing through two points in the form (a, b, c): (ax + by = c).
        """
        x1, y1 = point1
        x2, y2 = point2
        
        if x2 - x1 != 0:
            m = (y2 - y1) / (x2 - x1)
        else:
            return None
        
        c = y1 - m * x1

        return (-m, 1, c)
    
    def directionalVector(point: 'tuple[float, float]', other: 'tuple[float, float]') :
            A = point
            B = other

            dirVector = (B[0] - A[0], B[1] - A[1])

            norm = math.sqrt(dirVector[0]**2 + dirVector[1]**2)

            if norm != 0:
                unitaryVector = (dirVector[0] / norm, dirVector[1] / norm)
            else:
                unitaryVector = (0, 0)  

            return tuple(unitaryVector)
        
    def pointOnDirection(position: 'tuple[float, float]', direction: 'tuple[float, float]', radius):
        
        return (position[0] + radius * direction[0], position[1] + radius* direction[1])
    
    def sumVector(dirVet: 'tuple[float, float]', other: 'tuple[float, float]') -> tuple:
        result = (dirVet[0] + other[0], dirVet[1] + other[1])
        
        norm = math.sqrt(result[0]**2 + result[1]**2)
        if norm != 0:
            normalized_result = (result[0] / norm, result[1] / norm)
        else:
            normalized_result = (0, 0)
        return normalized_result
    
    def perpendicularVector(directionalVector: 'tuple[float, float]'):
        return (directionalVector[1], -directionalVector[0])
    