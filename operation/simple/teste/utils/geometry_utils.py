import math
import numpy as np

class GeometryUtils:
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
    def between(value, lower, upper):
        return lower <= value <= upper
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
        return GeometryUtils.normalizeAngle(radians, 0, PI)

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
        position1: 'tuple[float, float]', 
        position2: 'tuple[float, float]',
        tolerance: float):

        return GeometryUtils.distance(position1, position2) < tolerance
    
    @staticmethod
    def findIntersection(line1: 'tuple[float, float, float]', line2: 'tuple[float, float, float]'):
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
            print("morri legal")
            return None
        
        c = y1 - m * x1

        return (-m, 1, c)
    
    def directionalVector(point: 'tuple[float, float]', other: 'tuple[float, float]') :
            A = point
            B = other

            vetor_direcao = (B[0] - A[0], B[1] - A[1])

            norma = math.sqrt(vetor_direcao[0]**2 + vetor_direcao[1]**2)

            if norma != 0:
                vetor_unitario = (vetor_direcao[0] / norma, vetor_direcao[1] / norma)
            else:
                vetor_unitario = (0, 0)  

            return vetor_unitario[0] * 2, vetor_unitario[1] * 2
    
    def PointOnDirection(position: 'tuple[float, float]', direction: 'tuple[float, float]', raio):
        
        C = (position[0] + raio * direction[0], position[1] + raio* direction[1])
        return C
    
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

   



