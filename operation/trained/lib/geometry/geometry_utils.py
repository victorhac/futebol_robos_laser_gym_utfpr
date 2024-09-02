import math
from shapely.geometry import Polygon
import random

from ..domain.rectangle import Rectangle

class GeometryUtils:
    @staticmethod
    def smallest_angle_diff(angle1: float, angle2: float):
        PI = math.pi
        angle = (angle2 - angle1) % (2 * PI)
        if angle >= PI:
            return angle - (2 * PI)
        elif angle < -PI:
            return angle + (2 * PI)
        return angle

    @staticmethod
    def normalize_angle(
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
    def normalize_in_PI(radians: float):
        return GeometryUtils.normalize_angle(radians, 0, math.pi)

    @staticmethod
    def distance(position1: tuple[float, float], position2: tuple[float, float]):
        position1x, position1y = position1
        position2x, position2y = position2

        return math.sqrt((position2x - position1x) ** 2 + (position2y - position1y) ** 2)

    @staticmethod
    def is_close(
        position1: tuple[float, float], 
        position2: tuple[float, float],
        tolerance: float):

        return GeometryUtils.distance(position1, position2) < tolerance
    
    @staticmethod
    def circunferences_intersect(
        center1: tuple[float, float],
        radius1: float,
        center2: tuple[float, float],
        radius2: float
    ):
        distance = GeometryUtils.distance(center1, center2)
        return distance <= radius1 + radius2 and distance >= abs(radius1 - radius2)
    
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
    
    @staticmethod
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
    
    @staticmethod
    def point_to_line_distance(
        point: tuple[float, float],
        line_equation: tuple[float, float, float]
    ):
        px, py = point
        a, b, c = line_equation
        return abs(a * px + b * py - c) / math.sqrt(a ** 2 + b ** 2)

    @staticmethod
    def is_between(
        point: tuple[float, float],
        endpoint1: tuple[float, float],
        endpoint2: tuple[float, float]
    ):
        px, py = point
        x1, y1 = endpoint1
        x2, y2 = endpoint2
    
        cross_product = (py - y1) * (x2 - x1) - (px - x1) * (y2 - y1)
        if abs(cross_product) > 1e-10:
            return False
        dot_product = (px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)
        if dot_product < 0:
            return False
        squared_length = (x2 - x1) ** 2 + (y2 - y1) ** 2
        if dot_product > squared_length:
            return False
        return True

    @staticmethod
    def point_to_point_distance(
        point1: tuple[float, float],
        point2: tuple[float, float]
    ):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @staticmethod
    def distance_point_to_line_segment(
        point: tuple[float, float],
        endpoint1: tuple[float, float],
        endpoint2: tuple[float, float]
    ):
        x1, y1 = endpoint1
        x2, y2 = endpoint2
        
        a = y2 - y1
        b = x1 - x2
        c = a * x1 + b * y1

        line_equation = (a, b, c)

        perp_distance = GeometryUtils.point_to_line_distance(point, line_equation)

        if GeometryUtils.is_between(point, endpoint1, endpoint2):
            return perp_distance
        else:
            dist_to_end1 = GeometryUtils.point_to_point_distance(point, endpoint1)
            dist_to_end2 = GeometryUtils.point_to_point_distance(point, endpoint2)
            return min(dist_to_end1, dist_to_end2)
        
    @staticmethod
    def find_y(
        line_equation: tuple[float, float, float],
        x: float
    ):
        a, b, c = line_equation

        if b == 0:
            None

        return (c - a * x) / b
    
    @staticmethod
    def lineEquationByPointAndAngle(
        point: tuple[float, float],
        angle: float
    ):
        x, y = point
        m = math.tan(angle)
        b = y - m * x

        return -m, 1, b
    
    @staticmethod
    def getMidpoint(point1: tuple[float, float], point2: tuple[float, float]):
        x1, y1 = point1
        x2, y2 = point2

        return (x1 + x2) / 2, (y1 + y2) / 2
    
    @staticmethod
    def getRotatedRectangleVertices(rectangle: Rectangle):
        half_width = rectangle.width / 2
        half_height = rectangle.height / 2

        vertices_local = [(-half_width, -half_height), (half_width, -half_height),
                        (half_width, half_height), (-half_width, half_height)]

        vertices_rotated = []
        for vertex in vertices_local:
            x_local, y_local = vertex
            x_rotated = rectangle.center[0] + x_local * math.cos(rectangle.angle) - y_local * math.sin(rectangle.angle)
            y_rotated = rectangle.center[1] + x_local * math.sin(rectangle.angle) + y_local * math.cos(rectangle.angle)
            vertices_rotated.append((x_rotated, y_rotated))

        return vertices_rotated

    @staticmethod
    def hasIntersection(
        rectangle1: Rectangle,
        rectangle2: Rectangle
    ):
        rectangle1Vertices = GeometryUtils.getRotatedRectangleVertices(rectangle1)
        rectangle2Vertices = GeometryUtils.getRotatedRectangleVertices(rectangle2)

        poly1 = Polygon(rectangle1Vertices)
        poly2 = Polygon(rectangle2Vertices)

        return poly1.intersects(poly2)
    
    @staticmethod
    def contains(
        rectangle1: Rectangle,
        rectangle2: Rectangle
    ):
        rectangle1Vertices = GeometryUtils.getRotatedRectangleVertices(rectangle1)
        rectangle2Vertices = GeometryUtils.getRotatedRectangleVertices(rectangle2)

        poly1 = Polygon(rectangle1Vertices)
        poly2 = Polygon(rectangle2Vertices)

        return poly1.contains(poly2)
    
    @staticmethod
    def getTangentPoints(center: tuple[float, float], radius: float, point: tuple[float, float]):
        """
        Calculate the tangent points on a circle to a given point.

        Args:
        - center: Tuple containing the (x, y) coordinates of the center of the circle.
        - radius: Radius of the circle.
        - point: Tuple containing the (x, y) coordinates of the given point.

        Returns:
        - Tuple containing the tangent points on the circle (tangent1, tangent2),
        where each tangent point is represented as a tuple (x, y).
        """
        cx, cy = center
        px, py = point
        
        distance = math.sqrt((px - cx)**2 + (py - cy)**2)
        
        if not distance > radius:
            return None
        
        angle = math.atan2(py - cy, px - cx)
        
        tangentAngle = math.acos(radius / distance)
        
        tangent1 = (cx + radius * math.cos(angle + tangentAngle),
                    cy + radius * math.sin(angle + tangentAngle))
        tangent2 = (cx + radius * math.cos(angle - tangentAngle),
                    cy + radius * math.sin(angle - tangentAngle))
        
        return tangent1, tangent2
    
    @staticmethod
    def calculateVectorCoordinates(
        magnitude: float,
        angle: float,
        x: float,
        y: float
    ):
        # verificar para o time da direita
        delta_x = magnitude * math.cos(angle)
        delta_y = magnitude * math.sin(angle)
        
        new_x = x + delta_x
        new_y = y + delta_y
        
        return new_x, new_y
    
    @staticmethod
    def angleBetweenVectors(
        v1: list[float],
        v2: list[float]
    ):
        dot_prod = GeometryUtils.dotProduct(v1, v2)

        mag_v1 = GeometryUtils.vectorMagnitude(v1)
        mag_v2 = GeometryUtils.vectorMagnitude(v2)

        cosine_angle = dot_prod / (mag_v1 * mag_v2)

        return math.acos(cosine_angle)
    
    @staticmethod
    def dotProduct(
        v1: list[float],
        v2: list[float]
    ):
        return sum((a * b) for a, b in zip(v1, v2))

    @staticmethod
    def vectorMagnitude(v: list[float]):
        return math.sqrt(sum(a**2 for a in v))
    
    @staticmethod
    def isInsideCircle(
        x: float,
        y: float,
        circle_x: float,
        circle_y: float,
        radius: float
    ):
        distance = math.sqrt((x - circle_x)**2 + (y - circle_y)**2)
        return distance <= radius
    
    @staticmethod
    def getRandomUniform(minValue: float, maxValue: float):
        return random.uniform(minValue, maxValue)