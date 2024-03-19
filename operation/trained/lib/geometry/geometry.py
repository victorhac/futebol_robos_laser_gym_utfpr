import math
from shapely.geometry import Polygon

from ..domain.rectangle import Rectangle

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
    def distance(position1: tuple[float, float], position2: tuple[float, float]):
        position1x, position1y = position1
        position2x, position2y = position2

        return math.sqrt((position2x - position1x) ** 2 + (position2y - position1y) ** 2)

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
    def getMidpoint(point1: tuple[float, float], point2: tuple[float, float]):
        """
        Return the midpoint of two points.
        """
        x1, y1 = point1
        x2, y2 = point2

        return (x1 + x2) / 2, (y1 + y2) / 2
    
    @staticmethod
    def getRotatedRectangleVertices(rectangle: Rectangle):
        """
        Calculate the vertices of a rotated rectangle.

        Args:
        - center: Center coordinates of the rectangle (x, y).
        - width: Width of the rectangle.
        - height: Height of the rectangle.
        - angle: Rotation angle in radians.

        Returns:
        - List of (x, y) coordinates representing the vertices of the rotated rectangle.
        """
        half_width = rectangle.width / 2
        half_height = rectangle.height / 2

        # Calculate the coordinates of the vertices in the local (unrotated) coordinate system
        vertices_local = [(-half_width, -half_height), (half_width, -half_height),
                        (half_width, half_height), (-half_width, half_height)]

        # Rotate each vertex around the center by the specified angle
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
        rectangle1Vertices = Geometry.getRotatedRectangleVertices(rectangle1)
        rectangle2Vertices = Geometry.getRotatedRectangleVertices(rectangle2)

        poly1 = Polygon(rectangle1Vertices)
        poly2 = Polygon(rectangle2Vertices)

        return poly1.intersects(poly2)
    
    @staticmethod
    def contains(
        rectangle1: Rectangle,
        rectangle2: Rectangle
    ):
        rectangle1Vertices = Geometry.getRotatedRectangleVertices(rectangle1)
        rectangle2Vertices = Geometry.getRotatedRectangleVertices(rectangle2)

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
    def truncate(value: float, minValue: float = None, maxValue: float = None) -> float:
        if minValue is None and maxValue is None:
            return value
        elif minValue is None:
            return min(value, maxValue)
        elif maxValue is None:
            return max(value, minValue)
        
        if value > maxValue:
            return maxValue
        elif value < minValue:
            return minValue
        else:
            return value