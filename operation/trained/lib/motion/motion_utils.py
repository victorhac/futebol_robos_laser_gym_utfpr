from ..utils.field_utils import FieldUtils
from ..domain.field_data import FieldData
from ..domain.rectangle import Rectangle
from ..domain.robot import Robot
from ..geometry.geometry_utils import GeometryUtils
from ..utils.configuration_utils import ConfigurationUtils
from ..utils.robot_utils import RobotUtils

import math

ROBOT_LENGTH = ConfigurationUtils.get_firasim_robot_length()
ROBOT_WIDTH = ConfigurationUtils.get_firasim_robot_width()

FIELD_WIDTH = ConfigurationUtils.get_field_width()
FIELD_LENGTH = ConfigurationUtils.get_field_length()

MOTION_COLLISION_AVOIDANCE_MIN_DISTANCE = ConfigurationUtils.get_motion_collision_avoidance_min_distance()

KP = ConfigurationUtils.get_motion_pid_constants_kp()
KD = ConfigurationUtils.get_motion_pid_constants_kd()

class MotionUtils:
    @staticmethod
    def go_to_point(
        robot: Robot, 
        target_position: tuple[float, float],
        last_error: float = 0,
        base_speed: float = 30
    ):
        x, y = robot.get_position_tuple()
        robot_angle = robot.position.theta

        x_target, y_target = (target_position[0], target_position[1])

        angle_to_target = math.atan2(y_target - y, x_target - x)

        error = GeometryUtils.smallest_angle_difference(angle_to_target, robot_angle)

        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
            reversed = True
            robot_angle = GeometryUtils.normalize_in_PI(robot_angle + math.pi)
            error = GeometryUtils.smallest_angle_difference(angle_to_target, robot_angle)
        else:
            reversed = False

        motorSpeed = (KP * error) + (KD * (error - last_error))

        motorSpeed = RobotUtils.truncateMotorSpeed(motorSpeed, base_speed)

        leftMotorSpeed, rightMotorSpeed = MotionUtils._get_speeds(motorSpeed, base_speed, reversed)

        return leftMotorSpeed, rightMotorSpeed, error
    
    @staticmethod
    def _get_speeds(motor_speed: float, base_speed: float, reversed: bool):
        if reversed:
            if motor_speed > 0:
                left_motor_speed = -base_speed + motor_speed
                right_motor_speed = -base_speed
            else:
                left_motor_speed = -base_speed
                right_motor_speed = -base_speed - motor_speed
        else:
            if motor_speed > 0:
                left_motor_speed = base_speed
                right_motor_speed = base_speed - motor_speed
            else:
                left_motor_speed = base_speed + motor_speed
                right_motor_speed = base_speed

        return left_motor_speed, right_motor_speed
    
    @staticmethod
    def spin(clockwise: bool, spinPower: float):
        if clockwise:
            return spinPower, -spinPower
        
        return -spinPower, spinPower
    
    @staticmethod
    def find_obstacles(
        robotId: int,
        fieldData: FieldData,
        opponentFieldData: FieldData,
        targetPosition: tuple[float, float]
    ) -> list[Robot] | None:
        robot = fieldData.robots[robotId]
        obstacles = []

        center = GeometryUtils.getMidpoint((robot.position.x, robot.position.y), targetPosition)
        width = GeometryUtils.distance((robot.position.x, robot.position.y), targetPosition)
        height = ROBOT_LENGTH
        angle = math.atan2(targetPosition[1] - robot.position.y, targetPosition[0] - robot.position.x)

        rectangle = Rectangle(center, width, height, angle)

        for i in range(len(opponentFieldData.robots)):
            otherRobot = opponentFieldData.robots[i]

            otherRobotRectangle = RobotUtils.getRectangle(otherRobot, ROBOT_WIDTH, ROBOT_LENGTH)

            if GeometryUtils.hasIntersection(rectangle, otherRobotRectangle):
                obstacles.append(otherRobot)

        for i in range(len(fieldData.robots)):
            if i == robotId:
                continue

            otherRobot = fieldData.robots[i]

            otherRobotRectangle = RobotUtils.getRectangle(otherRobot, ROBOT_WIDTH, ROBOT_LENGTH)

            if GeometryUtils.hasIntersection(rectangle, otherRobotRectangle):
                obstacles.append(otherRobot)

        return obstacles

    @staticmethod
    def find_closest_obstacle(
        robotId: int,
        fieldData: FieldData,
        opponentFieldData: FieldData,
        targetPosition: tuple[float, float]
    ) -> Robot | None:
        obstacles = MotionUtils.find_obstacles(robotId, fieldData, opponentFieldData, targetPosition)

        if len(obstacles) == 0:
            return None

        robot = fieldData.robots[robotId]
        closestObstacle = obstacles[0]

        minDistance = GeometryUtils.distance(
            (robot.position.x, robot.position.y),
            (closestObstacle.position.x, closestObstacle.position.y))

        for obstacle in obstacles:
            distance = GeometryUtils.distance(
                (robot.position.x, robot.position.y),
                (obstacle.position.x, obstacle.position.y))

            if distance < minDistance:
                closestObstacle = obstacle
                minDistance = distance

        return closestObstacle

    @staticmethod
    def find_tangent_point_obstacle(
        robotId: int,
        fieldData: FieldData,
        opponentFieldData: FieldData,
        targetPosition: tuple[float, float]
    ):
        obstacleRobot = MotionUtils.find_closest_obstacle(
            robotId,
            fieldData,
            opponentFieldData,
            targetPosition)

        if obstacleRobot is None:
            return None

        robot = fieldData.robots[robotId]
        center = (obstacleRobot.position.x, obstacleRobot.position.y)
        point = (robot.position.x, robot.position.y)

        distance = GeometryUtils.distance(center, point)

        circleRadius = MOTION_COLLISION_AVOIDANCE_MIN_DISTANCE

        if not distance > circleRadius:
            circleRadius = distance - 0.01

        tangentPoints = GeometryUtils.getTangentPoints(center, circleRadius, point)

        if tangentPoints is None:
            return None

        rectangleField = Rectangle((0,0), FIELD_WIDTH, FIELD_LENGTH, 0)

        angleToTangent1 = math.atan2(tangentPoints[0][1] - robot.position.y, tangentPoints[0][0] - robot.position.x)
        rectangleTangent1 = Rectangle(tangentPoints[0], ROBOT_LENGTH, ROBOT_WIDTH, angleToTangent1)

        angleToTangent2 = math.atan2(tangentPoints[1][1] - robot.position.y, tangentPoints[1][0] - robot.position.x)
        rectangleTangent2 = Rectangle(tangentPoints[1], ROBOT_LENGTH, ROBOT_WIDTH, angleToTangent2)

        isTangent1InsideField = GeometryUtils.contains(rectangleField, rectangleTangent1)
        isTangent2InsideField = GeometryUtils.contains(rectangleField, rectangleTangent2)

        if isTangent1InsideField and isTangent2InsideField:
            # TODO: verificar quais pontos são válidos e qual é o melhor pelo número de obstáculos 
            # a partir de cada ponto ou pela distância até o objetivo
            return tangentPoints[1]
        elif isTangent1InsideField:
            return tangentPoints[0]
        elif isTangent2InsideField:
            return tangentPoints[1]
        
        return tangentPoints[1]