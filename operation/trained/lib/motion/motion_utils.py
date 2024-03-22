from ..helpers.field_helper import FieldHelper
from ..domain.field_data import FieldData
from ..domain.rectangle import Rectangle
from ..domain.robot import Robot
from ..geometry.geometry_utils import GeometryUtils
from ..helpers.configuration_helper import ConfigurationHelper
from ..helpers.robot_helper import RobotHelper

import math

CONFIGURATION = ConfigurationHelper.getConfiguration()

ROBOT_LENGTH = CONFIGURATION["robot"]["length"]
ROBOT_WIDTH = CONFIGURATION["robot"]["width"]

FIELD_WIDTH = CONFIGURATION["field"]["width"]
FIELD_LENGTH = CONFIGURATION["field"]["length"]

MOTION_COLLISION_AVOIDANCE_MIN_DISTANCE = CONFIGURATION["motion"]["collision-avoidance"]["min-distance"]

class MotionUtils:
    @staticmethod
    def goToPoint(
        robot: Robot, 
        targetPosition: tuple[float, float],
        lastError: float = 0
    ):
        configuration = ConfigurationHelper.getConfiguration()

        position = robot.position

        positionX = position.x
        positionY = position.y
        robotAngle = position.theta

        xTarget, yTarget = (targetPosition[0], targetPosition[1])

        angleToTarget = math.atan2(yTarget - positionY, xTarget - positionX)

        error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)

        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
            reversed = True
            robotAngle = GeometryUtils.normalizeInPI(robotAngle + math.pi)
            error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)
        else:
            reversed = False

        kP = configuration["motion"]["pid"]["constants"]["Kp"]
        kD = configuration["motion"]["pid"]["constants"]["Kd"]

        motorSpeed = (kP * error) + (kD * (error - lastError))

        baseSpeed = configuration["robot"]["speed"]["base"]

        motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

        leftMotorSpeed, rightMotorSpeed = Motion._getSpeeds(motorSpeed, baseSpeed, reversed)

        return leftMotorSpeed, rightMotorSpeed, error
    
    @staticmethod
    def _getSpeeds(motorSpeed: float, baseSpeed: float, reversed: bool):
        if reversed:
            if motorSpeed > 0:
                leftMotorSpeed = -baseSpeed + motorSpeed
                rightMotorSpeed = -baseSpeed
            else:
                leftMotorSpeed = -baseSpeed
                rightMotorSpeed = -baseSpeed - motorSpeed
        else:
            if motorSpeed > 0:
                leftMotorSpeed = baseSpeed
                rightMotorSpeed = baseSpeed - motorSpeed
            else:
                leftMotorSpeed = baseSpeed + motorSpeed
                rightMotorSpeed = baseSpeed

        return leftMotorSpeed, rightMotorSpeed
    
    @staticmethod
    def spin(clockwise, spinPower):
        if clockwise:
            return spinPower, -spinPower
        
        return -spinPower, spinPower
    
    @staticmethod
    def findObstacles(
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

            otherRobotRectangle = RobotHelper.getRectangle(otherRobot, ROBOT_WIDTH, ROBOT_LENGTH)

            if GeometryUtils.hasIntersection(rectangle, otherRobotRectangle):
                obstacles.append(otherRobot)

        for i in range(len(fieldData.robots)):
            if i == robotId:
                continue

            otherRobot = fieldData.robots[i]

            otherRobotRectangle = RobotHelper.getRectangle(otherRobot, ROBOT_WIDTH, ROBOT_LENGTH)

            if GeometryUtils.hasIntersection(rectangle, otherRobotRectangle):
                obstacles.append(otherRobot)

        return obstacles

    @staticmethod
    def findClosestObstacle(
        robotId: int,
        fieldData: FieldData,
        oponnentFieldData: FieldData,
        targetPosition: tuple[float, float]
    ) -> Robot | None:
        obstacles = Motion.findObstacles(robotId, fieldData, oponnentFieldData, targetPosition)

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
    def findTangentPointObstacle(
        robotId: int,
        fieldData: FieldData,
        opponentFieldData: FieldData,
        targetPosition: tuple[float, float]
    ):
        obstacleRobot = Motion.findClosestObstacle(
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