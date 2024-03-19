import time
import math

from lib.comm.vision import ProtoVision
from lib.comm.control import ProtoControl
from lib.geometry.geometry import Geometry

from lib.helpers.configuration_helper import ConfigurationHelper
from lib.motion.motion import Motion
from lib.helpers.field_helper import FieldHelper
from lib.helpers.firasim_helper import FIRASimHelper

from lib.domain.field_data import FieldData
from lib.domain.robot import Robot
from lib.domain.rectangle import Rectangle
from lib.helpers.robot_helper import RobotHelper

CONFIGURATION = ConfigurationHelper.getConfiguration()

IS_YELLOW_TEAM = CONFIGURATION["team"]["is-yellow-team"]
IS_YELLOW_LEFT_TEAM = CONFIGURATION["team"]["is-yellow-left-team"]

IS_LEFT_TEAM = FieldHelper.isLeftTeam(IS_YELLOW_TEAM, IS_YELLOW_LEFT_TEAM)

ROBOT_LENGTH = CONFIGURATION["robot"]["length"]
ROBOT_WIDTH = CONFIGURATION["robot"]["width"]

FIELD_WIDTH = CONFIGURATION["field"]["width"]
FIELD_LENGTH = CONFIGURATION["field"]["length"]

MOTION_COLLISION_AVOIDANCE_MIN_DISTANCE = CONFIGURATION["motion"]["collision-avoidance"]["min-distance"]

FIRASIM_CONTROL_IP = CONFIGURATION["FIRASim"]["control"]["ip"]
FIRASIM_CONTROL_PORT = CONFIGURATION["FIRASim"]["control"]["port"]
FIRASIM_VISION_IP = CONFIGURATION["FIRASim"]["vision"]["ip"]
FIRASIM_VISION_PORT = CONFIGURATION["FIRASim"]["vision"]["port"]

def findObstacles(
    robotId: int,
    fieldData: FieldData,
    opponentFieldData: FieldData,
    targetPosition: tuple[float, float]
) -> list[Robot] | None:
    robot = fieldData.robots[robotId]
    obstacles = []

    center = Geometry.getMidpoint((robot.position.x, robot.position.y), targetPosition)
    width = Geometry.distance((robot.position.x, robot.position.y), targetPosition)
    height = ROBOT_LENGTH
    angle = math.atan2(targetPosition[1] - robot.position.y, targetPosition[0] - robot.position.x)

    rectangle = Rectangle(center, width, height, angle)

    for i in range(len(opponentFieldData.robots)):
        otherRobot = opponentFieldData.robots[i]

        otherRobotRectangle = RobotHelper.getRectangle(otherRobot, ROBOT_WIDTH, ROBOT_LENGTH)

        if Geometry.hasIntersection(rectangle, otherRobotRectangle):
            obstacles.append(otherRobot)

    for i in range(len(fieldData.robots)):
        if i == robotId:
            continue

        otherRobot = fieldData.robots[i]

        otherRobotRectangle = RobotHelper.getRectangle(otherRobot, ROBOT_WIDTH, ROBOT_LENGTH)

        if Geometry.hasIntersection(rectangle, otherRobotRectangle):
            obstacles.append(otherRobot)

    return obstacles

def findClosestObstacle(
    robotId: int,
    fieldData: FieldData,
    oponnentFieldData: FieldData,
    targetPosition: tuple[float, float]
) -> Robot | None:
    obstacles = findObstacles(robotId, fieldData, oponnentFieldData, targetPosition)

    if len(obstacles) == 0:
        return None

    robot = fieldData.robots[robotId]
    closestObstacle = obstacles[0]

    minDistance = Geometry.distance(
        (robot.position.x, robot.position.y),
        (closestObstacle.position.x, closestObstacle.position.y))

    for obstacle in obstacles:
        distance = Geometry.distance(
            (robot.position.x, robot.position.y),
            (obstacle.position.x, obstacle.position.y))

        if distance < minDistance:
            closestObstacle = obstacle
            minDistance = distance

    return closestObstacle

def findTangentPointObstacle(
    robotId: int,
    fieldData: FieldData,
    opponentFieldData: FieldData,
    targetPosition: tuple[float, float]
):
    obstacleRobot = findClosestObstacle(
        robotId,
        fieldData,
        opponentFieldData,
        targetPosition)

    if obstacleRobot is None:
        return None

    robot = fieldData.robots[robotId]
    center = (obstacleRobot.position.x, obstacleRobot.position.y)
    point = (robot.position.x, robot.position.y)

    distance = Geometry.distance(center, point)

    circleRadius = MOTION_COLLISION_AVOIDANCE_MIN_DISTANCE

    if not distance > circleRadius:
        circleRadius = distance - 0.01

    tangentPoints = Geometry.getTangentPoints(center, circleRadius, point)

    if tangentPoints is None:
        return None

    rectangleField = Rectangle((0,0), FIELD_WIDTH, FIELD_LENGTH, 0)

    angleToTangent1 = math.atan2(tangentPoints[0][1] - robot.position.y, tangentPoints[0][0] - robot.position.x)
    rectangleTangent1 = Rectangle(tangentPoints[0], ROBOT_LENGTH, ROBOT_WIDTH, angleToTangent1)

    angleToTangent2 = math.atan2(tangentPoints[1][1] - robot.position.y, tangentPoints[1][0] - robot.position.x)
    rectangleTangent2 = Rectangle(tangentPoints[1], ROBOT_LENGTH, ROBOT_WIDTH, angleToTangent2)

    isTangent1InsideField = Geometry.contains(rectangleField, rectangleTangent1)
    isTangent2InsideField = Geometry.contains(rectangleField, rectangleTangent2)

    if isTangent1InsideField and isTangent2InsideField:
        # TODO: verificar quais pontos são válidos e qual é o melhor pelo número de obstáculos 
        # a partir de cada ponto ou pela distância até o objetivo
        return tangentPoints[1]
    elif isTangent1InsideField:
        return tangentPoints[0]
    elif isTangent2InsideField:
        return tangentPoints[1]
    
    return tangentPoints[1]

def getProtoVision(isYellowTeam: bool, fieldData: FieldData):
    return ProtoVision(
        team_color_yellow=isYellowTeam,
        field_data=fieldData,
        vision_ip=FIRASIM_VISION_IP,
        vision_port=FIRASIM_VISION_PORT)

def getProtoControl():
    return ProtoControl(
        team_color_yellow=IS_YELLOW_TEAM, 
        control_ip=FIRASIM_CONTROL_IP, 
        control_port=FIRASIM_CONTROL_PORT)

def updateVisions(vision: ProtoVision, oppositeTeamVision: ProtoVision):
    vision.update()
    oppositeTeamVision.update()

def main():
    fieldData = FieldData()
    opponentFieldData = FieldData()

    vision = getProtoVision(IS_YELLOW_TEAM, fieldData)
    opponentVision = getProtoVision(not IS_YELLOW_TEAM, opponentFieldData)
    
    teamControl = getProtoControl()
    
    targetPosition = FIRASimHelper.normalizePosition(x=0.6, y=0.6, isLeftTeam=IS_LEFT_TEAM)

    currentTargetPosition = targetPosition
    robot = fieldData.robots[0]
    ball = fieldData.ball
    position = robot.position
    error = 0

    updateVisions(vision, opponentVision)
    
    while True:

        targetPosition = FIRASimHelper.normalizePosition(ball.position.x, ball.position.y, IS_LEFT_TEAM)

        tangentPoint = findTangentPointObstacle(0, fieldData, opponentFieldData, targetPosition)

        if tangentPoint is not None:
            currentTargetPosition = tangentPoint

        velocities = Motion.goToPoint(robot, currentTargetPosition, error)

        (leftSpeed, rightSpeed, error) = velocities

        teamControl.transmit_robot(0, leftSpeed, rightSpeed)

        currentTargetPosition = targetPosition

        updateVisions(vision, opponentVision)

    teamControl.transmit_robot(0, 0, 0)

if __name__ == '__main__':
    main()
