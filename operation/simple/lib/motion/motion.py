from ..core.data import EntityData
from ..geometry.geometry import Geometry
from ..helpers.configuration_helper import ConfigurationHelper
from ..helpers.robot_helper import RobotHelper
from ..helpers.firasim_helper import FIRASimHelper

import math

class Motion:
    @staticmethod
    def goToPoint(robot: EntityData, 
            targetPosition: 'tuple[float, float]',
            isLeftTeam: bool,
            lastError: float = 0):
        configuration = ConfigurationHelper.getConfiguration()

        position = robot.position

        positionX = position.x
        positionY = position.y
        robotAngle = position.theta

        xTarget, yTarget = FIRASimHelper.normalizePosition(targetPosition[0], targetPosition[1], isLeftTeam)

        angleToTarget = math.atan2(yTarget - positionY, xTarget - positionX)

        error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)

        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
            reversed = True
            robotAngle = Geometry.normalizeInPI(robotAngle + math.pi)
            error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)
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
    
    def FaceDirection(robot: EntityData, targetPosition:'tuple[float, float]', isLeftTeam = True):
        
        configuration = ConfigurationHelper.getConfiguration()

        position = robot.position

        positionX = position.x
        positionY = position.y
        robotAngle = position.theta

        xTarget, yTarget = FIRASimHelper.normalizePosition(targetPosition[0], targetPosition[1], isLeftTeam)

        angleToTarget = math.atan2(yTarget - positionY, xTarget - positionX)

        error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)

        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
            reversed = True
            robotAngle = Geometry.normalizeInPI(robotAngle + math.pi)
            error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)
        else:
            reversed = False
            
        kP = configuration["motion"]["pid"]["constants"]["Kp"]
        kD = configuration["motion"]["pid"]["constants"]["Kd"]

        motorSpeed = (kP * error) + (kD * (error))

        baseSpeed = configuration["robot"]["speed"]["base"]

        motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

        motorSpeed = kP * error

        leftMotorSpeed = motorSpeed

        rightMotorSpeed = -motorSpeed

        return leftMotorSpeed, rightMotorSpeed, error
    def GoOnDirection(direction, robot, lastError: float = 0):
        configuration = ConfigurationHelper.getConfiguration()

        position = robot.position

        positionX = position.x
        positionY = position.y
        robotAngle = position.theta

        # Use the direction vector as the target position
        xTarget, yTarget = direction

        angleToTarget = math.atan2(yTarget - positionY, xTarget - positionX)

        error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)

        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
            reversed = True
            robotAngle = Geometry.normalizeInPI(robotAngle + math.pi)
            error = Geometry.smallestAngleDiff(angleToTarget, robotAngle)
        else:
            reversed = False

        kP = configuration["motion"]["pid"]["constants"]["Kp"]
        kD = configuration["motion"]["pid"]["constants"]["Kd"]

        motorSpeed = (kP * error) + (kD * (error - lastError))

        baseSpeed = configuration["robot"]["speed"]["base"]

        motorSpeed = RobotHelper.truncateMotorSpeed(motorSpeed, baseSpeed)

        leftMotorSpeed, rightMotorSpeed = Motion._getSpeeds(motorSpeed, baseSpeed, reversed)

        return leftMotorSpeed, rightMotorSpeed, error
    
    def Orbit(fsimcontroler, id: int, isLeftTeam: bool, lastError: float = 0):
        raio = 0.0002

        fieldData = fsimcontroler[0]
        vision = fsimcontroler[1]
        teamControl = fsimcontroler[2]

        robot = fieldData.robots[id]
        ball = fieldData.ball
        center = ball.position.x, ball.position.y

        # Compute the initial angle in radians
        theta = math.atan2(center[1] - robot.position.y, center[0] - robot.position.x)
            
        x_d = center[0] + raio * math.cos(theta)
        y_d = center[1] + raio * math.sin(theta)
        orientation =1

        while ball.position.x < 0.1: 
            vision.update()
            if(not Geometry.isClose(center, (robot.position.x, robot.position.y), 0.05)):
                center = ball.position.x, ball.position.y
                x_d = center[0] + raio * math.cos(theta)
                y_d = center[1] + raio * math.sin(theta)
                
            desired_position = (x_d, y_d)
            
            if(ball.position.y > 0):
                orientation = -1
            else:
                orientation = 1
            leftMotorSpeed, rightMotorSpeed, error = Motion.goToOrbitPoint(robot, desired_position, isLeftTeam, orientation, lastError)

            teamControl.transmit_robot(id, leftMotorSpeed, rightMotorSpeed)
            theta += 0.0614
            lastError = error