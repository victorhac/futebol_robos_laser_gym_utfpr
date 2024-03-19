from ..domain.robot import Robot
from ..geometry.geometry import Geometry
from ..helpers.configuration_helper import ConfigurationHelper
from ..helpers.robot_helper import RobotHelper
from ..helpers.firasim_helper import FIRASimHelper

import math

class Motion:
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