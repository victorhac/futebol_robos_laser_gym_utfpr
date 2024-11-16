

import math


from configuration.configuration import Configuration
from domain.entity import Entity
from utils.grsim_utils import GrSimUtils
from utils.geometry_utils import GeometryUtils

class MotionUtils:
    @staticmethod
    def truncate_motor_speed(motorSpeed: float, baseSpeed: float):
        if motorSpeed > baseSpeed:
            motorSpeed = baseSpeed
        elif motorSpeed < -baseSpeed:
            motorSpeed = -baseSpeed
        return motorSpeed
    
    @staticmethod
    def go_to_point(
            robot: Entity, 
            targetPosition: 'tuple[float, float]',
            isLeftTeam: bool,
            lastError: float = 0):
        
        configuration = Configuration.get_object()
        
        position = robot.position
        positionX = position.x
        positionY = position.y
        robotAngle = position.theta

        xTarget, yTarget = targetPosition

        angleToTarget = math.atan2(yTarget - positionY, xTarget - positionX)

        error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)   

        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
            reversed = not isLeftTeam
            robotAngle = GeometryUtils.normalizeInPI(robotAngle + math.pi)
            error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)
        else:
            reversed = isLeftTeam

        kP = configuration.motion_pid_constants_kp
        kD = configuration.motion_pid_constants_kd

        motorSpeed = (kP * error) + (kD * (error - lastError))

        baseSpeed = configuration.robot_speed_max_radians_seconds

        motorSpeed = MotionUtils.truncate_motor_speed(motorSpeed, baseSpeed)

        leftMotorSpeed, rightMotorSpeed = MotionUtils._getSpeeds(motorSpeed, baseSpeed, reversed)

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
    
    def FaceDirection(Robot, targetPosition:'tuple[float, float]', isLeftTeam, lastError):
        
        configuration = Configuration.get_object()

        position = Robot.position

        positionX = position.x
        positionY = position.y
        robotAngle = position.theta

        xTarget, yTarget = targetPosition

        angleToTarget = math.atan2(yTarget - positionY, xTarget - positionX)

        error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)

        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
            reversed = not isLeftTeam
            robotAngle = GeometryUtils.normalizeInPI(robotAngle + math.pi)
            error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)
        else:
            reversed = isLeftTeam
            
        kP = configuration.motion_pid_constants_kp
        kD = configuration.motion_pid_constants_kd

        motorSpeed = (kP * error) + (kD * (error - lastError))

        baseSpeed = configuration.robot_speed_max_radians_seconds

        motorSpeed = MotionUtils.truncate_motor_speed(motorSpeed, baseSpeed)

        motorSpeed = kP * error

        leftMotorSpeed = motorSpeed

        rightMotorSpeed = -motorSpeed

        return leftMotorSpeed, rightMotorSpeed, error
            

    def GoOnDirection(direction, robot, is_left_team, lastError: float = 0):
        configuration = Configuration.get_object()

        position =  robot.position

        positionX = position.x
        positionY = position.y
        robotAngle = position.theta

        # Use the direction vector as the target position
        xTarget, yTarget = direction

        angleToTarget = math.atan2(yTarget - positionY, xTarget - positionX)

        error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)

        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
            reversed = not is_left_team
            robotAngle = GeometryUtils.normalizeInPI(robotAngle + math.pi)
            error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)
        else:
            reversed = is_left_team

        kP = configuration.motion_pid_constants_kp
        kD = configuration.motion_pid_constants_kd

        motorSpeed = (kP * error) + (kD * (error - lastError))

        baseSpeed = configuration.robot_speed_max_radians_seconds

        motorSpeed = MotionUtils.truncate_motor_speed(motorSpeed, baseSpeed)

        leftMotorSpeed, rightMotorSpeed = MotionUtils._getSpeeds(motorSpeed, baseSpeed, reversed)

        return leftMotorSpeed, rightMotorSpeed, error
    
    
    def goToOrbitPoint(robot,
                targetPosition: 'tuple[float, float]',
                isLeftTeam: bool,
                orientation: int,
                lastError: float = 0,
                phaseShift: float = -math.pi*1.25,
                deslocamento: int = 0.075): 
            configuration = Configuration.get_object()

            position = robot.position

            positionX = position.x
            positionY = position.y
            robotAngle = position.theta

            xTarget, yTarget = targetPosition

            # Add a phase shift to the angle calculation for orbital motion
            angleToTarget = math.atan2(yTarget - positionY, xTarget - positionX) + (orientation * phaseShift)

            error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)

            if abs(error) > math.pi / 2.0 + math.pi / 20.0:
                reversed = True
                robotAngle = GeometryUtils.normalizeInPI(robotAngle + math.pi)
                error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)
            else:
                reversed = False

            kP = configuration.motion_pid_constants_kp
            kD = configuration.motion_pid_constants_kd

            motorSpeed = (kP * error) + (kD * (error - lastError))

            baseSpeed = configuration.robot_speed_max_radians_seconds

            motorSpeed = MotionUtils.truncate_motor_speed(motorSpeed, baseSpeed)

            leftMotorSpeed, rightMotorSpeed = MotionUtils._getSpeeds(motorSpeed, baseSpeed, reversed)

            return leftMotorSpeed, rightMotorSpeed, error
