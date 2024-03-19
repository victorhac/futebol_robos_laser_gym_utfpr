from ..domain.rectangle import Rectangle
from ..domain.robot import Robot


class RobotHelper:
    @staticmethod
    def truncateMotorSpeed(motorSpeed: float, baseSpeed: float):
        if motorSpeed > baseSpeed:
            motorSpeed = baseSpeed
        elif motorSpeed < -baseSpeed:
            motorSpeed = -baseSpeed
        return motorSpeed
    
    @staticmethod
    def getRectangle(robot: Robot, width: float, height: float):
        position = robot.position

        return Rectangle(
            (position.x, position.y),
            width,
            height,
            robot.position.theta
        )