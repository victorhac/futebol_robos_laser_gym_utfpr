class RobotHelper:
    @staticmethod
    def truncateMotorSpeed(motorSpeed: float, baseSpeed: float):
        if motorSpeed > baseSpeed:
            motorSpeed = baseSpeed
        elif motorSpeed < -baseSpeed:
            motorSpeed = -baseSpeed
        return motorSpeed