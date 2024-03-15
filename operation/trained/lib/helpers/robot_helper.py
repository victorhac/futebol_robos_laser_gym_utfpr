class RobotHelper:
    @staticmethod
    def truncateMotorSpeed(motorSpeed: float):
        if motorSpeed > 30:
            motorSpeed = 30
        elif motorSpeed < -30:
            motorSpeed = -30
        return motorSpeed