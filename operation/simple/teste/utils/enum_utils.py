from domain.enums.robot_behavior_enum import RobotBehaviorEnum


class EnumUtils:
    @staticmethod
    def get_robot_behavior_enum_by_name(name: str):
        name = name.lower()

        if name == "stopped":
            return RobotBehaviorEnum.STOPPED
        elif name == "manual":
            return RobotBehaviorEnum.MANUAL
        elif name == "ball_tracker":
            return RobotBehaviorEnum.BALL_TRACKER
        else:
            return RobotBehaviorEnum.NONE