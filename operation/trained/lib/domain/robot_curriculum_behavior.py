from lib.enums.position_enum import PositionEnum
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum

class RobotCurriculumBehavior:
    def __init__(
        self,
        robot_curriculum_behavior_enum: RobotCurriculumBehaviorEnum,
        id: int,
        is_yellow: bool,
        position_enum: PositionEnum,
        distance_range: tuple[float, float] | None = None,
        start_distance: float | None = None,
        distance_beta: float | None = None,
        velocity_beta: float | None = None,
        start_velocity_alpha: float | None = None
    ):
        self.robot_curriculum_behavior_enum = robot_curriculum_behavior_enum
        self.id = id
        self.is_yellow = is_yellow
        self.position_enum = position_enum

        self.backup_start_velocity_alpha = start_velocity_alpha
        self.backup_start_distance = start_distance

        self.distance_range = distance_range
        self.distance = start_distance
        self.distance_beta = distance_beta

        self.velocity_alpha = start_velocity_alpha
        self.velocity_beta = velocity_beta

    def set_model(self, model):
        self.model = model

    def update(self):
        def trucate(value: float, range: tuple[float, float]):
            return RobotCurriculumBehavior._truncate(value, range)

        if self.robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.BALL_FOLLOWING:
            self.velocity_alpha = trucate(self.velocity_alpha + self.velocity_beta, self.alpha_range)
            self.distance = trucate(self.distance + self.distance_beta, self.distance_range)
        elif self.robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.FROM_MODEL:
            self.distance = trucate(self.distance + self.distance_beta, self.distance_range)

    def reset(self):
        self.velocity_alpha = 0
        self.distance = self.backup_start_distance
        self.model = None

    @staticmethod
    def _truncate(value: float, range: tuple[float, float]):
        return max(range[0], min(value, range[1]))

    def _is_distance_in_range(self):
        return self.distance >= self.distance_range[0] and self.distance <= self.distance_range[1]
    
    def _is_velocity_alpha_in_range(self):
        return self.velocity_alpha >= 0 and self.velocity_alpha <= 1

    def is_over(self): #rever
        if self.robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.BALL_FOLLOWING:
            return not self._is_distance_in_range() or\
                not self._is_velocity_alpha_in_range()
        elif self.robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.FROM_MODEL:
            return not self._is_distance_in_range()
        
        return False
    
    def has_behavior(self, robot_curriculum_behavior_enum: RobotCurriculumBehaviorEnum):
        return self.robot_curriculum_behavior_enum == robot_curriculum_behavior_enum