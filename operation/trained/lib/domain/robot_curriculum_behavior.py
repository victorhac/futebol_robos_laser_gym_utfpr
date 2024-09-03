from lib.enums.position_enum import PositionEnum
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum

class RobotCurriculumBehavior:
    def __init__(
        self,
        robot_curriculum_behavior_enum: RobotCurriculumBehaviorEnum,
        id: int,
        is_yellow: bool,
        position_enum: PositionEnum,
        steps: int = 1000,
        is_positive_distance_beta: bool = True,
        distance_range: tuple[float, float] | None = None,
        start_distance: float | None = None,
        is_positive_velocity_beta: bool = True,
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

        if distance_range is None:
            self.distance_beta = 0
        else:
            self.distance_beta = RobotCurriculumBehavior._get_beta(
                steps,
                distance_range,
                is_positive_distance_beta)

        self.alpha_range = (0,1)
        self.velocity_alpha = start_velocity_alpha

        self.velocity_beta = RobotCurriculumBehavior._get_beta(
            steps,
            self.alpha_range,
            is_positive_velocity_beta)

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

    def _is_distance_in_limit(self):
        if self.distance_beta > 0:
            return self.distance == self.distance_range[1]
        elif self.distance_beta < 0:
            return self.distance == self.distance_range[0]
        return True
    
    def _is_velocity_alpha_in_limit(self):
        if self.velocity_beta > 0:
            return self.velocity_alpha == 1
        elif self.velocity_beta < 0:
            return self.velocity_alpha == 0
        return True

    def is_over(self):
        if self.robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.BALL_FOLLOWING:
            return self._is_distance_in_limit() and self._is_velocity_alpha_in_limit()
        elif self.robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.FROM_MODEL:
            return self._is_distance_in_limit()
        return True
    
    def has_behavior(self, robot_curriculum_behavior_enum: RobotCurriculumBehaviorEnum):
        return self.robot_curriculum_behavior_enum == robot_curriculum_behavior_enum
    
    @staticmethod
    def _truncate(value: float, range: tuple[float, float]):
        return max(range[0], min(value, range[1]))
    
    @staticmethod
    def _get_beta(
        steps: int,
        range: tuple[float, float],
        is_positive: bool
    ):
        beta = (range[1] - range[0]) / steps
        if not is_positive:
            return -beta
        return beta