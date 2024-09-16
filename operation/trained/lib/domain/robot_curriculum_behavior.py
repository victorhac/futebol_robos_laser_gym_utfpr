from lib.enums.position_enum import PositionEnum
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum

class RobotCurriculumBehavior:
    def __init__(
        self,
        robot_curriculum_behavior_enum: RobotCurriculumBehaviorEnum,
        robot_id: int,
        is_yellow: bool,
        position_enum: PositionEnum,
        updates_per_task: int,
        is_positive_distance_beta: bool = True,
        distance_range: 'tuple[float, float]' | None = None,
        is_positive_velocity_beta: bool = True,
        start_velocity_alpha: float | None = None
    ):
        self.robot_curriculum_behavior_enum = robot_curriculum_behavior_enum
        self.robot_id = robot_id
        self.is_yellow = is_yellow
        self.position_enum = position_enum
        self.updates_per_task = updates_per_task

        self.is_positive_distance_beta = is_positive_distance_beta
        self.distance_range = distance_range
        self.distance_beta = None
        self.distance = None

        self.is_positive_velocity_beta = is_positive_velocity_beta
        self.start_velocity_alpha = start_velocity_alpha
        self.velocity_alpha = start_velocity_alpha
        self.alpha_range = None
        self.velocity_beta = None

        if distance_range is not None:
            self.distance = RobotCurriculumBehavior._get_start_value(
                is_positive_velocity_beta,
                distance_range
            )
            self.distance_beta = RobotCurriculumBehavior._get_beta(
                updates_per_task,
                distance_range,
                is_positive_distance_beta)

        if start_velocity_alpha is not None:
            if is_positive_velocity_beta:
                self.alpha_range = (start_velocity_alpha, 1)
            else:
                self.alpha_range = (0, start_velocity_alpha)
                
            self.velocity_beta = RobotCurriculumBehavior._get_beta(
                updates_per_task,
                self.alpha_range,
                is_positive_velocity_beta)

        self.model_path = None

    def set_model_path(self, model_path):
        self.model_path = model_path

    def update(self):
        def trucate(value: float, range: 'tuple[float, float]'):
            return RobotCurriculumBehavior._truncate(value, range)

        if self.robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.BALL_FOLLOWING:
            self.velocity_alpha = trucate(self.velocity_alpha + self.velocity_beta, self.alpha_range)
            self.distance = trucate(self.distance + self.distance_beta, self.distance_range)
        elif self.robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.FROM_MODEL:
            self.distance = trucate(self.distance + self.distance_beta, self.distance_range)

    def reset(self):
        self.velocity_alpha = self.start_velocity_alpha
        self.model_path = None
        
        if self.distance_range is not None:
            self.distance = RobotCurriculumBehavior._get_start_value(
                self.is_positive_velocity_beta,
                self.distance_range)
        else:
            self.distance = None

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
    def _truncate(value: float, range: 'tuple[float, float]'):
        return max(range[0], min(value, range[1]))
    
    @staticmethod
    def _get_beta(
        updates_per_task: int,
        range: 'tuple[float, float]',
        is_positive: bool
    ):
        beta = (range[1] - range[0]) / updates_per_task
        if not is_positive:
            return -beta
        return beta
    
    @staticmethod
    def _get_start_value(is_positive_beta: float, range: 'tuple[float, float]'):
        if is_positive_beta:
            return range[0]
        return range[1]