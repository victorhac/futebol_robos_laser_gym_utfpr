from lib.enums.position_enum import PositionEnum
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum
import numpy as np

class RobotCurriculumBehavior:
    def __init__(
        self,
        robot_curriculum_behavior_enum: RobotCurriculumBehaviorEnum,
        robot_id: int,
        is_yellow: bool,
        position_enum: PositionEnum,
        updates_per_task: int,
        is_positive_distance_beta: bool = True,
        distance_range: 'tuple[float, float] | None' = None,
        is_positive_velocity_beta: bool = True,
        velocity_alpha_range: 'tuple[float, float] | None' = None
    ):
        self.robot_curriculum_behavior_enum = robot_curriculum_behavior_enum
        self.robot_id = robot_id
        self.is_yellow = is_yellow
        self.position_enum = position_enum
        self.updates_per_task = updates_per_task

        self.is_positive_distance_beta = is_positive_distance_beta
        self.distance = None
        self.distance_range = distance_range
        self.distance_beta = None

        self.is_positive_velocity_beta = is_positive_velocity_beta
        self.velocity_alpha = None
        self.velocity_alpha_range = velocity_alpha_range
        self.velocity_beta = None

        self._try_set_distance()
        self._try_set_velocity_alpha()

        self.model_path = None

    def _try_set_distance(self):
        if self.distance_range is not None:
            self.distance = RobotCurriculumBehavior._get_start_value(
                self.is_positive_distance_beta,
                self.distance_range)
            
            self.distance_beta = RobotCurriculumBehavior._get_beta(
                self.updates_per_task,
                self.distance_range,
                self.is_positive_distance_beta)
            
    def _try_set_velocity_alpha(self):
        if self.velocity_alpha_range is not None:
            self.velocity_alpha = RobotCurriculumBehavior._get_start_value(
                self.is_positive_velocity_beta,
                self.velocity_alpha_range)

            self.velocity_beta = RobotCurriculumBehavior._get_beta(
                self.updates_per_task,
                self.velocity_alpha_range,
                self.is_positive_velocity_beta)

    def set_model_path(self, model_path):
        self.model_path = model_path

    def update(self, times: int = 1):
        def clip(value: float, range: 'tuple[float, float]'):
            return np.clip(value, range[0], range[1])
        
        if self.distance_range is not None:
            self.distance = clip(
                self.distance + self.distance_beta * times,
                self.distance_range)
            
        if self.velocity_alpha_range is not None:
            self.velocity_alpha = clip(
                self.velocity_alpha + self.velocity_beta * times,
                self.velocity_alpha_range)

    def reset(self):
        if self.velocity_alpha_range is not None:
            self.velocity_alpha = RobotCurriculumBehavior._get_start_value(
                self.is_positive_velocity_beta,
                self.velocity_alpha_range)
        else:
            self.velocity_alpha = None

        if self.distance_range is not None:
            self.distance = RobotCurriculumBehavior._get_start_value(
                self.is_positive_velocity_beta,
                self.distance_range)
        else:
            self.distance = None

        self.model_path = None
        
    def _is_distance_in_limit(self):
        if self.distance_range is None:
            return True
        if self.distance_beta > 0:
            return self.distance == self.distance_range[1]
        elif self.distance_beta < 0:
            return self.distance == self.distance_range[0]
        return True
    
    def _is_velocity_alpha_in_limit(self):
        if self.velocity_alpha_range is None:
            return True
        if self.velocity_beta > 0:
            return self.velocity_alpha == self.velocity_alpha_range[1]
        elif self.velocity_beta < 0:
            return self.velocity_alpha == self.velocity_alpha_range[0]
        return True

    def is_over(self):
        return self._is_distance_in_limit() and self._is_velocity_alpha_in_limit()
    
    def has_behavior(self, robot_curriculum_behavior_enum: RobotCurriculumBehaviorEnum):
        return self.robot_curriculum_behavior_enum == robot_curriculum_behavior_enum
    
    def get_velocity_alpha(self):
        if self.velocity_alpha_range is None:
            return 1
        return self.velocity_alpha
    
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