from lib.enums.position_enum import PositionEnum
import numpy as np

class BallCurriculumBehavior:
    def __init__(
        self,
        position_enum: PositionEnum,
        updates_per_task: int = 10,
        is_positive_distance_beta: bool = True,
        distance_range: 'tuple[float, float] | None' = None
    ):
        self.position_enum = position_enum
        self.updates_per_task = updates_per_task
        self.is_positive_distance_beta = is_positive_distance_beta
        self.distance_range = distance_range
        
        if distance_range is None:
            self.distance = None
            self.distance_beta = None
        else:
            self.distance = BallCurriculumBehavior._get_start_distance(
                is_positive_distance_beta,
                distance_range
            )
            self.distance_beta = BallCurriculumBehavior._get_beta(
                updates_per_task,
                distance_range,
                is_positive_distance_beta
            )
            
    def update(self, times: int = 1):
        if self.distance_range is not None:
            def clip(value: float, range: 'tuple[float, float]'):
                return np.clip(value, range[0], range[1])

            self.distance = clip(
                self.distance + self.distance_beta * times,
                self.distance_range)

    def reset(self):
        if self.distance_range is not None:
            self.distance = BallCurriculumBehavior._get_start_distance(
                self.is_positive_distance_beta,
                self.distance_range)
    
    def is_over(self):
        return self._is_distance_in_limit()

    def _is_distance_in_limit(self):
        if self.distance_beta > 0:
            return self.distance == self.distance_range[1]
        elif self.distance_beta < 0:
            return self.distance == self.distance_range[0]
        return True

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
    def _get_start_distance(is_positive_beta: float, range: 'tuple[float, float]'):
        if is_positive_beta:
            return range[0]
        return range[1]