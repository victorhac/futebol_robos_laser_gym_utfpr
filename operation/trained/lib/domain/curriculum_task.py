from collections import deque
import numpy as np

from lib.domain.ball_curriculum_behavior import BallCurriculumBehavior
from lib.domain.robot_curriculum_behavior import RobotCurriculumBehavior
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum

class CurriculumTask:
    def __init__(
        self,
        id: str,
        behaviors: 'list[RobotCurriculumBehavior]',
        ball_behavior: BallCurriculumBehavior,
        update_count: int=0,
        updates_per_task: int=100,
        games_count: int=100,
        threshold_intervals: 'list[tuple[int, float]]'=[],
        default_threshold: float=.7
    ):
        self.id = id
        self.behaviors = behaviors
        self.ball_behavior = ball_behavior
        self.update_count = 0
        self.updates_per_task = updates_per_task

        self.default_threshold=default_threshold
        self.threshold_intervals = threshold_intervals

        self.scores = deque(maxlen=games_count)

        self.update(update_count)

    def update(self, times: int = 1):
        for item in self.behaviors:
            item.update(times)

        self.ball_behavior.update(times)
        self.update_count += times

        self.scores.clear()

    def reset(self):
        for item in self.behaviors:
            item.reset()

        self.ball_behavior.reset()
        self.update_count = 0

    def set_scores(self, scores: 'list[int]'):
        self.scores.extend(scores)

    def get_scores_log_text(self):
        return f"Last {len(self.scores)} games score: {self.get_scores_mean()}"
    
    def is_limit_reached(self):
        if len(self.scores) < self.scores.maxlen:
            return False
        
        update_threshold = None

        for item in self.threshold_intervals:
            if item[0] > self.update_count:
                break

            update_threshold = item[1]

        if update_threshold is None:
            update_threshold = self.default_threshold
        
        return self.get_scores_mean() > update_threshold
    
    def get_scores_mean(self):
        return np.mean(self.scores)

    def set_opponent_model_path(self, model_path: str):
        yellow_behaviors = self.get_yellow_behaviors()

        for item in yellow_behaviors:
            if item.robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.FROM_MODEL:
                item.set_model_path(model_path)

    def get_yellow_behaviors(self) -> 'list[RobotCurriculumBehavior]':
        return filter(lambda item: item.is_yellow, self.behaviors)
    
    def get_blue_behaviors(self) -> 'list[RobotCurriculumBehavior]':
        return filter(lambda item: not item.is_yellow, self.behaviors)
    
    def get_yellow_behaviors_by_robot_id(self, robot_id: int):
        return next((item for item in self.get_yellow_behaviors() if item.robot_id == robot_id), None)

    def get_blue_behaviors_by_robot_id(self, robot_id: int):
        return next((item for item in self.get_blue_behaviors() if item.robot_id == robot_id), None)

    def all_over_behavior(self):
        return self.update_count >= self.updates_per_task