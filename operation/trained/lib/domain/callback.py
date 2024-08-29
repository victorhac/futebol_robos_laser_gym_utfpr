import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
import os

from lib.domain.robot_curriculum_behavior import RobotCurriculumBehavior
from lib.enums.position_enum import PositionEnum
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum
from lib.helpers.configuration_helper import ConfigurationHelper

NUMBER_ROBOTS_BLUE = ConfigurationHelper.get_rsoccer_team_blue_number_robots()
NUMBER_ROBOTS_YELLOW = ConfigurationHelper.get_rsoccer_team_yellow_number_robots()

class ScoreCallback(BaseCallback):
    def __init__(self, check_freq, save_path, threshold=0.6, n_games=100, verbose=1):
        super(ScoreCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.save_path = save_path
        self.threshold = threshold
        self.n_games = n_games
        self.scores = []

    def get_stopped_behavior(
        self,
        id: int,
        is_yellow: bool,
        position_enum: PositionEnum
    ):
        return RobotCurriculumBehavior(
            id,
            is_yellow,
            position_enum,
            RobotCurriculumBehaviorEnum.STOPPED
        )
    
    def get_default_ball_following_behavior(
        id: int,
        is_yellow: bool,
        position_enum: PositionEnum,
        distance_range: list[float],
        start_distance: float,
        distance_beta: float,
        velocity_beta: float,
        velocity_alpha: float
    ):
        return RobotCurriculumBehavior(
            id: int,
            is_yellow: bool,
            position_enum: PositionEnum,
            distance_range: list[float],
            start_distance: float,
            distance_beta: float,
            velocity_beta: float,
            velocity_alpha: float
        )

    def get_task_1_blue_team_behaviors(self):
        behaviors = []

        
        for i
        
        RobotCurriculumBehavior(

        )

    def _on_step(self) -> bool:
        last_score = self.training_env.get_attr('last_game_score')[0]
        self.scores.append(last_score)

        if len(self.scores) > self.n_games:
            self.scores.pop(0)

        if len(self.scores) == self.n_games:
            if np.mean(self.scores) >= self.threshold:
                model_path = os.path.join(self.save_path, 'best_model')
                self.model.save(model_path)
                print(f"Model saved at {model_path}. Updating opponent behavior...")
                self.training_env.env_method("ds",)

        return True
