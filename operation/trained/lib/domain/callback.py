import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
import os
from collections import deque

from lib.helpers.behavior.behavior_helper import BehaviorHelper

class ScoreCallback(BaseCallback):
    def __init__(
        self,
        check_freq,
        save_path,
        number_robot_blue: int,
        number_robot_yellow: int,
        threshold=0.6,
        n_games=100,
        verbose=1
    ):
        super(ScoreCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.save_path = save_path
        self.threshold = threshold
        self.n_games = n_games
        self.scores = deque(maxlen=n_games)
        self.number_robot_blue = number_robot_blue
        self.number_robot_yellow = number_robot_yellow

        self.number = 0

        first_model_path = self._save_model()

        self.behaviors = [
            BehaviorHelper.get_task_1_behaviors(number_robot_blue, number_robot_yellow),
            BehaviorHelper.get_task_2_behaviors(number_robot_blue, number_robot_yellow),
            BehaviorHelper.get_task_3_behaviors(number_robot_blue, number_robot_yellow),
            BehaviorHelper.get_task_4_behaviors(number_robot_blue, number_robot_yellow)
        ]

        current_task = 0

        self.current_task = current_task
        self.current_behavior = self.behaviors[current_task]
        self.current_behavior["blue"][current_task].set_initial_model_path(first_model_path)

    def _save_model(self):
        self.number += 1
        model_path = os.path.join(self.save_path, 'model' + self.number)
        self.model.save(model_path)
        return model_path
    
    def _set_next_task(self):
        self.current_task += 1

        if self.current_task < 4:
            self.current_behavior = self.behaviors[self.current_task]

    def _on_step(self) -> bool:
        last_score = self.training_env.get_attr('last_game_score')
        
        self.scores.extend(last_score)

        if len(self.scores) == self.n_games:
            if np.mean(self.scores) >= self.threshold:
                self._set_next_task(self)

        return True
