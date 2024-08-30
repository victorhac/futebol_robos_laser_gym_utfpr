import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
import os
from collections import deque

from lib.helpers.behavior.behavior_helper import BehaviorHelper
from operation.trained.lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum

import tempfile

from stable_baselines3 import PPO

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

        self.behaviors = [
            BehaviorHelper.get_task_1_behaviors(number_robot_blue, number_robot_yellow),
            BehaviorHelper.get_task_2_behaviors(number_robot_blue, number_robot_yellow),
            BehaviorHelper.get_task_3_behaviors(number_robot_blue, number_robot_yellow),
            BehaviorHelper.get_task_4_behaviors(number_robot_blue, number_robot_yellow)
        ]

        current_task = 0

        self.current_task = current_task
        self.current_behavior = self.behaviors[current_task]

        self._set_behaviors()

    def _get_model(self):
        with tempfile.TemporaryDirectory() as tmpdirname:
            model_path = os.path.join(tmpdirname, "temp_model")
            self.model.save(model_path)
            return PPO.load(model_path) #TODO: deixar genérico

    def _save_model(self):
        model_path = os.path.join(self.save_path, f'model_task_{self.current_task + 1}_{self.num_timesteps}')
        self.model.save(model_path)
        return model_path
    
    def _set_next_task(self):
        self.current_task += 1

        if self.current_task < 4:
            self.current_behavior = self.behaviors[self.current_task]

    def _set_behaviors(self):
        self.training_env.env_method('set_behaviors', self.current_behavior)

    def _update_behaviors(self):
        blue_behaviors = self.current_behavior["blue"]
        yellow_behaviors = self.current_behavior["yellow"]

        for i in range(len(blue_behaviors)):
            blue_behaviors[i].update()
        
        for i in range(len(yellow_behaviors)):
            yellow_behaviors[i].update()

        self._set_previous_model_to_opponent()

    def _any_over_behavior(self):
        blue_behaviors = self.current_behavior["blue"]
        yellow_behaviors = self.current_behavior["yellow"]

        return any(item.is_over() for item in blue_behaviors) or\
            any(item.is_over() for item in yellow_behaviors)
    
    def _set_previous_model_to_opponent(self):
        yellow_behaviors = self.current_behavior["yellow"]

        for item in yellow_behaviors:
            if item.has_behavior(RobotCurriculumBehaviorEnum.FROM_MODEL):
                model = self._get_model()
                item.set_model(model)

    def _on_step(self) -> bool:
        last_scores = self.training_env.get_attr('last_game_score')
        
        self.scores.extend(last_scores)

        if self.num_timesteps % self.check_freq == 0:
            self._save_model()

        if len(self.scores) == self.n_games:
            if np.mean(self.scores) >= self.threshold:
                self.scores = []
                self._update_behaviors()

                if self._any_over_behavior():
                    if self.current_task + 1 < len(self.behaviors):
                        self._set_next_task(self)
                    else:
                        return False

        return True
