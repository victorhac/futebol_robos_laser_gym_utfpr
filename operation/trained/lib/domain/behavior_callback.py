import os
import logging
from collections import deque
import numpy as np

from stable_baselines3.common.callbacks import BaseCallback
from lib.domain.curriculum_task import CurriculumTask

class BehaviorCallback(BaseCallback):
    def __init__(
        self,
        check_frequency: int,
        total_timesteps: int,
        model_name: str,
        save_path: str,
        log_path: str,
        number_robot_blue: int,
        number_robot_yellow: int,
        tasks: 'list[CurriculumTask]',
        threshold=0.7,
        number_games=100,
        log_interval=10000,
        verbose=1
    ):
        super(BehaviorCallback, self).__init__(verbose)

        self.check_frequency = check_frequency
        self.model_name = model_name
        self.save_path = save_path
        self.threshold = threshold
        self.number_games = number_games
        self.log_interval = log_interval
        self.log_path = log_path

        self.scores = deque(maxlen=number_games)
        self.number_robot_blue = number_robot_blue
        self.number_robot_yellow = number_robot_yellow
        self.total_timesteps = total_timesteps

        self.tasks = tasks

        self.update_count = 0
        self.current_task_index = 0
        self.current_task = self.tasks[self.current_task_index]
        self.opponent_model_path = None

        self.log_file_name = "log.txt"

        logging.basicConfig(level=logging.DEBUG, format='%(asctime)s: %(message)s')

    def _get_total_num_on_step_calls(self):
        return self.total_timesteps // self.training_env.num_envs
    
    def _get_num_on_step_calls(self):
        return self.num_timesteps // self.training_env.num_envs

    def _try_save_model(self):
        num_calls = self._get_total_num_on_step_calls()
        num_calls_to_update = num_calls // self.check_frequency

        if (self._get_num_on_step_calls()) % num_calls_to_update == 0:
            self._save_model()
        
    def _save_temporary_opponent_model(self):
        if self.opponent_model_path is not None:
            os.remove(self.opponent_model_path)

        model_path = os\
            .path\
            .join(
                "temp",
                f"opponent_model_task_{self.current_task_index + 1}_update_{self.update_count}.zip"
            )
        
        self.model.save(model_path)
        self.opponent_model_path = model_path

    def _save_model(self):
        model_path = os.path.join(
            self.save_path,
            f'{self.model_name}_model_task_{self.current_task_index + 1}_update_{self.update_count}_{self.num_timesteps}_steps.zip')
        
        self.model.save(model_path)

        return model_path
    
    def _set_next_task(self):
        self.current_task_index += 1
        self.update_count = 0

        if self.current_task_index < len(self.tasks):
            self.current_task = self.tasks[self.current_task_index]
            self._set_task()

    def _update_behaviors(self):
        self.current_task.update()
        self.update_count += 1
        self._set_task()

    def _set_previous_model_to_opponent(self):
        self._save_temporary_opponent_model()
        self.current_task.set_opponent_model_path(self.opponent_model_path)

    def _set_task(self):
        self._set_previous_model_to_opponent()
        self.training_env.env_method('set_task', self.current_task)

    def _on_rollout_start(self):
        self._set_task()

    def _try_update_scores(self):
        dones = self.locals["dones"]
        last_games_scores = self.training_env.get_attr("last_game_score")

        for i in range(len(dones)):
            if dones[i]:
                last_score = last_games_scores[i]

                if last_score is not None:
                    self.scores.append(last_score)

    def _log(self, text: str):
        with open(f"{self.log_path}/{self.log_file_name}", 'a') as file:
            file.write(text)

    def _try_log(self):
        if self._get_num_on_step_calls() % self.log_interval == 0 and\
                self._is_scores_max_len():
            self._log(f"Task: {self.current_task_index + 1}; "\
                f"Update: {self.update_count}; "\
                f"Last {self.number_games} games score: {np.mean(self.scores)}.\n")
            
    def _is_scores_max_len(self):
        return len(self.scores) == self.scores.maxlen
    
    def _has_scores_mean_exceeded_threshold(self):
        return np.mean(self.scores) > self.threshold

    def _on_step(self) -> bool:
        if self.num_timesteps > self.total_timesteps:
            return False
        
        self._try_save_model()
        self._try_log()

        if any(self.locals["dones"]):
            self._try_update_scores()

            if self._is_scores_max_len() and\
                    self._has_scores_mean_exceeded_threshold():
                self.scores.clear()

                if self.current_task.all_over_behavior():
                    if self.current_task_index + 1 < len(self.tasks):
                        self._set_next_task()
                    else:
                        return False
                else:
                    self._update_behaviors()

        return True
    
    def _on_training_start(self):
        self._set_task()

    def _on_training_end(self):
        self._save_model()