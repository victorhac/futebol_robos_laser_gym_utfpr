from stable_baselines3.common.callbacks import BaseCallback
import os
from collections import deque
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum
from lib.utils.behavior.behavior_utils import BehaviorUtils

class BehaviorCallback(BaseCallback):
    def __init__(
        self,
        check_frequency: int,
        model_name: str,
        save_path: str,
        number_robot_blue: int,
        number_robot_yellow: int,
        threshold=0.6,
        number_games=100,
        updates_per_task=100,
        verbose=1
    ):
        super(BehaviorCallback, self).__init__(verbose)

        self.check_frequency = check_frequency
        self.model_name = model_name
        self.save_path = save_path
        self.threshold = threshold
        self.number_games = number_games

        self.scores = deque(maxlen=number_games)
        self.number_robot_blue = number_robot_blue
        self.number_robot_yellow = number_robot_yellow
        self.updates_per_task = updates_per_task

        self.behaviors = [
            BehaviorUtils.get_task_1_behaviors(
                number_robot_blue,
                number_robot_yellow,
                updates_per_task),
            BehaviorUtils.get_task_2_behaviors(
                number_robot_blue,
                number_robot_yellow,
                updates_per_task),
            BehaviorUtils.get_task_3_behaviors(
                number_robot_blue,
                number_robot_yellow,
                updates_per_task)
        ]

        self.update_count = 0
        self.current_task = 0
        self.current_behavior = self.behaviors[self.current_task]
        self.opponent_model_path = None

        self.previous_saved_model_task_number = None
        self.previous_saved_model_update_number = None

    def _try_save_model(self):
        number_task_to_update = self.updates_per_task // self.check_frequency

        is_savable = self.update_count == self.updates_per_task or\
            (
                self.update_count > 0 and
                self.update_count % number_task_to_update == 0 and
                self.update_count + number_task_to_update <= self.updates_per_task
            )
        
        if is_savable:
            is_current_different_last_saved =\
                self.current_task != self.previous_saved_model_task_number or\
                self.update_count != self.previous_saved_model_update_number
            
            if is_current_different_last_saved:
                self._save_model()
        
    def _save_temporary_opponent_model(self):
        if self.opponent_model_path is not None:
            os.remove(self.opponent_model_path)

        model_path = os\
            .path\
            .join(
                "temp",
                f"opponent_model_task_{self.current_task + 1}_update_{self.update_count}.zip"
            )
        
        self.model.save(model_path)
        self.opponent_model_path = model_path

    def _save_model(self):
        model_path = os.path.join(
            self.save_path,
            f'{self.model_name}_model_task_{self.current_task + 1}_update_{self.update_count}_{self.num_timesteps}_steps.zip')
        
        self.model.save(model_path)

        self.previous_saved_model_task_number = self.current_task
        self.previous_saved_model_update_number = self.update_count

        return model_path
    
    def _set_next_task(self):
        self.current_task += 1
        self.update_count = 0

        if self.current_task < len(self.behaviors):
            self.current_behavior = self.behaviors[self.current_task]
            self._set_behaviors()

    def _update_behaviors(self):
        blue_behaviors = self.current_behavior["blue"]
        yellow_behaviors = self.current_behavior["yellow"]

        for i in range(len(blue_behaviors)):
            blue_behaviors[i].update()
        
        for i in range(len(yellow_behaviors)):
            yellow_behaviors[i].update()

        self.update_count += 1
        self._set_behaviors()

    def _any_over_behavior(self):
        blue_behaviors = self.current_behavior["blue"]
        yellow_behaviors = self.current_behavior["yellow"]

        return all(item.is_over() for item in blue_behaviors) and\
            all(item.is_over() for item in yellow_behaviors)
    
    def _set_previous_model_to_opponent(self):
        self._save_temporary_opponent_model()

        yellow_behaviors = self.current_behavior["yellow"]

        for item in yellow_behaviors:
            if item.has_behavior(RobotCurriculumBehaviorEnum.FROM_MODEL):
                item.set_model_path(self.opponent_model_path)

    def _set_behaviors(self):
        self._set_previous_model_to_opponent()
        self.training_env.env_method('set_behaviors', self.current_behavior)

    def _on_rollout_start(self):
        self._set_behaviors()

    def _try_update_scores(self):
        dones = self.locals["dones"]
        last_games_scores = self.training_env.get_attr("last_game_score")

        for i in range(len(dones)):
            if dones[i]:
                last_score = last_games_scores[i]

                if last_score is not None:
                    self.scores.append(last_score)

    def _on_step(self) -> bool:
        if any(self.locals["dones"]):
            self._try_update_scores()
            self._try_save_model()

            if len(self.scores) == self.scores.maxlen and\
                    (self.scores / self.number_games > self.threshold):
                self.scores.clear()

                if self._any_over_behavior():
                    if self.current_task + 1 < len(self.behaviors):
                        self._set_next_task()
                    else:
                        return False
                else:
                    self._update_behaviors()

        return True
