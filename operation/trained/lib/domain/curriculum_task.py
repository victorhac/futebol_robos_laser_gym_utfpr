from lib.domain.ball_curriculum_behavior import BallCurriculumBehavior
from lib.domain.robot_curriculum_behavior import RobotCurriculumBehavior
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum

class CurriculumTask:
    def __init__(
        self,
        behaviors: list[RobotCurriculumBehavior],
        ball_behavior: BallCurriculumBehavior
    ):
        self.behaviors = behaviors
        self.ball_behavior = ball_behavior

    def update(self):
        for item in self.behaviors:
            item.update()

        self.ball_behavior.update()

    def reset(self):
        for item in self.behaviors:
            item.reset()

        self.ball_behavior.reset()

    def set_opponent_model_path(self, model_path: str):
        yellow_behaviors = self.get_yellow_behaviors()

        for item in yellow_behaviors:
            if item.robot_curriculum_behavior_enum == RobotCurriculumBehaviorEnum.FROM_MODEL:
                item.set_model_path(model_path)

    def get_yellow_behaviors(self) -> list[RobotCurriculumBehavior]:
        return filter(lambda item: item.is_yellow, self.behaviors)
    
    def get_blue_behaviors(self) -> list[RobotCurriculumBehavior]:
        return filter(lambda item: not item.is_yellow, self.behaviors)
    
    def get_yellow_behaviors_by_robot_id(self, robot_id: int):
        return next((item for item in self.get_yellow_behaviors() if item.robot_id == robot_id), None)

    def get_blue_behaviors_by_robot_id(self, robot_id: int):
        return next((item for item in self.get_blue_behaviors() if item.robot_id == robot_id), None)

    def all_over_behavior(self):
        return all(item.is_over() for item in self.behaviors)