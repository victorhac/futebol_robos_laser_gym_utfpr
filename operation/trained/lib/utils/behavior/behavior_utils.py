from lib.domain.ball_curriculum_behavior import BallCurriculumBehavior
from lib.domain.curriculum_task import CurriculumTask
from lib.domain.robot_curriculum_behavior import RobotCurriculumBehavior
from lib.enums.position_enum import PositionEnum
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum

class BehaviorUtils:
    @staticmethod
    def get_stopped_behavior(
        robot_id: int,
        is_yellow: bool,
        position_enum: PositionEnum,
        updates_per_task: int
    ):
        return RobotCurriculumBehavior(
            RobotCurriculumBehaviorEnum.STOPPED,
            robot_id,
            is_yellow,
            position_enum,
            updates_per_task=updates_per_task)
    
    @staticmethod
    def get_from_model_behavior(
        robot_id: int,
        position_enum: PositionEnum,
        updates_per_task: int
    ):
        return RobotCurriculumBehavior(
            RobotCurriculumBehaviorEnum.FROM_MODEL,
            robot_id,
            False,
            position_enum,
            distance_range=(.2, .5),
            updates_per_task=updates_per_task)
    
    @staticmethod
    def get_opponent_from_model_behavior(
        robot_id: int,
        position_enum: PositionEnum,
        updates_per_task: int
    ):
        return RobotCurriculumBehavior(
            RobotCurriculumBehaviorEnum.FROM_MODEL,
            robot_id,
            True,
            position_enum,
            distance_range=(.3, .6),
            is_positive_distance_beta=True,
            updates_per_task=updates_per_task)
    
    @staticmethod
    def get_ball_following_behavior(
        robot_id: int,
        is_yellow: bool,
        position_enum: PositionEnum,
        is_positive_distance_beta: bool,
        distance_range: 'tuple[float, float]',
        is_positive_velocity_beta: bool,
        start_velocity_alpha: float,
        updates_per_task: int
    ):
        return RobotCurriculumBehavior(
            RobotCurriculumBehaviorEnum.BALL_FOLLOWING,
            robot_id,
            is_yellow,
            position_enum,
            is_positive_distance_beta=is_positive_distance_beta,
            distance_range=distance_range,
            is_positive_velocity_beta=is_positive_velocity_beta,
            start_velocity_alpha=start_velocity_alpha,
            updates_per_task=updates_per_task)
    
    @staticmethod
    def get_default_opponent_ball_following_behavior(
        robot_id: int,
        updates_per_task: int
    ):
        return BehaviorUtils.get_ball_following_behavior(
            robot_id=robot_id,
            is_yellow=True,
            position_enum=PositionEnum.RELATIVE_TO_BALL,
            is_positive_distance_beta=True,
            distance_range=(.3, .6),
            is_positive_velocity_beta=True,
            start_velocity_alpha=0,
            updates_per_task=updates_per_task)
    
    @staticmethod
    def get_ball_behavior(
        position_enum: PositionEnum,
        updates_per_task: int = 10,
        is_positive_distance_beta: bool = True,
        distance_range: 'tuple[float, float]' | None = None
    ):
        return BallCurriculumBehavior(
            position_enum=position_enum,
            updates_per_task=updates_per_task,
            is_positive_distance_beta=is_positive_distance_beta,
            distance_range=distance_range)
    
    @staticmethod
    def get_task_1(updates_per_task: int):
        behaviors = [
            BehaviorUtils.get_from_model_behavior(
                0,
                PositionEnum.RELATIVE_TO_BALL,
                updates_per_task)
        ]

        ball_behavior = BehaviorUtils.get_ball_behavior(
            PositionEnum.RELATIVE_TO_OPPONENT_GOAL,
            updates_per_task,
            is_positive_distance_beta=True,
            distance_range=(.2, 1.3)
        )

        return CurriculumTask(behaviors, ball_behavior)
    
    @staticmethod
    def get_task_2(updates_per_task: int):
        behaviors = [
            BehaviorUtils.get_from_model_behavior(
                0,
                PositionEnum.RELATIVE_TO_BALL,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                0,
                True,
                PositionEnum.OWN_AREA,
                updates_per_task)
        ]
    
        ball_behavior = BehaviorUtils.get_ball_behavior(
            PositionEnum.OWN_AREA,
            updates_per_task
        )

        return CurriculumTask(behaviors, ball_behavior)
    
    @staticmethod
    def get_task_3(updates_per_task: int):
        behaviors = [
            BehaviorUtils.get_from_model_behavior(
                0,
                PositionEnum.RELATIVE_TO_BALL,
                updates_per_task),
            BehaviorUtils.get_default_opponent_ball_following_behavior(
                0,
                updates_per_task)
        ]
    
        ball_behavior = BehaviorUtils.get_ball_behavior(
            PositionEnum.OWN_AREA,
            updates_per_task
        )

        return CurriculumTask(behaviors, ball_behavior)
    
    @staticmethod
    def get_task_4(updates_per_task: int):
        behaviors = [
            BehaviorUtils.get_from_model_behavior(
                0,
                PositionEnum.RELATIVE_TO_BALL,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                1,
                False,
                PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA,
                updates_per_task),
            BehaviorUtils.get_default_opponent_ball_following_behavior(
                0,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                1,
                True,
                PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA,
                updates_per_task)
        ]
    
        ball_behavior = BehaviorUtils.get_ball_behavior(
            PositionEnum.OWN_AREA,
            updates_per_task
        )

        return CurriculumTask(behaviors, ball_behavior)

    @staticmethod
    def get_task_5(updates_per_task: int):
        behaviors = [
            BehaviorUtils.get_from_model_behavior(
                0,
                PositionEnum.RELATIVE_TO_BALL,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                1,
                False,
                PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                2,
                False,
                PositionEnum.GOAL_AREA,
                updates_per_task),
            BehaviorUtils.get_default_opponent_ball_following_behavior(
                0,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                1,
                True,
                PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                2,
                True,
                PositionEnum.GOAL_AREA,
                updates_per_task)
        ]
    
        ball_behavior = BehaviorUtils.get_ball_behavior(
            PositionEnum.OWN_AREA,
            updates_per_task
        )

        return CurriculumTask(behaviors, ball_behavior)
    
    @staticmethod
    def get_task_6(updates_per_task: int):
        behaviors = [
            BehaviorUtils.get_from_model_behavior(
                0,
                PositionEnum.RELATIVE_TO_BALL,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                1,
                False,
                PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                2,
                False,
                PositionEnum.GOAL_AREA,
                updates_per_task),
            BehaviorUtils.get_opponent_from_model_behavior(
                0,
                PositionEnum.RELATIVE_TO_BALL,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                1,
                True,
                PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                2,
                True,
                PositionEnum.GOAL_AREA,
                updates_per_task),
        ]
    
        ball_behavior = BehaviorUtils.get_ball_behavior(
            PositionEnum.OWN_AREA,
            updates_per_task
        )

        return CurriculumTask(behaviors, ball_behavior)
    
    @staticmethod
    def get_task_7(updates_per_task: int):
        behaviors = [
            BehaviorUtils.get_from_model_behavior(
                0,
                PositionEnum.RELATIVE_TO_BALL,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                1,
                False,
                PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                2,
                False,
                PositionEnum.GOAL_AREA,
                updates_per_task),
            BehaviorUtils.get_opponent_from_model_behavior(
                0,
                PositionEnum.RELATIVE_TO_BALL,
                updates_per_task),
            BehaviorUtils.get_default_opponent_ball_following_behavior(
                1,
                updates_per_task),
            BehaviorUtils.get_stopped_behavior(
                2,
                True,
                PositionEnum.GOAL_AREA,
                updates_per_task)
        ]
    
        ball_behavior = BehaviorUtils.get_ball_behavior(
            PositionEnum.OWN_AREA,
            updates_per_task
        )

        return CurriculumTask(behaviors, ball_behavior)