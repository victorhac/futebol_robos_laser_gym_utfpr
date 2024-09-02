from lib.domain.robot_curriculum_behavior import RobotCurriculumBehavior
from lib.enums.position_enum import PositionEnum
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum

class BehaviorUtils:
    def get_stopped_behavior(
        id: int,
        is_yellow: bool,
        position_enum: PositionEnum
    ):
        return RobotCurriculumBehavior(
            RobotCurriculumBehaviorEnum.STOPPED,
            id,
            is_yellow,
            position_enum
        ) 
    
    def get_from_model_behavior(
        id: int,
        position_enum: PositionEnum
    ):
        return RobotCurriculumBehavior(
            RobotCurriculumBehaviorEnum.FROM_MODEL,
            id,
            False,
            position_enum,
            distance_range=[10,50],
            start_distance=10,
            distance_beta=0.0004
        )
    
    def get_opponent_from_model_behavior(
        id: int,
        position_enum: PositionEnum
    ):
        return RobotCurriculumBehavior(
            RobotCurriculumBehaviorEnum.STOPPED,
            id,
            True,
            position_enum,
            distance_range=[30,50],
            start_distance=50,
            distance_beta=-0.0002
        )
    
    def get_ball_following_behavior(
        id: int,
        is_yellow: bool,
        position_enum: PositionEnum,
        distance_range: list[float],
        start_distance: float,
        distance_beta: float,
        velocity_beta: float,
        start_velocity_alpha: float
    ):
        return RobotCurriculumBehavior(
            RobotCurriculumBehaviorEnum.BALL_FOLLOWING,
            id,
            is_yellow,
            position_enum,
            distance_range,
            start_distance,
            distance_beta,
            velocity_beta,
            start_velocity_alpha
        )
    
    def get_default_yellow_team_ball_following_behavior(id: int):
        return BehaviorUtils.get_ball_following_behavior(
            id=id,
            is_yellow=True,
            position_enum=PositionEnum.RELATIVE_TO_BALL,
            distance_range=[0.3, 0.5],
            start_distance=0.5,
            distance_beta=-0.0002,
            velocity_beta=0.001,
            start_velocity_alpha=0)
    
    def get_task_return(blue_behaviors, yellow_behaviors) -> dict[str, list[RobotCurriculumBehavior]]:
        return {
            "blue": blue_behaviors,
            "yellow": yellow_behaviors
        }


    def get_task_1_behaviors(number_robot_blue, number_robot_yellow):
        blue_behaviors = []
        yellow_behaviors = []

        for i in range(number_robot_blue):
            if i == 0:
                blue_behaviors.append(BehaviorUtils.get_from_model_behavior(i, PositionEnum.RELATIVE_TO_BALL))
            elif i == 1:
                blue_behaviors.append(BehaviorUtils.get_stopped_behavior(i, False, PositionEnum.GOAL_AREA))
            else:
                blue_behaviors.append(BehaviorUtils.get_stopped_behavior(i, False, PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA))

        for i in range(number_robot_yellow):
            if i == 0:
                yellow_behaviors.append(BehaviorUtils.get_stopped_behavior(i, True, PositionEnum.GOAL_AREA))
            elif i == 1:
                yellow_behaviors.append(BehaviorUtils.get_default_yellow_team_ball_following_behavior(i))
            else:
                yellow_behaviors.append(BehaviorUtils.get_stopped_behavior(i, True, PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA))

        return BehaviorUtils.get_task_return(blue_behaviors, yellow_behaviors)
    
    def get_task_2_behaviors(number_robot_blue, number_robot_yellow):
        blue_behaviors = []
        yellow_behaviors = []

        for i in range(number_robot_blue):
            if i == 0:
                blue_behaviors.append(BehaviorUtils.get_from_model_behavior(i, PositionEnum.RELATIVE_TO_BALL))
            elif i == 1:
                blue_behaviors.append(BehaviorUtils.get_stopped_behavior(i, False, PositionEnum.GOAL_AREA))
            else:
                blue_behaviors.append(BehaviorUtils.get_stopped_behavior(i, False, PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA))

        for i in range(number_robot_yellow):
            if i == 0:
                yellow_behaviors.append(BehaviorUtils.get_stopped_behavior(i, True, PositionEnum.GOAL_AREA))
            elif i == 1:
                yellow_behaviors.append(BehaviorUtils.get_opponent_from_model_behavior(i, PositionEnum.RELATIVE_TO_BALL))
            else:
                yellow_behaviors.append(BehaviorUtils.get_stopped_behavior(i, True, PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA))

        return BehaviorUtils.get_task_return(blue_behaviors, yellow_behaviors)
    
    def get_task_3_behaviors(number_robot_blue, number_robot_yellow):
        blue_behaviors = []
        yellow_behaviors = []

        for i in range(number_robot_blue):
            if i == 0:
                blue_behaviors.append(BehaviorUtils.get_from_model_behavior(i, PositionEnum.RELATIVE_TO_BALL))
            elif i == 1:
                blue_behaviors.append(BehaviorUtils.get_stopped_behavior(i, False, PositionEnum.GOAL_AREA))
            else:
                blue_behaviors.append(BehaviorUtils.get_stopped_behavior(i, False, PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA))

        for i in range(number_robot_yellow):
            if i == 0:
                yellow_behaviors.append(BehaviorUtils.get_stopped_behavior(i, True, PositionEnum.GOAL_AREA))
            elif i == 1:
                yellow_behaviors.append(BehaviorUtils.get_opponent_from_model_behavior(i, PositionEnum.RELATIVE_TO_BALL))
            else:
                yellow_behaviors.append(BehaviorUtils.get_default_yellow_team_ball_following_behavior(i))

        return BehaviorUtils.get_task_return(blue_behaviors, yellow_behaviors)
    
    def get_task_4_behaviors(number_robot_blue, number_robot_yellow):
        blue_behaviors = []
        yellow_behaviors = []

        for i in range(number_robot_blue):
            if i == 0:
                blue_behaviors.append(BehaviorUtils.get_from_model_behavior(i, PositionEnum.RELATIVE_TO_BALL))
            elif i == 1:
                blue_behaviors.append(BehaviorUtils.get_stopped_behavior(i, False, PositionEnum.GOAL_AREA))
            else:
                blue_behaviors.append(BehaviorUtils.get_stopped_behavior(i, False, PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA))

        for i in range(number_robot_yellow):
            if i == 0:
                yellow_behaviors.append(BehaviorUtils.get_opponent_from_model_behavior(i, PositionEnum.GOAL_AREA))
            else:
                yellow_behaviors.append(BehaviorUtils.get_default_yellow_team_ball_following_behavior(i))

        return BehaviorUtils.get_task_return(blue_behaviors, yellow_behaviors)