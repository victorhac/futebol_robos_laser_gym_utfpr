from lib.domain.robot_curriculum_behavior import RobotCurriculumBehavior
from lib.enums.position_enum import PositionEnum
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum
from lib.utils.configuration_utils import ConfigurationUtils
from lib.environment.attacker.environment import Environment

from lib.utils.rsoccer_utils import RSoccerUtils
from lib.motion.motion_utils import MotionUtils

IS_YELLOW_TEAM = ConfigurationUtils.get_rsoccer_team_is_yellow_team()
FIELD_LENGTH = ConfigurationUtils.get_field_length()
IS_LEFT_TEAM = ConfigurationUtils.get_rsoccer_is_left_team()
ROBOT_SPEED_BASE = ConfigurationUtils.get_rsoccer_robot_speed_max_radians_seconds()

env = Environment("human")

behaviors = {
    "blue": [
        RobotCurriculumBehavior(RobotCurriculumBehaviorEnum.STOPPED, 0, False, PositionEnum.GOAL_AREA),
        RobotCurriculumBehavior(RobotCurriculumBehaviorEnum.STOPPED, 1, False, PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA),
        RobotCurriculumBehavior(
            RobotCurriculumBehaviorEnum.BALL_FOLLOWING,
            2,
            False,
            PositionEnum.OPPONENT_AREA_EXCEPT_GOAL_AREA,
            (30, 50),
            30,
            0.001
        )
    ],
    "yellow": [
        RobotCurriculumBehavior(RobotCurriculumBehaviorEnum.STOPPED, 0, True, PositionEnum.GOAL_AREA),
        RobotCurriculumBehavior(RobotCurriculumBehaviorEnum.STOPPED, 1, True, PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA),
        RobotCurriculumBehavior(RobotCurriculumBehaviorEnum.STOPPED, 2, True, PositionEnum.OWN_AREA_EXCEPT_GOAL_AREA)
    ]
}

env.set_behaviors(behaviors)

try:
    for i in range(100):
        obs = env.reset()
        reward = 0
        done = False

        try:
            while done is False:
                next_state, reward, done, _, _ = env.step((0, 0))
                env.render()
        except KeyboardInterrupt:
            env.reset()

        #input("Press Enter to continue...")
        env.reset()
        
except KeyboardInterrupt:
    env.reset()