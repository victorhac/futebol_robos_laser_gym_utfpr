from lib.domain.robot_curriculum_behavior import RobotCurriculumBehavior
from lib.enums.position_enum import PositionEnum
from lib.enums.robot_curriculum_behavior_enum import RobotCurriculumBehaviorEnum
from lib.utils.behavior.behavior_utils import BehaviorUtils
from lib.utils.configuration_utils import ConfigurationUtils
from lib.environment.attacker.environment import Environment

IS_YELLOW_TEAM = ConfigurationUtils.get_rsoccer_team_is_yellow_team()
FIELD_LENGTH = ConfigurationUtils.get_field_length()
IS_LEFT_TEAM = ConfigurationUtils.get_rsoccer_is_left_team()
ROBOT_SPEED_BASE = ConfigurationUtils.get_rsoccer_robot_speed_max_radians_seconds()

env = Environment("human")

behaviors = BehaviorUtils.get_task_1(10)

env.set_task(behaviors)

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