from lib.helpers.configuration_helper import ConfigurationHelper
from lib.environment.goalkeeper.environment import Environment

from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils

IS_YELLOW_TEAM = ConfigurationHelper.get_rsoccer_team_is_yellow_team()
FIELD_LENGTH = ConfigurationHelper.get_field_length()
IS_LEFT_TEAM = ConfigurationHelper.get_rsoccer_is_left_team()
ROBOT_SPEED_BASE = ConfigurationHelper.get_rsoccer_robot_speed_max_radians_seconds()

env = Environment()

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