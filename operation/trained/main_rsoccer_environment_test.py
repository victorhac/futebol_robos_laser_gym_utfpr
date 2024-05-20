from lib.helpers.configuration_helper import ConfigurationHelper
from lib.environment.goalkeeper_v2.environment import Environment

from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils

IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
FIELD_LENGTH = ConfigurationHelper.getFieldLength()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()
ROBOT_SPEED_BASE = ConfigurationHelper.getRobotSpeedBase()

env = Environment()

try:
    for i in range(100):
        obs = env.reset()
        reward = 0
        done = False
        fator = 5
        error = 0
        left_motor_speed, right_motor_speed = 0, 0

        try:
            while done is False:
                next_state, reward, done, _ = env.step((left_motor_speed / ROBOT_SPEED_BASE, right_motor_speed / ROBOT_SPEED_BASE))
                env.render()
        except KeyboardInterrupt:
            pass
        finally:
            env.reset()

        input("Press Enter to continue...")
except KeyboardInterrupt:
    env.reset()