from lib.helpers.configuration_helper import ConfigurationHelper
from lib.environment.attacker.environment import Environment
import time

from lib.motion.motion_utils import MotionUtils

IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
FIELD_LENGTH = ConfigurationHelper.getFieldLength()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()

env = Environment()

for i in range(10):
    obs = env.reset()
    reward = 0
    done = False

    while done is False:
        field_data, _ = env._get_field_datas()
        ball = field_data.ball

        obs, reward, done, info = env.step(ball.get_position_tuple())

        env.render()

    input("Press Enter to continue...")

    env.reset()