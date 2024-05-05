from lib.helpers.configuration_helper import ConfigurationHelper
from lib.environment.goalkeeper.environment import Environment
import time

IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
FIELD_LENGTH = ConfigurationHelper.getFieldLength()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()

env = Environment()

for i in range(10):
    env.reset()
    done = False


    obs = env.reset()
    reward = 0
    done = False

    while done is False:
        goalReferencePosition = env._get_goal_target_position()
        obs, reward, done, info = env.step(goalReferencePosition)
        env.render()

    input("Press Enter to continue...")

    env.reset()