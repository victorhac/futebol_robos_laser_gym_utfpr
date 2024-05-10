from lib.helpers.configuration_helper import ConfigurationHelper
from lib.environment.attacker.environment import Environment
import time

from lib.motion.motion_utils import MotionUtils

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

    leftSpeed, rightSpeed = 0, 0
    error = 0

    while done is False:
        field_data, _ = env._get_field_datas()
        robot = field_data.robots[0]
        ball = field_data.ball

        (leftSpeed, rightSpeed, error) = MotionUtils.goToPoint(robot, ball.get_position_tuple(), error)

        obs, reward, done, info = env.step((leftSpeed / 30, rightSpeed / 30))

        env.render()

    input("Press Enter to continue...")

    env.reset()