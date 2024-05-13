from lib.helpers.configuration_helper import ConfigurationHelper
from lib.environment.attacker_v2.environment import Environment
import time

from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils

IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
FIELD_LENGTH = ConfigurationHelper.getFieldLength()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()
ROBOT_SPEED_BASE = ConfigurationHelper.getRobotSpeedBase()

env = Environment()

for i in range(10):
    obs = env.reset()
    reward = 0
    done = False
    fator = 5
    error = 0
    left_motor_speed, right_motor_speed = 0, 0

    while done is False:
        next_state, reward, done, _ = env.step((left_motor_speed / ROBOT_SPEED_BASE, right_motor_speed / ROBOT_SPEED_BASE))

        print("Reward: ", reward)

        fieldData, opponentFieldData = RSoccerHelper.getFieldDatas(env._get_state(), IS_YELLOW_TEAM)

        left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(
            fieldData.robots[0],
            fieldData.ball.get_position_tuple(),
            error)

        env.render()

    input("Press Enter to continue...")

    env.reset()