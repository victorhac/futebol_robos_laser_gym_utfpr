from lib.helpers.configuration_helper import ConfigurationHelper
from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils
from lib.environment.rsoccer_validation.teste import Environment
import time

IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
FIELD_LENGTH = ConfigurationHelper.getFieldLength()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()

ROBOT_SPEED_BASE = ConfigurationHelper.getRobotSpeedBase()

env = Environment()

for i in range(1):
    env.reset()

    done = False
    error = 0

    left_motor_speed, right_motor_speed, error = (0, 0, 0)

    fator = 5

    while not done:
        next_state, reward, done, _ = env.step((left_motor_speed / ROBOT_SPEED_BASE / fator, right_motor_speed / ROBOT_SPEED_BASE / fator))

        fieldData, opponentFieldData = RSoccerHelper.get_field_datas(next_state, IS_YELLOW_TEAM)

        left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(
            fieldData.robots[0],
            fieldData.ball.get_position_tuple(),
            error)

        env.render()