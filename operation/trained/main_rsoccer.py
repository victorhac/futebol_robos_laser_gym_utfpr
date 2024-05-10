from lib.helpers.configuration_helper import ConfigurationHelper
from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils
from lib.environment.rsoccer_validation.environment import Environment
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
    leftSpeed, rightSpeed = 0, 0

    fator = 1 * ROBOT_SPEED_BASE * 10

    while not done:
        action = [leftSpeed / fator, rightSpeed / fator]

        next_state, reward, done, _ = env.step(action)

        fieldData, opponentFieldData = RSoccerHelper.getFieldDatas(next_state, IS_YELLOW_TEAM)

        robot = fieldData.robots[0]
        ball = fieldData.ball

        (leftSpeed, rightSpeed, error) = MotionUtils.goToPoint(robot, ball.get_position_tuple(), error)

        env.render()