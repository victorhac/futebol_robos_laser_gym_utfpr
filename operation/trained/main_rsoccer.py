from lib.helpers.configuration_helper import ConfigurationHelper
from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils
from lib.environment.rsoccer_validation.environment import Environment
from lib.helpers.field_helper import FieldHelper
import time

IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
FIELD_LENGTH = ConfigurationHelper.getFieldLength()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()

env = Environment()

ownGoalPosition = FieldHelper.getOwnGoalPosition(FIELD_LENGTH, IS_LEFT_TEAM)

for i in range(20):
    episodeInitialTime = time.time()
    env.reset()

    done = False
    error = 0
    leftSpeed, rightSpeed = 0, 0

    fator = 5

    while not done and time.time() - episodeInitialTime < 5:
        action = [RSoccerHelper.getRSoccerRobotAction(0, not IS_YELLOW_TEAM, leftSpeed, rightSpeed)]
        next_state, reward, done, _ = env.step(action)

        fieldData, opponentFieldData = RSoccerHelper.getFieldDatas(next_state, IS_YELLOW_TEAM)

        robot = opponentFieldData.robots[0]

        velocities = MotionUtils.goToPoint(robot, ownGoalPosition, error)

        (leftSpeed, rightSpeed, _) = velocities

        env.render()