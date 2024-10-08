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

for i in range(1):
    episodeInitialTime = time.time()
    env.reset()

    goalReferencePosition = env.get_goal_reference_position()

    done = False
    error = 0
    error2 = 0
    leftSpeed, rightSpeed = 0, 0
    leftSpeed2, rightSpeed2 = 0, 0

    fator = 5

    while not done and time.time() - episodeInitialTime < 20:
        action = [
            RSoccerHelper.getRSoccerRobotAction(0, not IS_YELLOW_TEAM, leftSpeed / fator, rightSpeed / fator),
            RSoccerHelper.getRSoccerRobotAction(0, IS_YELLOW_TEAM, leftSpeed2 / fator, rightSpeed2 / fator)
        ]

        next_state, reward, done, _ = env.step(action)

        fieldData, opponentFieldData = RSoccerHelper.getFieldDatas(next_state, IS_YELLOW_TEAM)

        robot = opponentFieldData.robots[0]

        velocities = MotionUtils.goToPoint(robot, goalReferencePosition, error)

        (leftSpeed, rightSpeed, _) = velocities

        robot = fieldData.robots[0]

        reference = env._get_goal_target_position()

        velocities = MotionUtils.goToPoint(robot, (reference[0], reference[1]), error2)

        (leftSpeed2, rightSpeed2, _) = velocities

        env.render()