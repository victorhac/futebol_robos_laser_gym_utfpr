from lib.helpers.configuration_helper import ConfigurationHelper
from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils
from lib.environment.rsoccer_validation.environment import Environment

IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()

env = Environment()

env.reset()

for i in range(1):
    done = False
    error = 0
    leftSpeed, rightSpeed = 0, 0

    fator = 5

    while not done:
        action = [RSoccerHelper.getRSoccerRobotAction(0, IS_YELLOW_TEAM, leftSpeed / fator, rightSpeed / fator)]
        next_state, reward, done, _ = env.step(action)

        fieldData, opponentFieldData = RSoccerHelper.getFieldDatas(next_state, IS_YELLOW_TEAM)

        ball = fieldData.ball
        robot = fieldData.robots[0]

        targetPosition = (ball.position.x, ball.position.y)

        velocities = MotionUtils.goToPoint(robot, targetPosition, error)

        (leftSpeed, rightSpeed, _) = velocities

        env.render()