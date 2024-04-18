from lib.helpers.configuration_helper import ConfigurationHelper
from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils
from lib.environment.environment2 import Environment2

CONFIGURATION = ConfigurationHelper.getConfiguration()

IS_YELLOW_TEAM = CONFIGURATION["team"]["is-yellow-team"]

env = Environment2()

env.reset()

for i in range(1):
    done = False
    error = 0
    leftSpeed, rightSpeed = 0, 0

    while not done:
        action = [RSoccerHelper.getRSoccerRobotAction(0, IS_YELLOW_TEAM, 1 / 20, -1 / 20)]
        next_state, reward, done, _ = env.step(action)

        fieldData, opponentFieldData = RSoccerHelper.getFieldDatas(next_state, IS_YELLOW_TEAM)

        ball = fieldData.ball
        robot = fieldData.robots[0]

        targetPosition = (ball.position.x, ball.position.y)

        tangentPoint = MotionUtils.findTangentPointObstacle(0, fieldData, opponentFieldData, targetPosition)

        if tangentPoint is not None:
            targetPosition = tangentPoint

        velocities = MotionUtils.goToPoint(robot, targetPosition, error)

        (leftSpeed, rightSpeed, _) = velocities

        env.render()