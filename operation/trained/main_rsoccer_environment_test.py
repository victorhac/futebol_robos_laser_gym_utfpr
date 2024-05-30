from lib.helpers.configuration_helper import ConfigurationHelper
from lib.environment.defensor.environment import Environment

from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils

IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
FIELD_LENGTH = ConfigurationHelper.getFieldLength()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()
ROBOT_SPEED_BASE = ConfigurationHelper.getRobotSpeedBase()

env = Environment()

try:
    for i in range(100):
        obs = env.reset()
        reward = 0
        done = False
        fator = 1
        error = 0
        left_motor_speed, right_motor_speed = 0, 0

        try:
            while done is False:
                next_state, reward, done, _ = env.step((left_motor_speed / ROBOT_SPEED_BASE, right_motor_speed / ROBOT_SPEED_BASE))

                print("Reward: ", reward)

                field_data, _ = RSoccerHelper.get_field_datas(env.get_state(), IS_YELLOW_TEAM)
                robot = field_data.robots[0]
                ball = field_data.ball

                left_motor_speed, right_motor_speed, error = MotionUtils.go_to_point(
                    robot,
                    ball.get_position_tuple(),
                    error
                )

                env.render()
        except KeyboardInterrupt:
            env.reset()

        input("Press Enter to continue...")
        env.reset()
        
except KeyboardInterrupt:
    env.reset()