from lib.helpers.configuration_helper import ConfigurationHelper
from lib.environment.attacker.environment import Environment

from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils

IS_YELLOW_TEAM = ConfigurationHelper.get_rsoccer_team_is_yellow_team()
FIELD_LENGTH = ConfigurationHelper.get_field_length()
IS_LEFT_TEAM = ConfigurationHelper.get_rsoccer_is_left_team()
ROBOT_SPEED_BASE = ConfigurationHelper.get_rsoccer_robot_speed_max_radians_seconds()

env = Environment()

try:
    for i in range(100):
        obs = env.reset()
        reward = 0
        done = False
        left_motor, right_motor, error = 0,0,0
        max_speed = 30

        try:
            while done is False:
                next_state, reward, done, _, _ = env.step((left_motor / max_speed, right_motor / max_speed))
                field_data = RSoccerHelper.get_field_data(env.get_frame(), IS_YELLOW_TEAM)
                ball_position = (field_data.ball.position.x, field_data.ball.position.y)
                left_motor, right_motor, error = MotionUtils.go_to_point(field_data.robots[0], ball_position, error)
                env.render()
        except KeyboardInterrupt:
            env.reset()

        input("Press Enter to continue...")
        env.reset()
        
except KeyboardInterrupt:
    env.reset()