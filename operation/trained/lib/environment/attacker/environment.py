import math
import random
import time
import numpy as np

from gymnasium.spaces import Box

from rsoccer_gym.Utils.Utils import OrnsteinUhlenbeckAction
from rsoccer_gym.Entities import Frame, Robot, Ball
from rsoccer_gym.Utils import KDTree

from ...helpers.rsoccer_helper import RSoccerHelper
from ...helpers.field_helper import FieldHelper
from ...helpers.configuration_helper import ConfigurationHelper

from ...environment.base_environment import BaseEnvironment

TRAINING_EPISODE_DURATION = ConfigurationHelper.get_rsoccer_training_episode_duration()

NUMBER_ROBOTS_BLUE = ConfigurationHelper.get_rsoccer_team_blue_number_robots()
NUMBER_ROBOTS_YELLOW = ConfigurationHelper.get_rsoccer_team_yellow_number_robots()

V_WHEEL_DEADZONE = ConfigurationHelper.get_rsoccer_robot_speed_dead_zone_meters_seconds()

TIME_STEP = ConfigurationHelper.get_rsoccer_training_time_step_seconds()

# addapt this for your robot
MAX_MOTOR_SPEED = ConfigurationHelper.get_firasim_robot_speed_max_radians_seconds()

class Environment(BaseEnvironment):
    def __init__(self, render_mode="rgb_array"):
        super().__init__(
            field_type=0,
            n_robots_blue=NUMBER_ROBOTS_BLUE,
            n_robots_yellow=NUMBER_ROBOTS_YELLOW,
            time_step=TIME_STEP,
            render_mode=render_mode)
        
        self.max_motor_speed = MAX_MOTOR_SPEED

        self.action_space = Box(
            low=-1,
            high=1,
            shape=(2,),
            dtype=np.float32)

        self.observation_space = Box(
            low=-1,
            high=1,
            shape=(34,),
            dtype=np.float32)

        self.previous_ball_potential = None
        self.episode_initial_time = 0

        self.training_episode_duration = TRAINING_EPISODE_DURATION
        self.v_wheel_deadzone = V_WHEEL_DEADZONE

        self._set_ou_actions()

    def _set_ou_actions(self):
        self.ou_actions = [
            OrnsteinUhlenbeckAction(self.action_space, dt=self.time_step) \
            for _ in range(self.n_robots_blue + self.n_robots_yellow)
        ]
    
    def _is_done(self):
        if self._any_team_scored_goal():
            return True
        elif time.time() - self.episode_initial_time > TRAINING_EPISODE_DURATION:
            return True
        return False
    
    def _create_robot(
        self,
        id: int,
        is_yellow_robot: bool,
        v_wheel_0: float,
        v_wheel_1: float
    ):
        return Robot(
            yellow=is_yellow_robot,
            id=id,
            v_wheel0=v_wheel_0,
            v_wheel1=v_wheel_1)
    
    def _get_ou_actions(self, is_yellow_robot: bool, robot_id: int):
        if is_yellow_robot:
            ou_action_id = self.n_robots_blue + robot_id
        else:
            ou_action_id = robot_id

        return self.ou_actions[ou_action_id].sample()
    
    def _create_robot_with_ou_action(
        self,
        robot_id: int,
        is_yellow_robot: bool
    ):
        actions = self._get_ou_actions(is_yellow_robot, robot_id)
        v_wheel0, v_wheel1 = self._actions_to_v_wheels(
            actions,
            not is_yellow_robot)

        return self._create_robot(
            robot_id,
            is_yellow_robot,
            v_wheel0,
            v_wheel1)
    
    def _frame_to_observations(self):
        observation = []

        ball = self.get_ball()

        observation.extend([
            self.norm_x(ball.x),
            self.norm_y(-ball.y),
            self.norm_v(ball.v_x),
            self.norm_v(-ball.v_y)
        ])

        frame = self.frame

        for i in range(self.n_robots_blue):
            robot = frame.robots_blue[i]
            theta = -RSoccerHelper.get_corrected_angle(robot.theta)

            observation.extend([
                self.norm_x(robot.x),
                self.norm_y(-robot.y),
                theta / np.pi,
                self.norm_v(robot.v_x),
                self.norm_v(-robot.v_y)
            ])

        for i in range(self.n_robots_yellow):
            robot = frame.robots_yellow[i]
            theta = -RSoccerHelper.get_corrected_angle(robot.theta)

            observation.extend([
                self.norm_x(robot.x),
                self.norm_y(-robot.y),
                theta / np.pi,
                self.norm_v(robot.v_x),
                self.norm_v(-robot.v_y)
            ])

        return np.array(observation, dtype=np.float32)
        
    def _get_commands(self, actions):
        commands = []

        v_wheel0, v_wheel1 = self._actions_to_v_wheels(actions, True)

        robot = self._create_robot(0, False, v_wheel0, v_wheel1)
        
        commands.append(robot)
            
        for i in range(1, self.n_robots_blue):
            robot = self._create_robot_with_ou_action(i, False)
            commands.append(robot)

        for i in range(self.n_robots_yellow):
            robot = self._create_robot_with_ou_action(i, True)
            commands.append(robot)

        return commands

    def _actions_to_v_wheels(
        self,
        actions: np.ndarray,
        is_own_team: bool
    ):
        left_wheel_speed = actions[0] * self.max_v
        right_wheel_speed = actions[1] * self.max_v

        left_wheel_speed, right_wheel_speed = np.clip(
            (left_wheel_speed, right_wheel_speed),
            -self.max_v,
            self.max_v)
        
        if is_own_team:
            rsoccer_max_motor_speed = (self.max_v / self.field.rbt_wheel_radius)
            factor = self.max_motor_speed / rsoccer_max_motor_speed
        else:
            factor = 1

        left_wheel_speed *= factor
        right_wheel_speed *= factor

        if abs(left_wheel_speed) < self.v_wheel_deadzone:
            left_wheel_speed = 0

        if abs(right_wheel_speed) < self.v_wheel_deadzone:
            right_wheel_speed = 0

        left_wheel_speed /= self.field.rbt_wheel_radius
        right_wheel_speed /= self.field.rbt_wheel_radius

        return left_wheel_speed, right_wheel_speed
    
    def _ball_gradient_reward(
        self,
        previous_ball_potential: float | None
    ):
        field_length = self.get_field_length()
        goal_depth = self.get_goal_depth()
        ball = self.get_ball()

        convert_m_to_cm = lambda x: x * 100

        length_cm = convert_m_to_cm(field_length)
        half_lenght = (field_length / 2) + goal_depth

        dx_d = convert_m_to_cm((half_lenght + ball.x))
        dx_a = convert_m_to_cm((half_lenght - ball.x))
        dy = convert_m_to_cm(ball.y)

        dist_1 = math.sqrt(dx_a ** 2 + 2 * dy ** 2)
        dist_2 = math.sqrt(dx_d ** 2 + 2 * dy ** 2)

        ball_potential = ((-dist_1 + dist_2) / length_cm - 1) / 2

        if previous_ball_potential is not None:
            diff = ball_potential - previous_ball_potential
            reward = np.clip(
                diff * 3 / self.time_step,
                -5.0,
                5.0)
        else:
            reward = 0
        
        return reward, ball_potential
        
    def _move_reward(self):
        ball = self.get_ball()
        robot = self._get_agent()

        ball_position = np.array([ball.x, ball.y])
        robot_position = np.array([robot.x, robot.y])
        
        robot_velocities = np.array([robot.v_x, robot.v_y])
        robot_ball_vector = ball_position - robot_position
        robot_ball_vector = robot_ball_vector / np.linalg.norm(robot_ball_vector)

        move_reward = np.dot(robot_ball_vector, robot_velocities)

        return np.clip(move_reward / 0.4, -5.0, 5.0)
    
    def _energy_penalty(self):
        en_penalty_1 = abs(self.sent_commands[0].v_wheel0)
        en_penalty_2 = abs(self.sent_commands[0].v_wheel1)
        return - (en_penalty_1 + en_penalty_2)
    
    def _any_team_scored_goal(self):
        ball = self.get_ball()
        return abs(ball.x) > (self.get_field_length() / 2)
    
    def _has_received_goal(self):        
        ball = self.get_ball()
        return ball.x < -self.get_field_length() / 2

    def _has_scored_goal(self):
        if not self._any_team_scored_goal():
            return None
        return not self._has_received_goal()
    
    def _get_agent(self):
        return self.frame.robots_blue[0]
    
    def _calculate_reward_and_done(self):
        reward = 0
        w_move = 0.2
        w_ball_grad = 0.8
        w_energy = 2e-4

        if self._any_team_scored_goal():
            if self._has_scored_goal():
                reward = 10
            else:
                reward = -10
        else:
            grad_ball_potential, ball_gradient = \
                self._ball_gradient_reward(self.previous_ball_potential)
            
            self.previous_ball_potential = ball_gradient

            move_reward = self._move_reward()
            energy_penalty = self._energy_penalty()

            reward = w_move * move_reward + \
                w_ball_grad * grad_ball_potential + \
                w_energy * energy_penalty

        return reward, self._is_done()
    
    def _get_random_position_inside_field(self):
        return FieldHelper.get_random_position_inside_field(
            self.get_field_length(),
            self.get_field_width())
    
    def _get_random_position_inside_own_penalty_area(self):
        return FieldHelper.get_random_position_inside_own_penalty_area(
            self.get_field_length(),
            self.get_penalty_length(),
            self.get_penalty_width(),
            True,
            self.get_robot_radius())
    
    def _get_random_position_inside_opponent_penalty_area(self):
        return FieldHelper.get_random_position_inside_own_penalty_area(
            self.get_field_length(),
            self.get_penalty_length(),
            self.get_penalty_width(),
            False,
            self.get_robot_radius())
    
    def _get_random_position_inside_own_area(self):
        return FieldHelper.get_random_position_inside_own_area(
            self.get_field_length(),
            self.get_field_width(),
            True)
    
    def _get_random_position_inside_opponent_area(self):
        return FieldHelper.get_random_position_inside_own_area(
            self.get_field_length(),
            self.get_field_width(),
            False)

    def _get_initial_positions_frame(self):
        def theta(): return random.uniform(0, 360)

        frame: Frame = Frame()

        ball_position = self._get_random_position_inside_opponent_area()

        frame.ball = Ball(x=ball_position[0], y=ball_position[1])

        min_distance = 0.15

        places = KDTree()

        places.insert(ball_position)

        frame.robots_blue[0] = Robot(x=ball_position[0] - 0.1, y=ball_position[1], theta=theta())
            
        def get_position(get_position_fn):
            position = get_position_fn()

            while places.get_nearest(position)[1] < min_distance:
                position = get_position_fn()

            places.insert(position)

            return position
        
        position_fns = [
            self._get_random_position_inside_own_penalty_area,
            self._get_random_position_inside_own_area
        ]

        for i in range(1, self.n_robots_blue):
            position = get_position(position_fns[i - 1])
            frame.robots_blue[i] = Robot(x=position[0], y=position[1], theta=theta())

        position_fns = [
            self._get_random_position_inside_opponent_penalty_area,
            self._get_random_position_inside_field,
            self._get_random_position_inside_opponent_area
        ]

        for i in range(self.n_robots_yellow):
            position = get_position(position_fns[i])
            frame.robots_yellow[i] = Robot(x=position[0], y=position[1], theta=theta())

        self.episode_initial_time = time.time()

        return frame