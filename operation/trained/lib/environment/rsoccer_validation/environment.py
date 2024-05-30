import math
import random
import time
import numpy as np
from gym.spaces import Box
from rsoccer_gym.Utils.Utils import OrnsteinUhlenbeckAction
from rsoccer_gym.Entities import Frame, Robot, Ball
from rsoccer_gym.Utils import KDTree

from lib.environment.base_environment import VSSBaseEnv
from lib.helpers.rsoccer_helper import RSoccerHelper

from ...helpers.field_helper import FieldHelper
from ...helpers.configuration_helper import ConfigurationHelper
from ...helpers.model_helper import ModelHelper

TRAINING_EPISODE_DURATION = ConfigurationHelper.getTrainingEpisodeDuration()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()
V_WHEEL_DEADZONE = 0.05

class Environment(VSSBaseEnv):
    def __init__(self):
        super().__init__(
            field_type=0,
            n_robots_blue=3,
            n_robots_yellow=3,
            time_step=0.025)

        self.action_space = Box(
            low=-1,
            high=1,
            shape=(2,),
            dtype=np.float32)

        self.observation_space = Box(
            low=-1,
            high=1,
            shape=(40,),
            dtype=np.float32)

        self.has_robot_touched_ball = False
        self.previous_ball_potential = None

        self.attacker = ModelHelper.get_attacker_model()
        self.defensor = ModelHelper.get_defensor_model()
        self.goalkeeper = ModelHelper.get_goalkeeper_v2_model()

        self.episode_initial_time = 0

        self.__set_ou_actions()

    def __set_ou_actions(self):
        self.ou_actions = [
            OrnsteinUhlenbeckAction(self.action_space, dt=self.time_step) \
            for _ in range(self.n_robots_blue + self.n_robots_yellow)
        ]
    
    def __is_ball_inside_goal_area(self):
        ball = self.get_ball()
        return self.__is_inside_own_goal_area((ball.x, ball.y))
    
    def __is_ball_inside_opponent_area(self):
        ball = self.get_ball()
        return self.__is_inside_opponent_area((ball.x, ball.y))
    
    def __is_inside_opponent_area(self, position: tuple[float, float]):
        return FieldHelper.is_inside_opponent_area(position, IS_LEFT_TEAM)
    
    def __is_robot_touching_ball(self):
        robot = self.frame.robots_blue[0]
        ball = self.get_ball()

        robot_position = (robot.x, robot.y)
        ball_position = (ball.x, ball.y)

        return FieldHelper.is_touching(
            robot_position,
            self.get_robot_radius(),
            ball_position,
            self.get_ball_radius(),
            .001)
    
    def __is_inside_own_goal_area(self, position: tuple[float, float]):
        return FieldHelper.is_inside_own_goal_area(
            position,
            self.get_field_length(),
            self.get_penalty_length(),
            self.get_penalty_width(),
            IS_LEFT_TEAM)
    
    def _has_goal_scored(self):
        if self.frame.ball.x > self.field.length / 2:
            return True
        elif self.frame.ball.x < -self.field.length / 2:
            return True
        return False
    
    def _is_done(self):
        if self._has_goal_scored():
            return True
        elif time.time() - self.episode_initial_time > TRAINING_EPISODE_DURATION:
            return True
        return False
    
    def __create_robot(
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
    
    def __create_robot_with_ou_action(
        self,
        id: int,
        is_yellow_robot: bool,
        ou_action_id: int
    ):
        actions = self.ou_actions[ou_action_id].sample()
        v_wheel0, v_wheel1 = self._actions_to_v_wheels(actions)

        return self.__create_robot(id, is_yellow_robot, v_wheel0, v_wheel1)
    
    def reset(self):
        self.actions = None
        self.previous_ball_potential = None
        self.has_robot_touched_ball = False

        for ou in self.ou_actions:
            ou.reset()

        return super().reset()

    def _attacker_observations(self, reflection=False):
        observation = []
        ball = self.frame.ball

        def correct(x):
            return -x if reflection else x

        observation.extend([
            self.norm_pos(correct(ball.x)),
            self.norm_pos(ball.y),
            self.norm_v(correct(ball.v_x)),
            self.norm_v(ball.v_y)
        ])

        if reflection:
            main_team = self.frame.robots_yellow
            secondary_team = self.frame.robots_blue
        else:
            main_team = self.frame.robots_blue
            secondary_team = self.frame.robots_yellow
        
        for _, robot in main_team.items():
            observation.extend([
                self.norm_pos(correct(robot.x)),
                self.norm_pos(robot.y),
                np.sin(np.deg2rad(robot.theta)),
                correct(np.cos(np.deg2rad(robot.theta))),
                self.norm_v(correct(robot.v_x)),
                self.norm_v(robot.v_y),
                self.norm_w(correct(robot.v_theta))
            ])

        for _, robot in secondary_team.items():
            observation.extend([
                self.norm_pos(correct(robot.x)),
                self.norm_pos(robot.y),
                self.norm_v(correct(robot.v_x)),
                self.norm_v(robot.v_y),
                self.norm_w(correct(robot.v_theta))
            ])

        return np.array(observation)
    
    def _defensor_observations(self, reflection=False):
        observation = []
        ball = self.frame.ball

        def correct(x):
            return -x if reflection else x

        observation.extend([
            self.norm_pos(correct(ball.x)),
            self.norm_pos(ball.y),
            self.norm_v(correct(ball.v_x)),
            self.norm_v(ball.v_y)
        ])

        if reflection:
            robot = self.frame.robots_yellow[1]
        else:
            robot = self.frame.robots_blue[1]

        observation.extend([
            self.norm_pos(correct(robot.x)),
            self.norm_pos(robot.y),
            np.sin(np.deg2rad(robot.theta)),
            correct(np.cos(np.deg2rad(robot.theta))),
            self.norm_v(correct(robot.v_x)),
            self.norm_v(robot.v_y),
            self.norm_w(correct(robot.v_theta))
        ])

        if reflection:
            main_team = self.frame.robots_yellow
            secondary_team = self.frame.robots_blue
        else:
            main_team = self.frame.robots_blue
            secondary_team = self.frame.robots_yellow
        
        for i, robot in main_team.items():
            if i == 1:
                continue

            observation.extend([
                self.norm_pos(correct(robot.x)),
                self.norm_pos(robot.y),
                np.sin(np.deg2rad(robot.theta)),
                correct(np.cos(np.deg2rad(robot.theta))),
                self.norm_v(correct(robot.v_x)),
                self.norm_v(robot.v_y),
                self.norm_w(correct(robot.v_theta))
            ])

        for i, robot in secondary_team.items():
            observation.extend([
                self.norm_pos(correct(robot.x)),
                self.norm_pos(robot.y),
                self.norm_v(correct(robot.v_x)),
                self.norm_v(robot.v_y),
                self.norm_w(correct(robot.v_theta))
            ])

        return np.array(observation)
    
    def _goalkeeper_observations(self, reflection=False):
        observation = []
        ball = self.frame.ball

        def correct(x):
            return -x if reflection else x

        observation.extend([
            self.norm_pos(correct(ball.x)),
            self.norm_pos(ball.y),
            self.norm_v(correct(ball.v_x)),
            self.norm_v(ball.v_y)
        ])
        
        if reflection:
            robot = self.frame.robots_yellow[2]
        else:
            robot = self.frame.robots_blue[2]

        observation.extend([
            self.norm_pos(correct(robot.x)),
            self.norm_pos(robot.y),
            np.sin(np.deg2rad(robot.theta)),
            correct(np.cos(np.deg2rad(robot.theta))),
            self.norm_v(correct(robot.v_x)),
            self.norm_v(robot.v_y),
            self.norm_w(correct(robot.v_theta))
        ])

        return np.array(observation)
    
    def _frame_to_observations(self):
        observation = []

        ball = self.get_ball()

        observation.extend([
            self.norm_x(ball.x),
            self.norm_y(ball.y),
            self.norm_v(ball.v_x),
            self.norm_v(ball.v_y)
        ])

        for i in range(self.n_robots_blue):
            robot = self.frame.robots_blue[i]
            robot_theta_radians = np.deg2rad(robot.theta)

            observation.extend([
                self.norm_x(robot.x),
                self.norm_y(robot.y),
                robot_theta_radians / np.pi,
                self.norm_v(robot.v_x),
                self.norm_v(robot.v_y)
            ])

        for i in range(self.n_robots_yellow):
            robot = self.frame.robots_yellow[i]

            observation.extend([
                self.norm_x(robot.x),
                self.norm_y(robot.y),
                robot_theta_radians / np.pi,
                self.norm_v(robot.v_x),
                self.norm_v(robot.v_y)
            ])

        return np.array(observation, dtype=np.float32)
    
    def _get_robot_command(
        self,
        robot_id: int,
        is_yellow: bool,
        observation: np.ndarray,
        model
    ):
        action, _ = model.predict(observation)
        v_wheel0, v_wheel1 = self._actions_to_v_wheels(action)

        return self.__create_robot(robot_id, is_yellow, v_wheel0, v_wheel1)
        
    def _get_commands(self, actions):
        commands = []

        v_wheel_0, v_wheel_1 = self._actions_to_v_wheels(actions)

        robot = self.__create_robot(0, False, 0, 0)
        commands.append(robot)

        for i in range(1, self.n_robots_blue):
            robot = self.__create_robot(i, False, 0, 0)
            commands.append(robot)

        robot = self.__create_robot(0, True, 1, 1)
        commands.append(robot)

        for i in range(1, self.n_robots_yellow):
            robot = self.__create_robot(i, True, 0, 0)
            commands.append(robot)

        return commands

    def _actions_to_v_wheels(self, actions):
        left_wheel_speed = actions[0] * self.max_v
        right_wheel_speed = actions[1] * self.max_v

        left_wheel_speed, right_wheel_speed = np.clip(
            (left_wheel_speed, right_wheel_speed),
            -self.max_v,
            self.max_v)

        if -V_WHEEL_DEADZONE < left_wheel_speed < V_WHEEL_DEADZONE:
            left_wheel_speed = 0

        if -V_WHEEL_DEADZONE < right_wheel_speed < V_WHEEL_DEADZONE:
            right_wheel_speed = 0

        left_wheel_speed /= self.field.rbt_wheel_radius
        right_wheel_speed /= self.field.rbt_wheel_radius

        return left_wheel_speed, right_wheel_speed
    
    def __ball_gradient_reward(
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
        
    def __move_reward(self):
        ball = self.get_ball()
        robot = self.frame.robots_blue[0]

        ball_position = np.array([ball.x, ball.y])
        robot_position = np.array([robot.x, robot.y])
        
        robot_velocities = np.array([robot.v_x, robot.v_y])
        robot_ball_vector = ball_position - robot_position
        robot_ball_vector = robot_ball_vector / np.linalg.norm(robot_ball_vector)

        move_reward = np.dot(robot_ball_vector, robot_velocities)

        return np.clip(move_reward / 0.4, -5.0, 5.0)
    
    def __energy_penalty(self):
        en_penalty_1 = abs(self.sent_commands[0].v_wheel0)
        en_penalty_2 = abs(self.sent_commands[0].v_wheel1)
        return - (en_penalty_1 + en_penalty_2)
    
    def __has_robot_defended(self):
        return self.has_robot_touched_ball and self.__is_ball_inside_opponent_area()
    
    def _calculate_reward_and_done(self):
        reward = 0
        w_move = 0.2
        w_ball_grad = 0.8
        w_energy = 2e-4

        robot = self.frame.robots_blue[0]

        if self.__is_robot_touching_ball():
            self.has_robot_touched_ball = True

        if self.__has_robot_defended():
            reward = 10
        elif self.__is_ball_inside_goal_area():
            reward = -10
        elif self.__is_inside_opponent_area((robot.x, robot.y)):
            reward = -5
        else:
            grad_ball_potential, ball_gradient = self.__ball_gradient_reward(self.previous_ball_potential)
            self.previous_ball_potential = ball_gradient

            move_reward = self.__move_reward()
            energy_penalty = self.__energy_penalty()

            reward = w_move * move_reward + \
                w_ball_grad * grad_ball_potential + \
                w_energy * energy_penalty

        return reward, self._is_done()
    
    def __get_random_position_inside_field(self):
        return FieldHelper.get_random_position_inside_field(
            self.get_field_length(),
            self.get_field_width())
    
    def __get_random_position_inside_own_area(self):
        return FieldHelper.get_random_position_inside_own_area(
            self.get_field_length(),
            self.get_field_width(),
            IS_LEFT_TEAM)
    
    def __get_random_position_inside_opponent_area(self):
        return FieldHelper.get_random_position_inside_opponent_area(
            self.get_field_length(),
            self.get_field_width(),
            IS_LEFT_TEAM)

    def _get_initial_positions_frame(self):
        def theta(): return random.uniform(0, 360)

        frame: Frame = Frame()
        ball_position = self.__get_random_position_inside_field()
        frame.ball = Ball(x=ball_position[0], y=ball_position[1])

        min_dist = 0.1

        places = KDTree()

        places.insert(ball_position)
        
        for i in range(self.n_robots_blue):
            pos = self.__get_random_position_inside_own_area()

            while places.get_nearest(pos)[1] < min_dist:
                pos = self.__get_random_position_inside_own_area()

            places.insert(pos)
            frame.robots_blue[i] = Robot(x=pos[0], y=pos[1], theta=-170)

        frame.robots_blue[0] = Robot(x=-0.5, y=0.5, theta=170)

        for i in range(self.n_robots_yellow):
            pos = self.__get_random_position_inside_opponent_area()

            while places.get_nearest(pos)[1] < min_dist:
                pos = self.__get_random_position_inside_opponent_area()

            places.insert(pos)
            frame.robots_yellow[i] = Robot(x=pos[0], y=pos[1], theta=-170)

        self.episode_initial_time = time.time()

        return frame