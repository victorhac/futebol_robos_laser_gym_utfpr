import math
import time
import numpy as np
from typing import Dict
from gym.spaces import Box

from rsoccer_gym.Utils.Utils import OrnsteinUhlenbeckAction
from rsoccer_gym.Entities import Frame, Robot, Ball
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv
from rsoccer_gym.Utils import KDTree

from lib.geometry.geometry_utils import GeometryUtils

from ...helpers.field_helper import FieldHelper
from ...helpers.rsoccer_helper import RSoccerHelper
from ...helpers.configuration_helper import ConfigurationHelper

TEAM_BLUE_NUMBER_ROBOTS = ConfigurationHelper.getTeamBlueNumberRobots()
TEAM_YELLOW_NUMBER_ROBOTS = ConfigurationHelper.getTeamYellowNumberRobots()
IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()

TRAINING_TIME_STEP = ConfigurationHelper.getTrainingTimeStep()
TRAINING_EPISODE_DURATION = ConfigurationHelper.getTrainingEpisodeDuration()

class Environment(VSSBaseEnv):
    def __init__(self):
        super().__init__(
            field_type=0,
            n_robots_blue=TEAM_BLUE_NUMBER_ROBOTS,
            n_robots_yellow=TEAM_YELLOW_NUMBER_ROBOTS,
            time_step=TRAINING_TIME_STEP)

        self.action_space = Box(
            low=-1,
            high=1,
            shape=(2,),
            dtype=np.float64)
        
        self.observation_space = Box(
            low=-self.NORM_BOUNDS,
            high=self.NORM_BOUNDS,
            shape=(40,),
            dtype=np.float64)

        self.previous_ball_potential = None
        self.v_wheel_deadzone = 0.05

        self.ou_actions = []
        for i in range(self.n_robots_yellow + self.n_robots_blue):
            self.ou_actions.append(
                OrnsteinUhlenbeckAction(
                    self.action_space,
                    dt=self.time_step))
            
    def _get_ou_action(self, robot_id, is_yellow_team):
        index = robot_id if is_yellow_team else self.n_robots_blue + robot_id
        return self.ou_actions[index].sample()

    def reset(self):
        self.previous_ball_potential = None
        for ou in self.ou_actions:
            ou.reset()

        return super().reset()

    def _frame_to_observations(self):
        # observation = []

        # observation.append(self.norm_pos(self.frame.ball.x))
        # observation.append(self.norm_pos(self.frame.ball.y))
        # observation.append(self.norm_v(self.frame.ball.v_x))
        # observation.append(self.norm_v(self.frame.ball.v_y))

        # for i in range(self.n_robots_blue):
        #     theta = np.deg2rad(self.frame.robots_blue[i].theta)
        #     observation.append(self.norm_pos(self.frame.robots_blue[i].x))
        #     observation.append(self.norm_pos(self.frame.robots_blue[i].y))
        #     observation.append(np.sin(theta))
        #     observation.append(np.cos(theta))
        #     observation.append(self.norm_v(self.frame.robots_blue[i].v_x))
        #     observation.append(self.norm_v(self.frame.robots_blue[i].v_y))
        #     observation.append(self.norm_w(self.frame.robots_blue[i].v_theta))

        # for i in range(self.n_robots_yellow):
        #     observation.append(self.norm_pos(self.frame.robots_yellow[i].x))
        #     observation.append(self.norm_pos(self.frame.robots_yellow[i].y))
        #     observation.append(self.norm_v(self.frame.robots_yellow[i].v_x))
        #     observation.append(self.norm_v(self.frame.robots_yellow[i].v_y))
        #     observation.append(self.norm_w(self.frame.robots_yellow[i].v_theta))

        # return np.array(observation, dtype=np.float32)
        return self._get_state()
    
    def _get_own_goal_position(self):
        return FieldHelper.getOwnGoalPosition(self._get_field_length(), IS_LEFT_TEAM)
    
    def _get_opponent_goal_position(self):
        return FieldHelper.getOpponentGoalPosition(self._get_field_length(), IS_LEFT_TEAM)
    
    def _get_distance_to_own_goal(self, robot_or_ball):
        own_goal_position = self._get_own_goal_position()
        robot_or_ball_position = robot_or_ball.get_position_tuple()
        return GeometryUtils.distance(own_goal_position, robot_or_ball_position)
    
    def _get_distance_to_opponent_goal(self, robot_or_ball):
        opponent_goal_position = self._get_opponent_goal_position()
        robot_or_ball_position = robot_or_ball.get_position_tuple()
        return GeometryUtils.distance(opponent_goal_position, robot_or_ball_position)
      
    def __ball_gradient(self):
        field_data, _ = self._get_field_datas()
        ball = field_data.ball

        convert_m_to_cm = lambda item: item * 100

        field_length_cm = convert_m_to_cm(self._get_field_length())

        distance_1 = convert_m_to_cm(self._get_distance_to_opponent_goal(ball))
        distance_2 = convert_m_to_cm(self._get_distance_to_own_goal(ball))

        ball_potential = ((-distance_1 + distance_2) / field_length_cm - 1) / 2

        gradient_ball_potential = 0

        if self.previous_ball_potential is not None:
            diff = ball_potential - self.previous_ball_potential
            gradient_ball_potential = np.clip(
                diff * 3 / self.time_step,
                -5.0,
                5.0)

        self.previous_ball_potential = ball_potential

        return gradient_ball_potential

    def __move_reward(self):
        field_data, _ = self._get_field_datas()
        ball = field_data.ball
        robot = field_data.robots[0]

        ball_vector = np.array([ball.position.x, ball.position.y])
        robot_vector = np.array([robot.position.x, robot.position.y])

        robot_speed_vector = np.array([robot.velocity.x, robot.velocity.y])

        robot_ball_vector = ball_vector - robot_vector
        robot_ball_vector = robot_ball_vector / np.linalg.norm(robot_ball_vector)

        move_reward = np.dot(robot_ball_vector, robot_speed_vector) / 0.4

        return np.clip(move_reward, -5.0, 5.0)

    def __energy_penalty(self):
        en_penalty_1 = abs(self.sent_commands[0].v_wheel0)
        en_penalty_2 = abs(self.sent_commands[0].v_wheel1)

        return - (en_penalty_1 + en_penalty_2)
    
    def _calculate_reward(self):
        w_move = 0.2
        w_ball_grad = 0.8
        w_energy = 2e-4

        if self._has_goal_scored():
            return -10 if self._is_goal_received() else 10
        else:
            if self.last_frame is not None:
                grad_ball_potential = self.__ball_gradient()
                move_reward = self.__move_reward()
                energy_penalty = self.__energy_penalty()

                return w_move * move_reward + \
                    w_ball_grad * grad_ball_potential + \
                    w_energy * energy_penalty
            else:
                return 0

    def _get_commands(self, actions):
        commands = []

        v_wheel0, v_wheel1 = self._actions_to_v_wheels_speeds(actions)
        commands.append(
            Robot(yellow=IS_YELLOW_TEAM,
                id=0,
                v_wheel0=v_wheel0,
                v_wheel1=v_wheel1))
        
        for i in range(1 if IS_YELLOW_TEAM else 0, self.n_robots_yellow):
            robot_actions = self._get_ou_action(i, True)
            v_wheel0, v_wheel1 = self._actions_to_v_wheels_speeds(robot_actions)
            commands.append(
                Robot(yellow=True,
                    id=i,
                    v_wheel0=v_wheel0,
                    v_wheel1=v_wheel1)
            )
        
        for i in range(0 if IS_YELLOW_TEAM else 1, self.n_robots_blue):
            robot_actions = self._get_ou_action(i, False)
            v_wheel0, v_wheel1 = self._actions_to_v_wheels_speeds(robot_actions)
            commands.append(
                Robot(yellow=False,
                    id=i,
                    v_wheel0=v_wheel0,
                    v_wheel1=v_wheel1)
                )

        return commands

    def _has_goal_scored(self):
        if self.frame.ball.x > self.field.length / 2:
            return True
        elif self.frame.ball.x < -self.field.length / 2:
            return True
        return False
    
    def _is_goal_received(self):
        if self.frame.ball.x > self.field.length / 2:
            return not IS_LEFT_TEAM
        elif self.frame.ball.x < -self.field.length / 2:
            return IS_LEFT_TEAM
        return None
    
    def _is_done(self):
        if self._has_goal_scored():
            return True
        if time.time() - self.episode_initial_time > TRAINING_EPISODE_DURATION:
            return True
        return False

    def _calculate_reward_and_done(self):
        reward = self._calculate_reward()
        done = self._is_done()
        reward = self._calculate_reward()

        print(f"Reward: {reward}")
        
        return reward, done
    
    def _get_field_length(self):
        return self.field.length
    
    def _get_field_width(self):
        return self.field.width
    
    def _get_field_random_position(self):
        margin = 0.1
        return FieldHelper.getFieldRandomPosition(
            self._get_field_length() - margin,
            self._get_field_width() - margin)
    
    def _get_random_theta(self):
        return FieldHelper.getRandomTheta() * (180 / math.pi)
    
    def _get_state(self):
        observations = []

        observations.append(self.frame.ball)

        for i in range(self.n_robots_blue):
            robot = self.frame.robots_blue[i]
            robot.yellow = False
            observations.append(robot)

        for i in range(self.n_robots_yellow):
            robot = self.frame.robots_yellow[i]
            robot.yellow = True
            observations.append(robot)
        
        return np.array(observations)
    
    def _get_field_datas(self):
        state = self._get_state()
        return RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)
    
    def _get_random_robot(
        self,
        id: float,
        is_yellow_team: bool
    ):
        robot_pos_x, robot_pos_y = self._get_field_random_position()
        robot_theta = self._get_random_theta()

        return Robot(
            id=id,
            x=robot_pos_x,
            y=robot_pos_y,
            theta=robot_theta,
            yellow=is_yellow_team)
    
    def _get_random_ball(self):
        ball_pos_x, ball_pos_y = self._get_field_random_position()

        return Ball(x=ball_pos_x, y=ball_pos_y)

    def _get_initial_positions_frame(self):
        frame: Frame = Frame()

        ball = self._get_random_ball()
        frame.ball = ball

        min_distance = 0.1

        places = KDTree()
        places.insert((ball.x, ball.y))

        def get_robot_without_intersection(id, is_yellow):
            robot = self._get_random_robot(id, is_yellow)
            position = (robot.x, robot.y)

            while places.get_nearest(position)[1] < min_distance:
                position = self._get_field_random_position()

            robot.x, robot.y = position
            places.insert(position)

            return robot
        
        for i in range(self.n_robots_blue):
            robot = get_robot_without_intersection(i, False)
            frame.robots_blue[i] = robot

        for i in range(self.n_robots_yellow):
            robot = get_robot_without_intersection(i, True)
            frame.robots_yellow[i] = robot

        self.episode_initial_time = time.time()

        return frame
    
    def _action_to_v_wheel_speed(self, action):
        v_wheel_speed = action * self.max_v
        v_wheel_speed = np.clip(v_wheel_speed, -self.max_v, self.max_v)

        if abs(v_wheel_speed) < self.v_wheel_deadzone:
            v_wheel_speed = 0

        v_wheel_speed /= self.field.rbt_wheel_radius

        return v_wheel_speed

    def _actions_to_v_wheels_speeds(self, actions):
        return (
            self._action_to_v_wheel_speed(actions[0]),
            self._action_to_v_wheel_speed(actions[1])
        )