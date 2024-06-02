import math
import random
import time
import numpy as np
from rsoccer_gym.Entities import Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv
from gym.spaces import Box
from rsoccer_gym.Utils.Utils import OrnsteinUhlenbeckAction

from operation.trained.lib.geometry.geometry_utils import GeometryUtils
from operation.trained.lib.helpers.field_helper import FieldHelper

from ...helpers.configuration_helper import ConfigurationHelper
from ...helpers.model_helper import ModelHelper

TRAINING_EPISODE_DURATION = ConfigurationHelper.get_rsoccer_training_episode_duration()
ROBOT_WIDTH = ConfigurationHelper.get_rsoccer_robot_width()
IS_LEFT_TEAM = ConfigurationHelper.get_rsoccer_is_left_team()

class Environment(VSSBaseEnv):
    def __init__(self):
        super().__init__(field_type=0,
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
            shape=(11,),
            dtype=np.float32)

        self.energy_penalty = 0
        self.previous_ball_direction = []
        self.is_inside = False
        self.ball_inside_area = False
        self.attacker = ModelHelper.get_attacker_model()
        self.v_wheel_deadzone = 0.05
        self.episode_initial_time = 0
        self.ou_actions = []
        for i in range(self.n_robots_blue + self.n_robots_yellow):
            self.ou_actions.append(
                OrnsteinUhlenbeckAction(self.action_space, dt=self.time_step)
            )

    def _attacker_observations(self):
        observation = []
        ball = self.frame.ball

        observation.extend([
            self.norm_pos(-ball.x),
            self.norm_pos(ball.y),
            self.norm_v(-ball.v_x),
            self.norm_v(ball.v_y)
        ])
        
        #  we reflect the side that the attacker is attacking,
        #  so that he will attack towards the goal where the goalkeeper is
        for i in range(self.n_robots_yellow):
            robot = self.frame.robots_yellow[i]

            observation.extend([
                self.norm_pos(-robot.x),
                self.norm_pos(robot.y),
                np.sin(np.deg2rad(robot.theta)),
                -np.cos(np.deg2rad(robot.theta)),
                self.norm_v(-robot.v_x),
                self.norm_v(robot.v_y),
                self.norm_w(-robot.v_theta)
            ])

        for i in range(self.n_robots_blue):
            robot = self.frame.robots_blue[i]

            observation.extend([
                self.norm_pos(-robot.x),
                self.norm_pos(robot.y),
                self.norm_v(-robot.v_x),
                self.norm_v(robot.v_y),
                self.norm_w(-robot.v_theta)
            ])

        return np.array(observation)

    def _frame_to_observations(self):
        observation = []

        ball = self.frame.ball

        observation.extend([
            self.norm_pos(ball.x),
            self.norm_pos(ball.y),
            self.norm_v(ball.v_x),
            self.norm_v(ball.v_y)
        ])

        robot = self.frame.robots_blue[0]
        robot_theta_radians = np.deg2rad(robot.theta)

        observation.extend([
            self.norm_pos(robot.x),
            self.norm_pos(robot.y),
            np.sin(robot_theta_radians),
            np.cos(robot_theta_radians),
            self.norm_v(robot.v_x),
            self.norm_v(robot.v_y),
            self.norm_w(robot.v_theta)
        ])

        return np.array(observation)
    
    def __get_robot(
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
    
    def __get_robot_with_ou_action(
        self,
        id: int,
        is_yellow_robot: bool,
        ou_action_id: int
    ):
        actions = self.ou_actions[ou_action_id].sample()
        v_wheel0, v_wheel1 = self._actions_to_v_wheels(actions)

        return self.__get_robot(id, is_yellow_robot, v_wheel0, v_wheel1)
        
    def _get_commands(self, actions):
        commands = []

        v_wheel0, v_wheel1 = self._actions_to_v_wheels(actions)

        robot = self.__get_robot(0, False, v_wheel0, v_wheel1)
        
        commands.append(robot)

        for i in range(1, self.n_robots_blue):
            blue_robot = self.__get_robot_with_ou_action(i, False, i)
            commands.append(blue_robot)

        attacker_action, _ = self.attacker.predict(self._attacker_observations())
        v_wheel0, v_wheel1 = self._actions_to_v_wheels(attacker_action)

        attacker_robot = self.__get_robot(0, True, v_wheel1, v_wheel0)
        commands.append(attacker_robot)

        for i in range(1, self.n_robots_yellow):
            yellow_robot = self.__get_robot_with_ou_action(i, True, self.n_robots_blue + i)
            commands.append(yellow_robot)

        return commands

    def _actions_to_v_wheels(self, actions):
        left_wheel_speed = actions[0] * self.max_v
        right_wheel_speed = actions[1] * self.max_v

        left_wheel_speed, right_wheel_speed = np.clip(
            (left_wheel_speed, right_wheel_speed),
            -self.max_v,
            self.max_v)

        if -self.v_wheel_deadzone < left_wheel_speed < self.v_wheel_deadzone:
            left_wheel_speed = 0

        if -self.v_wheel_deadzone < right_wheel_speed < self.v_wheel_deadzone:
            right_wheel_speed = 0

        left_wheel_speed /= self.field.rbt_wheel_radius
        right_wheel_speed /= self.field.rbt_wheel_radius

        return left_wheel_speed , right_wheel_speed
    
    def __get_field_length(self):
        return self.field.length
    
    def _get_field_width(self):
        return self.field.width
    
    def _get_field_penalty_length(self):
        return self.field.penalty_length

    def __move_reward_y(self):
        robot = self.frame.robots_blue[0]
        ball = self.frame.ball

        field_half_width = self._get_field_width() / 2

        ball_vector = np.array([np.clip(ball.y, -field_half_width, field_half_width)])
        robot_vector = np.array([robot.y])
        robot_vel_vector = np.array([robot.v_y])

        robot_ball_vector = ball_vector - robot_vector
        robot_ball_vector = robot_ball_vector / np.linalg.norm(robot_ball_vector)

        move_reward = np.dot(robot_ball_vector, robot_vel_vector)

        return np.clip(move_reward / 0.4, -5.0, 5.0)
    
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
    
    def __get_own_goal_position(self):
        return FieldHelper.getOwnGoalPosition(self.__get_field_length(), IS_LEFT_TEAM)
    
    def _get_own_penalty_position(self):
        return FieldHelper.getOwnPenaltyPosition(
            self.__get_field_length(),
            self._get_field_penalty_length(),
            IS_LEFT_TEAM)
    
    def __calculate_ball_vector_goal_intersection(self):
        own_goal_position = self.__get_own_goal_position()

        goal_line_x = own_goal_position[0]

        ball = self.frame.ball

        if ball.v_x == 0:
            return None
        
        time_to_goal = (goal_line_x - ball.x) / ball.v_x
        intersection_y = ball.y + ball.v_y * time_to_goal
        
        if abs(intersection_y) <= self.__get_goal_width() / 2:
            return goal_line_x, intersection_y
        else:
            return None
    
    def __get_distance_to_ball_goal_defense_line_segment(self):
        ball = self.frame.ball
        robot = self.frame.robots_blue[0]

        own_goal_position = self.__get_own_goal_position()
        goal_target_position = self.__calculate_ball_vector_goal_intersection()

        if goal_target_position is None:
            robot_max_y = self.__get_goal_width() / 2 - ROBOT_WIDTH / 2
            x = own_goal_position[0]
            y = np.clip(
                ball.y,
                -robot_max_y,
                robot_max_y
            )
        else:
            x, y = goal_target_position

        own_penalty_position = self._get_own_penalty_position()

        ball_to_goal_line_equation = GeometryUtils.lineEquation(
            (x, y),
            (ball.x, ball.y)
        )

        penalty_y = GeometryUtils.find_y(
            ball_to_goal_line_equation,
            own_penalty_position[0]
        )

        robot_position = robot.x, robot.y

        return GeometryUtils.distance_point_to_line_segment(
            robot_position,
            goal_target_position,
            (own_penalty_position[0], penalty_y)
        )
    
    def __get_ball(self):
        return self.frame.ball
    
    def __ball_gradient(
        self,
        previous_potencial: float | None = None
    ):
        field_length = self.__get_field_length()
        goal_depth = self.__get_goal_depth()
        ball = self.__get_ball()

        convert_m_to_cm = lambda x: x * 100

        length_cm = convert_m_to_cm(field_length)
        half_lenght = (field_length / 2) + goal_depth

        dx_d = convert_m_to_cm((half_lenght + ball.x))
        dx_a = convert_m_to_cm((half_lenght - ball.x))
        dy = convert_m_to_cm(ball.y)

        dist_1 = math.sqrt(dx_a ** 2 + 2 * dy ** 2)
        dist_2 = math.sqrt(dx_d ** 2 + 2 * dy ** 2)

        ball_potential = ((-dist_1 + dist_2) / length_cm - 1) / 2

        if previous_potencial is not None:
            diff = ball_potential - previous_potencial
            return np.clip(
                diff * 3 / self.time_step,
                -5.0,
                5.0)
        else:
            return 0
        
    def __move_reward(self):
        ball = self.__get_ball()
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
    
    def __defended_ball(self):
        
        
        return defense_reward
    
    def __get_goal_depth(self):
        return self.field.goal_depth

    def __get_goal_width(self):
        return self.field.goal_width

    def _calculate_reward_and_done(self):
        done = False
        reward = 0
        goal_score = 0
        move_y_reward = 0
        dist_robot_own_goal_bar = 0
        ball_defense_reward = 0
        ball_leave_area_reward = 0

        w_defense = 1.8
        w_move_y  = 0.3
        w_distance = 0.1
        w_blva = 2.0

        goalkeeper = self.frame.robots_blue[0]
        ball = self.frame.ball

        goalkeeper_left_goal_area = goalkeeper.x > -0.63 or abs(goalkeeper.y) > 0.4
        is_ball_inside_goal_area = ball.x <= -0.6 and abs(ball.y) <= 0.35
        has_received_goal = ball.x < -self.__get_field_length() / 2

        if goalkeeper_left_goal_area:
            reward = -5
            done = True
            self.is_inside = False
            self.ball_inside_area = False
        elif self.last_frame is not None:
            self.previous_ball_potential = None

            if (not self.ball_inside_area) and is_ball_inside_goal_area:
                self.ball_inside_area = True

            if self.ball_inside_area and not is_ball_inside_goal_area:
                ball_leave_area_reward = 1 
                self.ball_inside_area = False
                done = True

            if has_received_goal:
                goal_score = -2 
                self.ball_inside_area = False

            if goal_score != 0:
                reward = goal_score
            else:
                move_y_reward = self.__move_reward_y()
                ball_defense_reward = self.__defended_ball() 
                dist_robot_own_goal_bar = -self.__get_field_length() / \
                    2 + 0.15 - goalkeeper.x

                reward = w_move_y * move_y_reward + \
                         w_distance * dist_robot_own_goal_bar + \
                         w_defense * ball_defense_reward + \
                         w_blva * ball_leave_area_reward

        done = goal_score != 0 or done or (time.time() - self.episode_initial_time > TRAINING_EPISODE_DURATION)

        return reward, done

    def _get_initial_positions_frame(self):
        """
        Goalie starts at the center of the goal, striker and ball randomly.
        Other robots also starts at random positions.
        """
        field_half_length = self.__get_field_length() / 2
        field_half_width = self.__get_field_length() / 2
        def x(): return random.uniform(-field_half_length + 0.1,
                                       field_half_length - 0.1)
        def y(): return random.uniform(-field_half_width + 0.1,
                                       field_half_width - 0.1)
        pos_frame: Frame = Frame()

        pos_frame.ball.x = random.uniform(-field_half_length + 0.1,
                                          field_half_length - 0.1)
        pos_frame.ball.y = random.uniform(-field_half_width + 0.1,
                                          field_half_width - 0.1)

        pos_frame.robots_blue[0] = Robot(x=-field_half_length + 0.05,
                                         y=0.0,
                                         theta=0)
        pos_frame.robots_blue[1] = Robot(x=x(), y=y(), theta=0)
        pos_frame.robots_blue[2] = Robot(x=x(), y=y(), theta=0)

        pos_frame.robots_yellow[0] = Robot(x=x(), y=y(), theta=math.pi)
        pos_frame.robots_yellow[1] = Robot(x=x(), y=y(), theta=math.pi)
        pos_frame.robots_yellow[2] = Robot(x=x(), y=y(), theta=math.pi)

        self.episode_initial_time = time.time()

        return pos_frame
