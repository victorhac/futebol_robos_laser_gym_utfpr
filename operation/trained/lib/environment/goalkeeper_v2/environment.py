import math
import random
import time
import numpy as np
from rsoccer_gym.Entities import Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv
from gym.spaces import Box
from rsoccer_gym.Utils.Utils import OrnsteinUhlenbeckAction

from ...helpers.configuration_helper import ConfigurationHelper
from ...helpers.model_helper import ModelHelper

TRAINING_EPISODE_DURATION = ConfigurationHelper.getTrainingEpisodeDuration()

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

    def _get_commands(self, actions):
        commands = []
        self.energy_penalty = -(abs(actions[0] * 100) + abs(actions[1] * 100))
        v_wheel0, v_wheel1 = self._actions_to_v_wheels(actions)
        commands.append(Robot(yellow=False, id=0, v_wheel0=v_wheel0,
                              v_wheel1=v_wheel1))

        # Send random commands to the other robots
        for i in range(1, self.n_robots_blue):
            actions = self.ou_actions[i].sample()
            v_wheel0, v_wheel1 = self._actions_to_v_wheels(actions)
            commands.append(Robot(yellow=False, id=i, v_wheel0=v_wheel0,
                                  v_wheel1=v_wheel1))

        atk_action, _ = self.attacker.predict(self._attacker_observations())
        v_wheel0, v_wheel1 = self._actions_to_v_wheels(atk_action)
        # we invert the speed on the wheels because of the attacker's reflection on the Y axis.
        commands.append(Robot(yellow=True, id=0, v_wheel0=v_wheel1,
                              v_wheel1=v_wheel0))
        for i in range(1, self.n_robots_yellow):
            actions = self.ou_actions[self.n_robots_blue+i].sample()
            v_wheel0, v_wheel1 = self._actions_to_v_wheels(actions)
            commands.append(Robot(yellow=False, id=i, v_wheel0=v_wheel0,
                                  v_wheel1=v_wheel1))

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
    
    def _get_field_length(self):
        return self.field.length
    
    def _get_field_width(self):
        return self.field.width

    def __move_reward_y(self):
        '''Calculate Move to ball_Y reward

        Cosine between the robot vel_Y vector and the vector robot_Y -> ball_Y.
        This indicates rather the robot is moving towards the ball_Y or not.
        '''
        ball = np.array([np.clip(self.frame.ball.y, -0.35, 0.35)])
        robot = np.array([self.frame.robots_blue[0].y])
        robot_vel = np.array([self.frame.robots_blue[0].v_y])
        robot_ball = ball - robot
        robot_ball = robot_ball/np.linalg.norm(robot_ball)

        move_reward = np.dot(robot_ball, robot_vel)

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
    
    def __defended_ball(self):
        '''Calculate Defended Ball Reward 
        
        Create a zone between the goalkeeper and if the ball enters this zone
        keep the ball speed vector norm to know the direction it entered, 
        and if the ball leaves the area in a different direction it means 
        that the goalkeeper defended the ball.
        '''
        pos = np.array([self.frame.robots_blue[0].x,
                        self.frame.robots_blue[0].y])
        ball = np.array([self.frame.ball.x, self.frame.ball.y])
        distance_gk_ball = np.linalg.norm(pos - ball) * 100 
        field_half_length = self._get_field_length() / 2

        defense_reward = 0
        if distance_gk_ball < 8 and not self.is_inside:
            self.previous_ball_direction.append((self.frame.ball.v_x + 0.000001) / \
                                                (abs(self.frame.ball.v_x)+ 0.000001))
            self.previous_ball_direction.append((self.frame.ball.v_y + 0.000001) / \
                                                (abs(self.frame.ball.v_y) + 0.000001))
            self.is_inside = True
        elif self.is_inside:
            direction_ball_vx = (self.frame.ball.v_x + 0.000001) / \
                                (abs(self.frame.ball.v_x) + 0.000001)
            direction_ball_vy = (self.frame.ball.v_y + 0.000001) / \
                                (abs(self.frame.ball.v_x) + 0.000001)

            if (self.previous_ball_direction[0] != direction_ball_vx or \
                self.previous_ball_direction[1] != direction_ball_vy) and \
                self.frame.ball.x > -field_half_length+0.1:
                self.is_inside = False
                self.previous_ball_direction.clear()
                defense_reward = 1
        
        return defense_reward
    
    def _get_goal_depth(self):
        return self.field.goal_depth

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
        has_received_goal = ball.x < -self._get_field_length() / 2

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
                dist_robot_own_goal_bar = -self._get_field_length() / \
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
        field_half_length = self._get_field_length() / 2
        field_half_width = self._get_field_length() / 2
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
