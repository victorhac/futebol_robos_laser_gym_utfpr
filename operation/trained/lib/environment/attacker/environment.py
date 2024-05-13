import math
import time
import numpy as np

from gym.spaces import Box

from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv
from rsoccer_gym.Utils.kdtree import KDTree

from lib.geometry.geometry_utils import GeometryUtils
from lib.motion.motion_utils import MotionUtils

from ...helpers.field_helper import FieldHelper
from ...helpers.rsoccer_helper import RSoccerHelper
from ...helpers.configuration_helper import ConfigurationHelper

TEAM_BLUE_NUMBER_ROBOTS = ConfigurationHelper.getTeamBlueNumberRobots()
TEAM_YELLOW_NUMBER_ROBOTS = ConfigurationHelper.getTeamYellowNumberRobots()
IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()

TRAINING_TIME_STEP = ConfigurationHelper.getTrainingTimeStep()
TRAINING_EPISODE_DURATION = ConfigurationHelper.getTrainingEpisodeDuration()
TRAINING_VELOCITY_CLIP_VALUE = ConfigurationHelper.getTrainingVelocityClipValue()

ROBOT_SPEED_DEAD_ZONE = ConfigurationHelper.getRobotSpeedDeadZone()
ROBOT_SPEED_BASE = ConfigurationHelper.getRobotSpeedBase()

class Environment(VSSBaseEnv):
    def __init__(self):
        super().__init__(
            field_type=0,
            n_robots_blue=TEAM_BLUE_NUMBER_ROBOTS,
            n_robots_yellow=TEAM_YELLOW_NUMBER_ROBOTS,
            time_step=TRAINING_TIME_STEP
        )

        self.action_space = self._get_action_space()
        self.observation_space = self._get_observation_space()
        self.episode_initial_time = 0
        self.error = 0

    def _get_action_space(self):
        max_field_x = self._get_field_length() / 2
        max_field_y = self._get_field_width() / 2

        return Box(
            low=np.array([-max_field_x, -max_field_y], dtype=np.float64),
            high=np.array([max_field_x, max_field_y], dtype=np.float64),
            dtype=np.float64,
            shape=(2,)
        )

    def _get_observation_space(self):
        max_field_x = self._get_field_length() / 2
        max_field_y = self._get_field_width() / 2

        observation_space_max_values = [
            max_field_x,
            max_field_y,
            math.pi,

            max_field_x,
            max_field_y,
            math.pi,

            max_field_x,
            max_field_y,
            math.pi,

            max_field_x,
            max_field_y,
            math.pi,

            max_field_x,
            max_field_y,
            TRAINING_VELOCITY_CLIP_VALUE,
            TRAINING_VELOCITY_CLIP_VALUE
        ]
        
        return Box(
            low=np.array([-i for i in observation_space_max_values], dtype=np.float64),
            high=np.array(observation_space_max_values, dtype=np.float64),
            dtype=np.float64,
            shape=(len(observation_space_max_values),)
        )
    
    def _frame_to_observations(self):
        observations = []

        frame = self.frame

        def append_robot_observations(robot):
            observations.extend([
                robot.x,
                robot.y,
                RSoccerHelper.getCorrectedAngle(robot.theta)
            ])
        
        for i in range(self.n_robots_yellow):
            append_robot_observations(frame.robots_yellow[i])

        for i in range(self.n_robots_blue):
            append_robot_observations(frame.robots_blue[i])

        ball = self.frame.ball

        speed_clip = lambda x: np.clip(
            x,
            -TRAINING_VELOCITY_CLIP_VALUE,
            TRAINING_VELOCITY_CLIP_VALUE)

        observations.extend([
            ball.x,
            ball.x,
            speed_clip(ball.v_x),
            speed_clip(ball.v_y)
        ])
        
        return np.array(observations)

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

    def _get_commands(self, actions):
        fieldData, _ = self._get_field_datas()

        robot = fieldData.robots[0]

        targetPosition = (actions[0], actions[1])

        velocities = MotionUtils.go_to_point(robot, targetPosition, self.error)

        (leftSpeed, rightSpeed, self.error) = velocities

        return [
            RSoccerHelper.get_rsoccer_robot_action(
                0,
                IS_YELLOW_TEAM,
                leftSpeed,
                rightSpeed
            )
        ]
    
    def _get_distance_to_obstacle_reward(self):
        field_data, opponent_field_data = self._get_field_datas()
        ball = field_data.ball
        robot = field_data.robots[0]

        robot_closest_obstacle = MotionUtils.findClosestObstacle(
            0,
            field_data,
            opponent_field_data,
            ball.get_position_tuple()
        )

        if robot_closest_obstacle is None:
            return 0
        
        distance = GeometryUtils.distance(
            robot.get_position_tuple(),
            robot_closest_obstacle.get_position_tuple()
        )

        danger_distance_threshold = 0.5

        if distance >= danger_distance_threshold:
            reward = 0
        else:
            reward = -danger_distance_threshold + distance

        return (1 / danger_distance_threshold) * reward
        
    def _get_ball_towards_goal_reward(self):
        field_data, _ = self._get_field_datas()
        ball = field_data.ball
        opponent_goal_position = self._get_opponent_goal_position()

        distance = GeometryUtils.distance(
            ball.get_position_tuple(),
            opponent_goal_position
        )

        return -distance

    def _get_heading_towards_ball_reward(self):
        field_data, _ = self._get_field_datas()
        ball = field_data.ball
        robot = field_data.robots[0]

        distance = GeometryUtils.distance(
            robot.get_position_tuple(),
            ball.get_position_tuple()
        )

        return -distance
    
    def _get_goal_reward(self):
        remaining_time = TRAINING_EPISODE_DURATION - (time.time() - self.episode_initial_time)
        reward = (8 * remaining_time) / TRAINING_EPISODE_DURATION + 2
        return -reward if self._is_goal_received() else reward
    
    def _calculate_reward(self):
        if self._has_goal_scored():
            return self._get_goal_reward()
        
        obstacle_reward = self._get_distance_to_obstacle_reward()
        distance_to_ball_reward = self._get_heading_towards_ball_reward()
        distance_ball_to_goal_reward = self._get_ball_towards_goal_reward()

        if obstacle_reward == 0:
            return (distance_to_ball_reward + distance_ball_to_goal_reward) / 2

        return (obstacle_reward +
            distance_to_ball_reward +
            distance_ball_to_goal_reward) / 3
        
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
    
    def _get_field_length(self):
        return self.field.length
    
    def _get_field_width(self):
        return self.field.width

    def _calculate_reward_and_done(self):
        reward = self._calculate_reward()
        done = self._is_done()
        reward = self._calculate_reward()
        
        return reward, done
    
    def _get_field_random_position(self):
        margin = 0.1
        return FieldHelper.getFieldRandomPosition(
            self._get_field_length() - margin,
            self._get_field_width() - margin)
    
    def _get_random_theta(self):
        return FieldHelper.getRandomTheta() * (180 / math.pi)
    
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
    
    def _get_own_goal_position(self):
        return FieldHelper.getOwnGoalPosition(self._get_field_length(), IS_LEFT_TEAM)
    
    def _get_opponent_goal_position(self):
        return FieldHelper.getOpponentGoalPosition(self._get_field_length(), IS_LEFT_TEAM)
    
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