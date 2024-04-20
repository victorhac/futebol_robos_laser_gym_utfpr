import math
import time
import random
import numpy as np

from gym.spaces import Box

from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv

from ..helpers.field_helper import FieldHelper
from ..training.training_utils import TrainingUtils

from ..helpers.rsoccer_helper import RSoccerHelper

from ..helpers.configuration_helper import ConfigurationHelper

CONFIGURATION = ConfigurationHelper.getConfiguration()

TEAM_BLUE_NUMBER_ROBOTS = CONFIGURATION["team"]["blue"]["number-robots"]
TEAM_YELLOW_NUMBER_ROBOTS = CONFIGURATION["team"]["yellow"]["number-robots"]
IS_YELLOW_TEAM = CONFIGURATION["team"]["is-yellow-team"]
IS_YELLOW_LEFT_TEAM = CONFIGURATION["team"]["is-yellow-left-team"]
IS_LEFT_TEAM = FieldHelper.isLeftTeam(IS_YELLOW_TEAM, IS_YELLOW_LEFT_TEAM)

FIELD_LENGTH = CONFIGURATION["field"]["length"]
FIELD_WIDTH = CONFIGURATION["field"]["width"]

TRAINING_TIME_STEP = CONFIGURATION["training"]["time-step"]

ROBOT_WIDTH = CONFIGURATION["robot"]["width"]

class Environment2(VSSBaseEnv):
    def __init__(self):
        super().__init__(
            field_type=0,
            n_robots_blue=TEAM_BLUE_NUMBER_ROBOTS,
            n_robots_yellow=TEAM_YELLOW_NUMBER_ROBOTS,
            time_step=TRAINING_TIME_STEP
        )

        n_obs = 11

        self.action_space = Box(
            low=-FIELD_LENGTH / 2,
            high=FIELD_LENGTH / 2,
            dtype=np.float32,
            shape=(2,)
        )

        self.observation_space = Box(
            low=-self.field.length/2,
            high=self.field.length/2,
            shape=(n_obs,)
        )

        self.episodeInitialTime = 0
        self.ballPast = None

    def get_state(self):
        observations = []

        observations.append(self.frame.ball)

        for i in range(TEAM_BLUE_NUMBER_ROBOTS):
            robot = self.frame.robots_blue[i]
            robot.yellow = False
            observations.append(robot)

        for i in range(TEAM_YELLOW_NUMBER_ROBOTS):
            robot = self.frame.robots_yellow[i]
            robot.yellow = True
            observations.append(robot)
        
        return np.array(observations)

    def _frame_to_observations(self):
        return self.get_state()

    def _get_commands(self, actions):
        self.episodeInitialTime = time.time()
        return actions
    
    def _calculate_reward(self):
        state = self.get_state()
        fieldData, _ = RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)

        robot = fieldData.robots[0]
        ball = fieldData.ball

        opponentGoalPosition = FieldHelper.getOpponentGoalPosition(FIELD_LENGTH, IS_LEFT_TEAM)

        # opponentGoalPosition = FieldHelper.getOpponentGoalPosition(FIELD_LENGTH, IS_LEFT_TEAM)

        # rewardAngleToGoal = TrainingUtils.rewardAngleToGoal(robot, ball, opponentGoalPosition)

        # rewardDistanceRobotBall = TrainingUtils.rewardDistanceRobotBall(robot, ball)

        rewardOfe = TrainingUtils.rOfe(robot, ball, opponentGoalPosition)

        rewardSpeed = TrainingUtils.rSpeed(self.ballPast, ball, opponentGoalPosition)

        rewardDist = TrainingUtils.rDist(fieldData.robots, ball)

        return 0.7 * rewardSpeed + 0.15 * rewardDist + 0.15 * rewardOfe
    
    def _has_goal_scored(self):
        if self.frame.ball.x > self.field.length / 2:
            return True
        elif self.frame.ball.x < -self.field.length / 2:
            return True
        return False
    
    def _is_goal_received(self):
        if self.frame.ball.x > self.field.length / 2:
            return IS_LEFT_TEAM
        elif self.frame.ball.x < -self.field.length / 2:
            return not IS_LEFT_TEAM
        return False

    def _calculate_reward_and_done(self):
        reward = self._calculate_reward()

        currentTime = time.time()

        done = self._has_goal_scored()

        print(f"Reward: {reward}")
        
        if done:
            reward += TrainingUtils.rewardGoal(
                self._is_goal_received(),
                self.episodeInitialTime,
                currentTime
            )
        
        return reward, done
    
    def _get_field_random_position(self):
        return FieldHelper.getFieldRandomPosition(FIELD_LENGTH, FIELD_WIDTH)
    
    def _get_random_theta(self):
        return FieldHelper.getRandomTheta()
    
    def _get_initial_positions_frame(self):
        pos_frame: Frame = Frame()

        robot_blue_pos_x, robot_blue_pos_y = self._get_field_random_position()
        robot_blue_theta = self._get_random_theta()

        robot_yellow_pos_x, robot_yellow_pos_y = self._get_field_random_position()
        robot_yellow_theta = self._get_random_theta()
        
        ball_pos_x, ball_pos_y = self._get_field_random_position()

        pos_frame.robots_blue[0] = Robot(
            id=0,
            x=robot_blue_pos_x,
            y=robot_blue_pos_y,
            theta=robot_blue_theta,
            yellow=False)

        pos_frame.robots_yellow[0] = Robot(
            id=0,
            x=-.4,
            y=-.3,
            theta=0,
            yellow=True)
        
        ball = Ball(x=ball_pos_x, y=ball_pos_y)

        pos_frame.ball = ball

        self.ballPast = RSoccerHelper.toBall(ball)
        
        return pos_frame
