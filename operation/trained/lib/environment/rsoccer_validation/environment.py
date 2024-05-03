import math
import time
import random
import numpy as np

from gym.spaces import Box

from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv

from ...geometry.geometry_utils import GeometryUtils

from ...helpers.field_helper import FieldHelper
from ...helpers.rsoccer_helper import RSoccerHelper
from ...helpers.configuration_helper import ConfigurationHelper

from ...training.training_utils import TrainingUtils

TEAM_BLUE_NUMBER_ROBOTS = ConfigurationHelper.getTeamBlueNumberRobots()
TEAM_YELLOW_NUMBER_ROBOTS = ConfigurationHelper.getTeamYellowNumberRobots()
IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()

FIELD_LENGTH = ConfigurationHelper.getFieldLength()
FIELD_WIDTH = ConfigurationHelper.getFieldWidth()
FIELD_GOAL_WIDTH = ConfigurationHelper.getFieldGoalWidth()
FIELD_GOAL_AREA_WIDTH = ConfigurationHelper.getFieldGoalAreaWidth()
FIELD_GOAL_AREA_WIDTH = ConfigurationHelper.getFieldGoalAreaWidth()

TRAINING_TIME_STEP = ConfigurationHelper.getTrainingTimeStep()

ROBOT_WIDTH = ConfigurationHelper.getRobotWidth()

class Environment(VSSBaseEnv):
    def __init__(self):
        super().__init__(
            field_type=0,
            n_robots_blue=TEAM_BLUE_NUMBER_ROBOTS,
            n_robots_yellow=TEAM_YELLOW_NUMBER_ROBOTS,
            time_step=TRAINING_TIME_STEP
        )

        self.action_space = Box(
            low=np.array([-FIELD_LENGTH / 2, -FIELD_WIDTH / 2]),
            high=np.array([FIELD_LENGTH / 2, FIELD_WIDTH / 2]),
            dtype=np.float64,
            shape=(2,)
        )

        self.observation_space = Box(
            low=np.array([
                -FIELD_LENGTH / 2,
                -FIELD_WIDTH / 2,
                -math.pi,
                -FIELD_LENGTH / 2,
                -FIELD_WIDTH / 2
            ]),
            high=np.array([
                FIELD_LENGTH / 2,
                FIELD_WIDTH / 2,
                math.pi,
                FIELD_LENGTH / 2,
                FIELD_WIDTH / 2
            ]),
            dtype=np.float64,
            shape=(5,)
        )

        self.error = 0
        self.episodeInitialTime = 0
        self.lastBall = None
        self.lastRobot = None
        self.lastRewardOfe = None

    def _get_state(self):
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
        return self._get_state()

    def _get_commands(self, actions):
        return actions
    
    def _get_field_datas(self):
        state = self._get_state()
        return RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)
    
    def _calculate_reward(self):
        fieldData, _ = self._get_field_datas()

        return 0
        
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

        done = self._has_goal_scored()

        print(f"Reward: {reward}")
        
        reward = self._calculate_reward()
        
        return reward, done
    
    def _get_field_random_position(self):
        return FieldHelper.getFieldRandomPosition(FIELD_LENGTH - 0.1, FIELD_WIDTH - 0.1)
    
    def _get_random_theta(self):
        return FieldHelper.getRandomTheta() * (180 / math.pi)
    
    def _get_random_robot(self, id: float, is_yellow_team: bool):
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
        return FieldHelper.getOwnGoalPosition(FIELD_LENGTH, IS_LEFT_TEAM)
    
    def _get_opponent_goal_position(self):
        return FieldHelper.getOpponentGoalPosition(FIELD_LENGTH, IS_LEFT_TEAM)
    
    def _get_initial_positions_frame(self):
        pos_frame: Frame = Frame()

        ownGoalPosition = self._get_own_goal_position()
        robotBluePosition = self._get_field_random_position()

        goalReferencePosition = (
            ownGoalPosition[0],
            ownGoalPosition[1] + GeometryUtils.getRandomUniform(-FIELD_GOAL_WIDTH / 2, FIELD_GOAL_WIDTH / 2)
        )

        robotBlueTheta = math.atan2(
            goalReferencePosition[1] - robotBluePosition[1],
            goalReferencePosition[0] - robotBluePosition[0]
        ) * (180 / math.pi)

        robot_yellow = Robot(
            id=0,
            x=ownGoalPosition[0] + 0.05,
            y=ownGoalPosition[1],
            theta=90,
            yellow=True)
        
        robot_blue = Robot(
            id=0,
            x=robotBluePosition[0],
            y=robotBluePosition[1],
            theta=robotBlueTheta,
            yellow=False)

        pos_frame.robots_yellow[0] = robot_yellow
        pos_frame.robots_blue[0] = robot_blue

        ballPosition = GeometryUtils.getMidpoint(
            goalReferencePosition,
            robotBluePosition
        )
        
        ball = Ball(x=ballPosition[0], y=ballPosition[1])

        pos_frame.ball = ball
        
        return pos_frame
