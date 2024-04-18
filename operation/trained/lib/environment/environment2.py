import math
import time
import numpy as np
from gym.spaces import Box

from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv

from ..geometry.geometry_utils import GeometryUtils

from ..helpers.field_helper import FieldHelper
from ..training.training_utils import TrainingUtils

from ..helpers.rsoccer_helper import RSoccerHelper
from ..motion.motion_utils import MotionUtils

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
    def __init__(self, episodeTime=10):
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

        self.error = 0
        self.episodeTime = episodeTime
        self.episodeInitialTime = 0
        self.ballPast = Ball(x=0, y=0)

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
        state = self.get_state()
        fieldData, _ = RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)

        return actions
    
    def calculate_vector_coordinates(self, x, y, angle, magnitude):
        # verificar para o time da direita
        delta_x = magnitude * math.cos(angle)
        delta_y = magnitude * math.sin(angle)
        
        new_x = x + delta_x
        new_y = y + delta_y
        
        return new_x, new_y
    
    def dot_product(self, v1, v2):
        return sum((a * b) for a, b in zip(v1, v2))

    def vector_magnitude(self, v):
        return math.sqrt(sum(a**2 for a in v))

    def angle_between_vectors(self, v1, v2):
        dot_prod = self.dot_product(v1, v2)
        mag_v1 = self.vector_magnitude(v1)
        mag_v2 = self.vector_magnitude(v2)

        cosine_angle = dot_prod / (mag_v1 * mag_v2)

        return math.acos(cosine_angle)

    
    def _calculate_reward(self):
        state = self.get_state()
        fieldData, _ = RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)

        robot = fieldData.robots[0]
        ball = fieldData.ball

        rSoccerRobot = self.frame.robots_yellow[0] if IS_YELLOW_TEAM else self.frame.robots_blue[0]

        distance = GeometryUtils.distance((robot.position.x, robot.position.y), (self.frame.ball.x, self.frame.ball.y))

        rewardDistance = -1 if distance >= 1 else 1 - distance

        opponentGoalPosition = FieldHelper.getOpponentGoalPosition(FIELD_LENGTH,  IS_LEFT_TEAM)

        angle = math.atan2(robot.position.y - opponentGoalPosition[1], robot.position.x - opponentGoalPosition[0])

        robot_vector = self.calculate_vector_coordinates(robot.position.x, robot.position.y, robot.position.theta, 1)

        vector1 = [robot_vector[0] - robot.position.x, robot_vector[1] - robot.position.y]
        vector2 = [opponentGoalPosition[0] - ball.position.x, opponentGoalPosition[1] - ball.position.y]

        angle = self.angle_between_vectors(vector1, vector2)

        print("Angle: ", angle)

        return rewardDistance

    def _calculate_reward_and_done(self):
        state = self.get_state()
        fieldData, _ = RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)

        reward = self._calculate_reward()

        currentTime = time.time()

        isTimeExceded = False

        done = self.frame.ball.x > self.field.length / 2 \
            or self.frame.ball.x < -self.field.length / 2 \
            or isTimeExceded
        
        if done and not isTimeExceded:
            reward += TrainingUtils.rewardGoal(
                IS_LEFT_TEAM if self.frame.ball.x > self.field.length / 2 else not IS_LEFT_TEAM,
                self.episodeInitialTime,
                currentTime
            )
        
        return reward, done
    
    def _get_initial_positions_frame(self):
        intervalBlue = FIELD_LENGTH / (self.n_robots_blue * 2)
        intervalYellow = FIELD_LENGTH / (self.n_robots_yellow * 2)

        pos_frame: Frame = Frame()
        pos_frame.ball = Ball(x=0, y=1/7)

        pos_frame.robots_blue[0] = Robot(id=0, x=0.5, y=0., theta=0, yellow=False)

        pos_frame.robots_yellow[0] = Robot(id=0, x=-0.2, y=0.2, theta=0, yellow=True)
        
        return pos_frame
