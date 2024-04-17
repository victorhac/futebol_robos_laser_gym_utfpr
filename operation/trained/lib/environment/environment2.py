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

        self.robotPast = fieldData.robots[0]

        return actions
    
    def calcular_vetor_movimento(self, v_esquerda, v_direita, angulo):
        v_linear = (v_esquerda + v_direita) / 2.0

        vetor_x = v_linear * math.cos(angulo)
        vetor_y = v_linear * math.sin(angulo)

        angulo_vetor = angulo + (math.pi / 2.0)

        return vetor_x, vetor_y, angulo_vetor
    
    def _calculate_reward(self):
        state = self.get_state()
        fieldData, _ = RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)

        robot = fieldData.robots[0]
        ball = fieldData.ball

        rSoccerRobot = self.frame.robots_yellow[0] if IS_YELLOW_TEAM else self.frame.robots_blue[0]

        distance = GeometryUtils.distance((robot.position.x, robot.position.y), (self.frame.ball.x, self.frame.ball.y))

        rewardDistance = -1 if distance >= 1 else 1 - distance

        vetor_1_x, vetor_1_y, _ = self.calcular_vetor_movimento(rSoccerRobot.v_x, rSoccerRobot.v_y, rSoccerRobot.theta)

        print(f"Speed 1: {vetor_1_x}, Speed 2: {vetor_1_y}")

        oponnentGoalPosition = FieldHelper.getOwnGoalPosition(FIELD_LENGTH, IS_YELLOW_TEAM)

        vetor_2_x, vetor_2_y = oponnentGoalPosition[0] - ball.position.x, oponnentGoalPosition[1] - ball.position.y

        angle = math.atan2(vetor_2_y - vetor_1_y, vetor_2_x - vetor_1_x)

        return angle

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
        pos_frame.ball = Ball(x=0.1, y=0.15)

        for i in range(self.n_robots_blue):
            pos_frame.robots_blue[i] = Robot(id=i, x=(0. + (i+1) * intervalBlue), y=0., theta=0, yellow=False)

        for i in range(self.n_robots_yellow):
            self.robotPast = Robot(id=i, x=(0. - (i+1) * intervalYellow), y=0., theta=0, yellow=True)
            pos_frame.robots_yellow[i] = Robot(id=i, x=(0. - (i+1) * intervalYellow), y=0., theta=0, yellow=True)
        
        return pos_frame
