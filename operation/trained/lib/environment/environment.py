import math
import time
import numpy as np
from gym.spaces import Box

from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv

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

class Environment(VSSBaseEnv):
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
        # observations = []

        # observations.append(self.frame.ball)

        # ball = self.frame.ball
        # robot = self.frame.robots_yellow[0]

        # observations = [
        #     ball.x,
        #     ball.y,
        #     ball.v_x,
        #     ball.v_y,
        #     robot.x,
        #     robot.y,
        #     math.sin(robot.theta),
        #     math.cos(robot.theta),
        #     robot.v_x,
        #     robot.v_y,
        #     robot.v_theta
        # ]
        
        # return np.array(observations)
        return self.get_state()

    def _get_commands(self, actions):
        state = self.get_state()

        fieldData, _ = RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)

        # robot = fieldData.robots[0]

        # targetPosition = (actions[0][0], actions[0][1])

        # velocities = MotionUtils.goToPoint(robot, targetPosition, self.error)

        # (leftSpeed, rightSpeed, self.error) = velocities

        self.ballPast = self.frame.ball

        # return [RSoccerHelper.getRSoccerRobotAction(0, IS_YELLOW_TEAM, leftSpeed, rightSpeed)]
        self.robotPast = fieldData.robots[0]

        return actions

    def _calculate_reward_and_done(self):
        state = self.get_state()
        fieldData, _ = RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)

        # reward = TrainingUtils.rewardAttack(
        #     0,
        #     fieldData.robots,
        #     RSoccerHelper.toBall(self.ballPast),
        #     fieldData.ball,
        #     FieldHelper.getOpponentGoalPosition(FIELD_LENGTH, IS_YELLOW_TEAM),
        # )

        reward = TrainingUtils.reward(
            fieldData.robots[0],
            self.robotPast,
            fieldData.ball,
            FieldHelper.getOpponentGoalPosition(FIELD_LENGTH, IS_YELLOW_TEAM))

        print(f"Reward: {reward}")

        currentTime = time.time()

        isTimeExceded = False#currentTime - self.episodeInitialTime > self.episodeTime

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
