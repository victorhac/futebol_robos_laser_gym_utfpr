import numpy as np
from gym.spaces import Box

from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv

from ..helpers.configuration_helper import ConfigurationHelper

CONFIGURATION = ConfigurationHelper.getConfiguration()

TEAM_BLUE_NUMBER_ROBOTS = CONFIGURATION["team"]["blue"]["number-robots"]
TEAM_YELLOW_NUMBER_ROBOTS = CONFIGURATION["team"]["yellow"]["number-robots"]

FIELD_LENGTH = CONFIGURATION["field"]["length"]

TRAINING_TIME_STEP = CONFIGURATION["training"]["time-step"]

class Environment(VSSBaseEnv):
    def __init__(self):
        super().__init__(
            field_type=0,
            n_robots_blue=TEAM_BLUE_NUMBER_ROBOTS,
            n_robots_yellow=TEAM_YELLOW_NUMBER_ROBOTS,
            time_step=TRAINING_TIME_STEP
        )
        n_obs = 4
        self.action_space = Box(low=-1, high=1, shape=(2, ))
        self.observation_space = Box(low=-self.field.length/2,\
            high=self.field.length/2,shape=(n_obs, ))

    def _frame_to_observations(self):
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

    def _get_commands(self, actions):
        return actions

    def _calculate_reward_and_done(self):
        if self.frame.ball.x > self.field.length / 2 \
            and abs(self.frame.ball.y) < self.field.goal_width / 2:
            reward, done = 1, True
        else:
            reward, done = 0, False
        return reward, done
    
    def _get_initial_positions_frame(self):
        intervalBlue = FIELD_LENGTH / (self.n_robots_blue * 2)
        intervalYellow = FIELD_LENGTH / (self.n_robots_yellow * 2)

        pos_frame: Frame = Frame()
        pos_frame.ball = Ball(x=0, y=-0.15)

        for i in range(self.n_robots_blue):
            pos_frame.robots_blue[i] = Robot(id=i, x=(0. + (i+1) * intervalBlue), y=0., theta=0, yellow=False)

        for i in range(self.n_robots_yellow):
            pos_frame.robots_yellow[i] = Robot(id=i, x=(0. - (i+1) * intervalYellow), y=0., theta=0, yellow=True)
        
        return pos_frame
