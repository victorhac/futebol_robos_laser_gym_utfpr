import math
import time
import numpy as np

from gym.spaces import Box

from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv

from ...motion.motion_utils import MotionUtils

from ...geometry.geometry_utils import GeometryUtils

from ...helpers.field_helper import FieldHelper

from ...helpers.rsoccer_helper import RSoccerHelper

from ...helpers.configuration_helper import ConfigurationHelper

TEAM_BLUE_NUMBER_ROBOTS = ConfigurationHelper.getTeamBlueNumberRobots()
TEAM_YELLOW_NUMBER_ROBOTS = ConfigurationHelper.getTeamYellowNumberRobots()
IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()

FIELD_LENGTH = ConfigurationHelper.getFieldLength()
FIELD_WIDTH = ConfigurationHelper.getFieldWidth()

TRAINING_TIME_STEP = ConfigurationHelper.getTrainingTimeStep()
EPISODE_DURATION = ConfigurationHelper.getTrainingEpisodeDuration()

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
            dtype=np.float32,
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
            dtype=np.float32,
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
        observations = []

        observations.append(self.frame.ball)

        fieldData, _ = self._get_field_datas()

        ball = fieldData.ball
        robot = fieldData.robots[0]

        observations = [
            robot.position.x,
            robot.position.y,
            robot.position.theta,
            ball.position.x,
            ball.position.y,
        ]
        
        return np.array(observations)

    def _get_commands(self, actions):
        fieldData, _ = self._get_field_datas()

        robot = fieldData.robots[0]

        targetPosition = (actions[0], actions[1])

        velocities = MotionUtils.goToPoint(robot, targetPosition, self.error)

        (leftSpeed, rightSpeed, self.error) = velocities

        return [RSoccerHelper.getRSoccerRobotAction(0, IS_YELLOW_TEAM, leftSpeed, rightSpeed)]
    
    def _get_field_datas(self):
        state = self._get_state()
        return RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)
    
    def distance_to_ball_reward(self):
        fieldData, _ = self._get_field_datas()
        robot = fieldData.robots[0]
        ball = fieldData.ball

        distanceToBall = GeometryUtils.distance(
            (ball.position.x, ball.position.y),
            (robot.position.x, robot.position.y)
        )

        return -distanceToBall if distanceToBall < 1 else -1
    
    def robot_towards_ball_reward(self):
        state = self._get_state()
        fieldData, _ = RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)
        robot = fieldData.robots[0]
        ball = fieldData.ball
        position = robot.position

        positionX = position.x
        positionY = position.y
        robotAngle = position.theta

        angleToTarget = math.atan2(ball.position.y - positionY, ball.position.x - positionX)

        error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)

        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
            robotAngle = GeometryUtils.normalizeInPI(robotAngle + math.pi)
            error = GeometryUtils.smallestAngleDiff(angleToTarget, robotAngle)

        return -abs(error / math.pi)
    
    def _is_close_to_ball(self):
        fieldData, _ = self._get_field_datas()
        robot = fieldData.robots[0]
        ball = fieldData.ball

        return GeometryUtils.isClose(
            robot.get_position_tuple(),
            ball.get_position_tuple(),
            0.1
        )
    
    def _calculate_reward(self):
        reward = .5 * self.robot_towards_ball_reward() + .5 * self.distance_to_ball_reward()

        if self._is_close_to_ball():
            reward += 13 * ((EPISODE_DURATION - (time.time() - self.episodeInitialTime)) / EPISODE_DURATION) + 2

        return reward
            
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

        done = self._has_goal_scored() or\
            time.time() - self.episodeInitialTime > EPISODE_DURATION or\
            self._is_close_to_ball()
        
        return reward, done
    
    def _get_field_random_position(self):
        return FieldHelper.getFieldRandomPosition(FIELD_LENGTH - .2, FIELD_WIDTH - .2)
    
    def _get_random_theta(self):
        return FieldHelper.getRandomTheta()
    
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
    
    def _get_initial_positions_frame(self):
        pos_frame: Frame = Frame()

        pos_frame.robots_blue[0] = self._get_random_robot(0, False)

        robot_yellow = self._get_random_robot(0, True)

        pos_frame.robots_yellow[0] = robot_yellow
        
        ball = self._get_random_ball()

        pos_frame.ball = ball

        self.episodeInitialTime = time.time()
        
        return pos_frame