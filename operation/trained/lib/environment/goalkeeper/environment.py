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
FIELD_GOAL_WIDTH = ConfigurationHelper.getFieldGoalWidth()
FIELD_GOAL_AREA_WIDTH = ConfigurationHelper.getFieldGoalAreaWidth()
FIELD_GOAL_AREA_LENGTH = ConfigurationHelper.getFieldGoalAreaLength()

TRAINING_TIME_STEP = ConfigurationHelper.getTrainingTimeStep()
TRAINING_EPISODE_DURATION = ConfigurationHelper.getTrainingEpisodeDuration()
TRAINING_VELOCITY_CLIP_VALUE = ConfigurationHelper.getTrainingVelocityClipValue()

ROBOT_WIDTH = ConfigurationHelper.getRobotWidth()

class Environment(VSSBaseEnv):
    def __init__(self):
        super().__init__(
            field_type=0,
            n_robots_blue=TEAM_BLUE_NUMBER_ROBOTS,
            n_robots_yellow=TEAM_YELLOW_NUMBER_ROBOTS,
            time_step=TRAINING_TIME_STEP
        )

        maxFieldX = FIELD_LENGTH / 2
        maxFieldY = FIELD_WIDTH / 2

        self.action_space = Box(
            low=np.array([-maxFieldX, -maxFieldY]),
            high=np.array([maxFieldX, maxFieldY]),
            dtype=np.float64,
            shape=(2,)
        )

        self.observation_space = Box(
            low=np.array([
                -maxFieldX,
                -maxFieldY,
                -math.pi,

                -maxFieldX,
                -maxFieldY,
                -TRAINING_VELOCITY_CLIP_VALUE,
                -TRAINING_VELOCITY_CLIP_VALUE
            ]),
            high=np.array([
                maxFieldX,
                maxFieldY,
                math.pi,

                maxFieldX,
                maxFieldY,
                TRAINING_VELOCITY_CLIP_VALUE,
                TRAINING_VELOCITY_CLIP_VALUE
            ]),
            dtype=np.float64,
            shape=(7,)
        )

        self.goalReferencePosition = None
        self.error = 0
        self.opponentError = 0
        self.episodeInitialTime = 0
        self.distanceOpponentRobotToGoal = 0
        self.distanceRobotToGoal = 0
        self.speedFactor = 0

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
        fieldData, _ = self._get_field_datas()

        ball = fieldData.ball
        robot = fieldData.robots[0]

        velocityClip = lambda x: np.clip(
            x,
            -TRAINING_VELOCITY_CLIP_VALUE,
            TRAINING_VELOCITY_CLIP_VALUE
        )

        observations = [
            robot.position.x,
            robot.position.y,
            robot.position.theta,
            ball.position.x,
            ball.position.y,
            velocityClip(ball.velocity.x),
            velocityClip(ball.velocity.y)
        ]
        
        return np.array(observations)

    def _get_commands(self, actions):
        rSoccer_robot_actions = []
        fieldData, opponentFieldData = self._get_field_datas()

        robot = fieldData.robots[0]

        (leftSpeed, rightSpeed, self.error) = MotionUtils.goToPoint(
            robot,
            (actions[0], actions[1]),
            self.error
        )

        rSoccer_robot_actions.append(
            RSoccerHelper.get_rsoccer_robot_action(
                0,
                IS_YELLOW_TEAM,
                leftSpeed,
                rightSpeed
            )
        )

        opponentRobot = opponentFieldData.robots[0]

        (leftSpeed, rightSpeed, self.opponentError) = MotionUtils.goToPoint(
            opponentRobot,
            self.get_goal_reference_position(),
            self.opponentError
        )

        getSpeed = lambda x: x * self.speedFactor

        rSoccer_robot_actions.append(
            RSoccerHelper.get_rsoccer_robot_action(
                0,
                not IS_YELLOW_TEAM,
                getSpeed(leftSpeed),
                getSpeed(rightSpeed)
            )
        )

        return rSoccer_robot_actions
    
    def calculate_goal_intersection(self):
        fieldData, _ = self._get_field_datas()
        ownGoalPosition = self._get_own_goal_position()

        goal_line_x = ownGoalPosition[0]

        ball = fieldData.ball

        ball_x, ball_y = ball.get_position_tuple()
        ball_vx, ball_vy = ball.velocity.x, ball.velocity.y

        if ball_vx == 0:
            return None
        
        time_to_goal = (goal_line_x - ball_x) / ball_vx
        intersection_y = ball_y + ball_vy * time_to_goal
        
        if abs(intersection_y) <= FIELD_GOAL_WIDTH / 2:
            return goal_line_x, intersection_y
        else:
            return None
    
    def _get_field_datas(self):
        state = self._get_state()
        return RSoccerHelper.getFieldDatas(state, IS_YELLOW_TEAM)
    
    def _get_goal_target_position(self):
        fieldData, _ = self._get_field_datas()
        ball = fieldData.ball

        targetPositionXMargin = 0.05

        ownGoalPosition = self._get_own_goal_position()
        goalTargetPosition = self.calculate_goal_intersection()

        if goalTargetPosition is None:
            robotMaxY = FIELD_GOAL_WIDTH / 2 - ROBOT_WIDTH / 2
            x = ownGoalPosition[0]
            y = np.clip(
                ball.position.y,
                -robotMaxY,
                robotMaxY
            )
        else:
            x, y = goalTargetPosition

        return (
            x + targetPositionXMargin,
            y
        )
    
    def _calculate_reward(self):
        fieldData, _ = self._get_field_datas()
        robot = fieldData.robots[0]

        goalTargetPosition = self._get_goal_target_position()

        distance = GeometryUtils.distance(
            robot.get_position_tuple(),
            goalTargetPosition
        )
        
        reward = -1 if distance > 1 else -distance

        if self._is_done():
            clip = lambda x: np.clip(x, 1, 5)
            factor = 2

            def getReward(positive, distanceReward):
                return factor * distanceReward if positive else -factor * distanceReward

            if self._has_goal_scored() and self._is_goal_received():
                distanceReward = clip(self.distanceOpponentRobotToGoal / self.distanceRobotToGoal)
                reward += getReward(False, distanceReward)
            else:
                distanceReward = clip(self.distanceRobotToGoal / self.distanceOpponentRobotToGoal)
                reward += getReward(True, distanceReward)

        return reward
                
        
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
        if time.time() - self.episodeInitialTime > TRAINING_EPISODE_DURATION:
            return True
        return False

    def _calculate_reward_and_done(self):
        reward = self._calculate_reward()
        done = self._is_done()
        reward = self._calculate_reward()
        
        return reward, done
    
    def _get_field_random_position(self):
        return FieldHelper.getFieldRandomPosition(FIELD_LENGTH - 0.1, FIELD_WIDTH - 0.1)
    
    def _get_random_theta(self):
        return FieldHelper.getRandomTheta() * (180 / math.pi)
    
    def _get_random_ball(self):
        ball_pos_x, ball_pos_y = self._get_field_random_position()

        return Ball(x=ball_pos_x, y=ball_pos_y)
    
    def _get_own_goal_position(self):
        return FieldHelper.getOwnGoalPosition(FIELD_LENGTH, IS_LEFT_TEAM)
    
    def _get_opponent_goal_position(self):
        return FieldHelper.getOpponentGoalPosition(FIELD_LENGTH, IS_LEFT_TEAM)
    
    def get_goal_reference_position(self):
        return self.goalReferencePosition
    
    def _set_goal_reference_position(self):
        ownGoalPosition = self._get_own_goal_position()

        self.goalReferencePosition = (
            ownGoalPosition[0],
            ownGoalPosition[1] + GeometryUtils.getRandomUniform(-FIELD_GOAL_WIDTH / 2, FIELD_GOAL_WIDTH / 2)
        )
    
    def _get_initial_positions_frame(self):
        pos_frame: Frame = Frame()

        self._set_goal_reference_position()

        ownGoalPosition = self._get_own_goal_position()
        robotBluePosition = self._get_field_random_position()

        robotBlueTheta = math.atan2(
            self.goalReferencePosition[1] - robotBluePosition[1],
            self.goalReferencePosition[0] - robotBluePosition[0]
        ) * (180 / math.pi)

        robotYellowPosition = (
            ownGoalPosition[0] +
            0.05 +
            GeometryUtils.getRandomUniform(0, FIELD_GOAL_AREA_LENGTH),
            ownGoalPosition[1] +
            GeometryUtils.getRandomUniform(-FIELD_GOAL_AREA_WIDTH / 2, FIELD_GOAL_AREA_WIDTH / 2)
        )

        robot_yellow = Robot(
            id=0,
            x=robotYellowPosition[0],
            y=robotYellowPosition[1],
            theta=self._get_random_theta(),
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
            self.goalReferencePosition,
            robotBluePosition
        )
        
        ball = Ball(x=ballPosition[0], y=ballPosition[1])

        pos_frame.ball = ball

        self.episodeInitialTime = time.time()
        self.speedFactor = GeometryUtils.getRandomUniform(1 / 5, 1)

        self.distanceOpponentRobotToGoal = GeometryUtils.distance(
            robotBluePosition,
            ownGoalPosition
        )
        self.distanceRobotToGoal = GeometryUtils.distance(
            robotYellowPosition,
            ownGoalPosition
        )
        
        return pos_frame
