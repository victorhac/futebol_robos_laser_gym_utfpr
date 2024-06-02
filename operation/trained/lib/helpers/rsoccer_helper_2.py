from .configuration_helper import ConfigurationHelper
from ..domain.field_data import FieldData
from ..domain.robot import Robot
from ..domain.ball import Ball
from ..geometry.geometry_utils import GeometryUtils

from rsoccer_gym.Entities import Robot as RSoccerRobot, Ball as RSoccerBall
from rsoccer_gym.Entities.Frame import Frame

import numpy as np
import math

class RSoccerHelper2:
    FIELD_WIDTH = ConfigurationHelper.getFieldWidth()
    FIELD_LENGTH = ConfigurationHelper.getFieldLength()
    PENALTY_LENGTH = 0.15
    ROBOT_WHEEL_RADIUS = ConfigurationHelper.getRobotWheelRadius()
    ROBOT_MAX_RPM = 440
    NORM_BOUNDS = 1.2
    V_WHEEL_DEADZONE = 0.05

    MAX_POS = max(
        FIELD_WIDTH / 2,
        (FIELD_LENGTH / 2) + PENALTY_LENGTH
    )

    MAX_V = (ROBOT_MAX_RPM / 60) * 2 * np.pi * ROBOT_WHEEL_RADIUS
    MAX_W = np.rad2deg(MAX_V / 0.04)

    @staticmethod
    def get_corrected_angle(angle: float):
        angleRadians = np.deg2rad(angle)
        return GeometryUtils.normalizeInPI(angleRadians)

    @staticmethod
    def to_robot(rSoccerRobot: RSoccerRobot):
        robot = Robot()

        robot.position.x = rSoccerRobot.x
        robot.position.y = rSoccerRobot.y
        robot.position.theta = RSoccerHelper2.get_corrected_angle(rSoccerRobot.theta)

        robot.velocity.x = rSoccerRobot.v_x
        robot.velocity.y = rSoccerRobot.v_y
        robot.velocity.theta = RSoccerHelper2.get_corrected_angle(rSoccerRobot.v_theta)
        
        return robot
    
    @staticmethod
    def to_ball(rSoccerBall: RSoccerBall):
        ball = Ball()

        ball.position.x = rSoccerBall.x
        ball.position.y = rSoccerBall.y
        ball.velocity.x = rSoccerBall.v_x
        ball.velocity.y = rSoccerBall.v_y

        return ball
    
    @staticmethod
    def get_rsoccer_robot_action(
        id: int,
        isYellowTeam: bool,
        leftMotorSpeed: float,
        rightMotorSpeed: float
    ):
        return RSoccerRobot(
            yellow=isYellowTeam,
            id=id,
            v_wheel0=leftMotorSpeed,
            v_wheel1=rightMotorSpeed
        )
    
    @staticmethod
    def get_field_datas(next_state, isYellowTeam: bool):
        fieldData = FieldData()
        opponentFieldData = FieldData()

        ball = RSoccerHelper2.to_ball(next_state[0])

        blueTeam = []
        yellowTeam = []

        for i in range(1, len(next_state)):
            robot = next_state[i]
            if robot.yellow is not None:
                if robot.yellow:
                    yellowTeam.append(RSoccerHelper2.to_robot(robot))
                else:
                    blueTeam.append(RSoccerHelper2.to_robot(robot))

        fieldData.ball = ball
        opponentFieldData.ball = ball

        if isYellowTeam:
            fieldData.robots = yellowTeam
            fieldData.foes = blueTeam

            opponentFieldData.robots = blueTeam
            opponentFieldData.foes = yellowTeam
        else:
            fieldData.robots = blueTeam
            fieldData.foes = yellowTeam

            opponentFieldData.robots = yellowTeam
            opponentFieldData.foes = blueTeam

        return fieldData, opponentFieldData
    
    @staticmethod
    def get_field_data(frame: Frame, is_yellow_team: bool):
        field_data = FieldData()

        ball = RSoccerHelper2.to_ball(frame.ball)

        blue_team = [RSoccerHelper2.to_robot(frame.robots_blue[i]) for i in frame.robots_blue]
        yellow_team = [RSoccerHelper2.to_robot(frame.robots_yellow[i]) for i in frame.robots_yellow]

        field_data.ball = ball

        if is_yellow_team:
            field_data.robots = yellow_team
            field_data.foes = blue_team
        else:
            field_data.robots = blue_team
            field_data.foes = yellow_team

        return field_data

    @staticmethod
    def norm_pos(pos):
        return np.clip(
            pos / RSoccerHelper2.MAX_POS,
            -RSoccerHelper2.NORM_BOUNDS,
            RSoccerHelper2.NORM_BOUNDS)

    @staticmethod
    def norm_v(v):
        return np.clip(
            v / RSoccerHelper2.MAX_V,
            -RSoccerHelper2.NORM_BOUNDS,
            RSoccerHelper2.NORM_BOUNDS)

    @staticmethod
    def norm_w(w):
        return (w / math.pi) * RSoccerHelper2.NORM_BOUNDS
    
    @staticmethod
    def get_attacker_observations(
        field_data: FieldData,
        is_left_team: bool,
        robot_id: int
    ):
        return RSoccerHelper2.get_default_observations(field_data, is_left_team, robot_id)
    
    @staticmethod
    def get_defensor_observations(
        field_data: FieldData,
        is_left_team: bool,
        robot_id: int
    ):
        return RSoccerHelper2.get_default_observations(field_data, is_left_team, robot_id)
    
    @staticmethod
    def get_goalkeeper_observations(
        field_data: FieldData,
        is_left_team: bool,
        robot_id: int
    ):
        observations = RSoccerHelper2.get_ball_observations(field_data, is_left_team)

        robot = field_data.robots[robot_id]

        observations.extend(RSoccerHelper2.get_own_team_robot_observations(is_left_team, robot))

        return np.array(observations, dtype=np.float32)
    
    @staticmethod
    def get_own_team_robot_observations(
        is_left_team: bool,
        robot: Robot
    ):
        def norm_pos(pos): return RSoccerHelper2.norm_pos(pos)
        def norm_v(v): return RSoccerHelper2.norm_v(v)
        def norm_w(w): return RSoccerHelper2.norm_w(w)
        def correct_to_side(x): return x if is_left_team else -x

        theta = robot.position.theta
        return [
            norm_pos(correct_to_side(robot.position.x)),
            norm_pos(robot.position.y),
            np.sin(theta),
            correct_to_side(np.cos(theta)),
            norm_v(correct_to_side(robot.velocity.x)),
            norm_v(robot.velocity.y),
            norm_w(correct_to_side(robot.velocity.theta))
        ]
    
    @staticmethod
    def get_opponent_team_robot_observations(
        is_left_team: bool,
        robot: Robot
    ):
        def norm_pos(pos): return RSoccerHelper2.norm_pos(pos)
        def norm_v(v): return RSoccerHelper2.norm_v(v)
        def norm_w(w): return RSoccerHelper2.norm_w(w)
        def correct_to_side(x): return x if is_left_team else -x

        return [
            norm_pos(correct_to_side(robot.position.x)),
            norm_pos(robot.position.y),
            norm_v(correct_to_side(robot.velocity.x)),
            norm_v(robot.velocity.y),
            norm_w(correct_to_side(robot.velocity.theta))
        ]
    
    @staticmethod
    def get_ball_observations(
        field_data: FieldData,
        is_left_team: bool
    ):
        ball = field_data.ball

        def norm_pos(pos): return RSoccerHelper2.norm_pos(pos)
        def norm_v(v): return RSoccerHelper2.norm_v(v)
        def correct_to_side(x): return x if is_left_team else -x

        return [
            norm_pos(correct_to_side(ball.position.x)),
            norm_pos(ball.position.y),
            norm_v(correct_to_side(ball.velocity.x)),
            norm_v(ball.velocity.y)
        ]
    
    @staticmethod
    def get_default_observations(
        field_data: FieldData,
        is_left_team: bool,
        robot_id: int
    ):
        observation = []

        observation.extend(RSoccerHelper2.get_ball_observations(field_data, is_left_team))

        robot = field_data.robots[robot_id]
        observation.extend(RSoccerHelper2.get_own_team_robot_observations(is_left_team, robot))

        for i in range(len(field_data.robots)):
            if i != robot_id:
                robot = field_data.robots[i]
                observation.extend(RSoccerHelper2.get_own_team_robot_observations(is_left_team, robot))

        for robot in field_data.foes:
            observation.extend(RSoccerHelper2.get_opponent_team_robot_observations(is_left_team, robot))

        return np.array(observation, dtype=np.float32)
    
    @staticmethod
    def actions_to_v_wheels(actions: np.ndarray):
        max_v = 10
        v_wheel_deadzone = RSoccerHelper2.V_WHEEL_DEADZONE
        rbt_wheel_radius = RSoccerHelper2.ROBOT_WHEEL_RADIUS

        left_wheel_speed = actions[0] * max_v
        right_wheel_speed = actions[1] * max_v

        left_wheel_speed, right_wheel_speed = np.clip(
            (left_wheel_speed, right_wheel_speed),
            -max_v,
            max_v)

        if -v_wheel_deadzone < left_wheel_speed < v_wheel_deadzone:
            left_wheel_speed = 0

        if -v_wheel_deadzone < right_wheel_speed < v_wheel_deadzone:
            right_wheel_speed = 0

        left_wheel_speed /= rbt_wheel_radius
        right_wheel_speed /= rbt_wheel_radius

        return left_wheel_speed, right_wheel_speed