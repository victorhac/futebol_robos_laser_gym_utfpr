import time
import numpy as np
from stable_baselines3 import PPO
from lib.comm.vision import ProtoVision
from lib.comm.control import ProtoControl

from lib.helpers.configuration_helper import ConfigurationHelper
from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils

from lib.domain.field_data import FieldData

from lib.comm.replacer import ReplacerComm
from lib.domain.robot import Robot

FIRASIM_CONTROL_IP = ConfigurationHelper.getFIRASimControlIp()
FIRASIM_CONTROL_PORT = ConfigurationHelper.getFIRASimControlPort()
FIRASIM_VISION_IP = ConfigurationHelper.getFIRASimVisionIp()
FIRASIM_VISION_PORT = ConfigurationHelper.getFIRASimVisionPort()

IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()

attacker_model = PPO.load("models/attacker/PPO/2024_6_1_0_39_1/PPO_model")

def getProtoVision(isYellowTeam: bool):
    fieldData = FieldData()
    return ProtoVision(
        team_color_yellow=isYellowTeam,
        field_data=fieldData,
        vision_ip=FIRASIM_VISION_IP,
        vision_port=FIRASIM_VISION_PORT
    ), fieldData

def getProtoControl():
    return ProtoControl(
        team_color_yellow=IS_YELLOW_TEAM, 
        control_ip=FIRASIM_CONTROL_IP, 
        control_port=FIRASIM_CONTROL_PORT)

def getOpponentProtoControl():
    return ProtoControl(
        team_color_yellow=not IS_YELLOW_TEAM, 
        control_ip=FIRASIM_CONTROL_IP, 
        control_port=FIRASIM_CONTROL_PORT)

def updateVisions(vision: ProtoVision, oppositeTeamVision: ProtoVision):
    vision.update()
    oppositeTeamVision.update()

vision, field_data = getProtoVision(IS_YELLOW_TEAM)
opponentVision, opponent_field_data = getProtoVision(not IS_YELLOW_TEAM)

teamControl = getProtoControl()
opponentTeamControl = getOpponentProtoControl()

updateVisions(vision, opponentVision)

def act(
    robot_id: int,
    is_own_team: bool = True
):
    if is_own_team:
        team_field_data = field_data
        is_left_team = IS_LEFT_TEAM
        team_control = teamControl
    else:
        team_field_data = opponent_field_data
        is_left_team = not IS_LEFT_TEAM
        team_control = opponentTeamControl

    observations = RSoccerHelper.get_attacker_observation(team_field_data, is_left_team, robot_id)

    action, _ = attacker_model.predict(observations)
    left_speed, right_speed = RSoccerHelper.actions_to_v_wheels(action, is_own_team)
    if is_left_team:
        team_control.transmit_robot(robot_id, right_speed, left_speed)
    else:
        team_control.transmit_robot(robot_id, left_speed, right_speed)

def go_to_point(
    robot_id: int,
    target_position: tuple[float, float],
    last_error: float,
    is_own_team: bool = True
):
    if is_own_team:
        team_field_data = field_data
        team_control = teamControl
    else:
        team_field_data = opponent_field_data
        team_control = opponentTeamControl

    robot = team_field_data.robots[robot_id]

    left_speed, right_speed, error = MotionUtils.go_to_point(
        robot,
        target_position,
        last_error
    )

    team_control.transmit_robot(robot_id, left_speed, right_speed)

    return error

error = 0

tempo = time.time()

teste = 0

while True:
    robot = field_data.robots[0]
    ball = field_data.ball

    act(0)

    updateVisions(vision, opponentVision)