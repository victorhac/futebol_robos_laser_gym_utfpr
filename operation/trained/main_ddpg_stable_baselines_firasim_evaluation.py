from lib.comm.vision import ProtoVision
from lib.comm.control import ProtoControl

from lib.helpers.configuration_helper import ConfigurationHelper
from lib.helpers.model_helper import ModelHelper
from lib.helpers.rsoccer_helper import RSoccerHelper
from lib.motion.motion_utils import MotionUtils

from lib.domain.field_data import FieldData

from lib.geometry.geometry_utils import GeometryUtils

import numpy as np

FIRASIM_CONTROL_IP = ConfigurationHelper.getFIRASimControlIp()
FIRASIM_CONTROL_PORT = ConfigurationHelper.getFIRASimControlPort()
FIRASIM_VISION_IP = ConfigurationHelper.getFIRASimVisionIp()
FIRASIM_VISION_PORT = ConfigurationHelper.getFIRASimVisionPort()

IS_YELLOW_TEAM = ConfigurationHelper.getTeamIsYellowTeam()
IS_LEFT_TEAM = ConfigurationHelper.isLeftTeam()

attacker_model = ModelHelper.get_attacker_model()
defensor_model = ModelHelper.get_defensor_model()
goalkeeper_model = ModelHelper.get_goalkeeper_v2_model()

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
    role_id: int,
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

    if role_id == 0:
        observations = RSoccerHelper.get_goalkeeper_observations(team_field_data, is_left_team, robot_id)
        model = goalkeeper_model
    elif role_id == 1:
        observations = RSoccerHelper.get_defensor_observations(team_field_data, is_left_team, robot_id)
        model = defensor_model
    else:
        observations = RSoccerHelper.get_attacker_observations(team_field_data, is_left_team, robot_id)
        model = attacker_model

    action, _ = model.predict(observations)
    left_speed, right_speed = RSoccerHelper.actions_to_v_wheels(action)
    team_control.transmit_robot(robot_id, -1, 1)

while True:
    act(0, 2)
    # act(1, 1)
    # act(2, 0)

    robot = field_data.robots[0]

    print(f"Robot position: {robot.velocity.x}, {robot.velocity.y} {robot.position.theta}")

    # act(0, 2, False)
    # act(1, 1, False)
    # act(2, 0, False)

    updateVisions(vision, opponentVision)