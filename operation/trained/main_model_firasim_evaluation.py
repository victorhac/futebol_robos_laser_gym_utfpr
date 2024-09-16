import time
from stable_baselines3 import PPO
from lib.comm.vision import ProtoVision
from lib.comm.control import ProtoControl

from lib.utils.configuration_utils import ConfigurationUtils
from lib.utils.rsoccer_utils import RSoccerUtils
from lib.motion.motion_utils import MotionUtils

from lib.domain.field_data import FieldData

FIRASIM_CONTROL_IP = ConfigurationUtils.get_firasim_control_ip()
FIRASIM_CONTROL_PORT = ConfigurationUtils.get_firasim_control_port()
FIRASIM_VISION_IP = ConfigurationUtils.get_firasim_vision_ip()
FIRASIM_VISION_PORT = ConfigurationUtils.get_firasim_vision_port()

IS_YELLOW_TEAM = ConfigurationUtils.get_firasim_team_is_yellow_team()
IS_LEFT_TEAM = ConfigurationUtils.get_firasim_is_left_team()

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

    observations = RSoccerUtils.get_attacker_observation(team_field_data, is_left_team, robot_id)

    action, _ = attacker_model.predict(observations)
    left_speed, right_speed = RSoccerUtils.actions_to_v_wheels(action, is_own_team)

    team_control.transmit_robot(robot_id, left_speed, right_speed)

def go_to_point(
    robot_id: int,
    target_position: 'tuple[float, float]',
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

while True:
    act(0)

    updateVisions(vision, opponentVision)