from stable_baselines3 import DDPG

from lib.comm.vision import ProtoVision
from lib.comm.control import ProtoControl

from lib.helpers.configuration_helper import ConfigurationHelper
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

TRAINING_VELOCITY_CLIP_VALUE = ConfigurationHelper.getTrainingVelocityClipValue()

print(FIRASIM_CONTROL_IP, FIRASIM_CONTROL_PORT, FIRASIM_VISION_IP, FIRASIM_VISION_PORT)

model = DDPG.load("models/ball_following/2024_05_01_17_30_50/DDPG_model_1000000_steps.zip")

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

def getObservation(fieldData: FieldData):
    robot = fieldData.robots[0]
    ball = fieldData.ball

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
        ball.position.y
    ]
        
    return np.array(observations)

vision, fieldData = getProtoVision(IS_YELLOW_TEAM)
opponentVision, opponentFieldData = getProtoVision(not IS_YELLOW_TEAM)

teamControl = getProtoControl()
opponentTeamControl = getOpponentProtoControl()

robot = fieldData.robots[0]
opponentRobot = opponentFieldData.robots[0]

ball = fieldData.ball

error = opponentError = 0

updateVisions(vision, opponentVision)

while True:
    obs = getObservation(fieldData)

    action, state = model.predict(obs)
    leftSpeed, rightSpeed, error = MotionUtils.goToPoint(robot, action, error)
    teamControl.transmit_robot(0, leftSpeed, rightSpeed)

    opponentTargetPosition = (ball.position.x, ball.position.y)
    leftSpeed, rightSpeed, opponentError = MotionUtils.goToPoint(opponentRobot, opponentTargetPosition, opponentError)
    opponentTeamControl.transmit_robot(0, leftSpeed, rightSpeed)

    updateVisions(vision, opponentVision)