from lib.comm.vision import ProtoVision
from lib.comm.control import ProtoControl

from lib.helpers.configuration_helper import ConfigurationHelper
from lib.motion.motion_utils import MotionUtils
from lib.helpers.field_helper import FieldHelper
from lib.helpers.firasim_helper import FIRASimHelper

from lib.domain.field_data import FieldData

CONFIGURATION = ConfigurationHelper.getConfiguration()

IS_YELLOW_TEAM = CONFIGURATION["team"]["is-yellow-team"]
IS_YELLOW_LEFT_TEAM = CONFIGURATION["team"]["is-yellow-left-team"]

IS_LEFT_TEAM = FieldHelper.isLeftTeam(IS_YELLOW_TEAM, IS_YELLOW_LEFT_TEAM)

ROBOT_LENGTH = CONFIGURATION["robot"]["length"]
ROBOT_WIDTH = CONFIGURATION["robot"]["width"]

FIELD_WIDTH = CONFIGURATION["field"]["width"]
FIELD_LENGTH = CONFIGURATION["field"]["length"]

MOTION_COLLISION_AVOIDANCE_MIN_DISTANCE = CONFIGURATION["motion"]["collision-avoidance"]["min-distance"]

FIRASIM_CONTROL_IP = CONFIGURATION["FIRASim"]["control"]["ip"]
FIRASIM_CONTROL_PORT = CONFIGURATION["FIRASim"]["control"]["port"]
FIRASIM_VISION_IP = CONFIGURATION["FIRASim"]["vision"]["ip"]
FIRASIM_VISION_PORT = CONFIGURATION["FIRASim"]["vision"]["port"]

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

def updateVisions(vision: ProtoVision, oppositeTeamVision: ProtoVision):
    vision.update()
    oppositeTeamVision.update()

def main():
    vision, fieldData = getProtoVision(IS_YELLOW_TEAM)
    opponentVision, opponentFieldData = getProtoVision(not IS_YELLOW_TEAM)
    
    teamControl = getProtoControl()
    
    targetPosition = FIRASimHelper.normalizePosition(x=0.6, y=0.6, isLeftTeam=IS_LEFT_TEAM)

    currentTargetPosition = targetPosition
    robot = fieldData.robots[0]
    ball = fieldData.ball
    error = 0

    updateVisions(vision, opponentVision)
    
    while True:
        targetPosition = FIRASimHelper.normalizePosition(ball.position.x, ball.position.y, IS_LEFT_TEAM)

        tangentPoint = MotionUtils.findTangentPointObstacle(0, fieldData, opponentFieldData, targetPosition)

        if tangentPoint is not None:
            currentTargetPosition = tangentPoint

        velocities = MotionUtils.goToPoint(robot, currentTargetPosition, error)

        (leftSpeed, rightSpeed, error) = velocities

        teamControl.transmit_robot(0, leftSpeed, rightSpeed)

        currentTargetPosition = targetPosition

        updateVisions(vision, opponentVision)

if __name__ == '__main__':
    main()
