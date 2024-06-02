from lib.comm.vision import ProtoVision
from lib.comm.control import ProtoControl

from lib.helpers.configuration_helper import ConfigurationHelper
from lib.motion.motion_utils import MotionUtils
from lib.helpers.field_helper import FieldHelper
from lib.helpers.firasim_helper import FIRASimHelper

from lib.domain.field_data import FieldData

IS_YELLOW_TEAM = ConfigurationHelper.get_firasim_team_is_yellow_team()

IS_LEFT_TEAM = ConfigurationHelper.get_firasim_is_left_team()

ROBOT_LENGTH = ConfigurationHelper.get_firasim_robot_length()
ROBOT_WIDTH = ConfigurationHelper.get_firasim_robot_width()

FIELD_WIDTH = ConfigurationHelper.get_field_width()
FIELD_LENGTH = ConfigurationHelper.get_field_length()

MOTION_COLLISION_AVOIDANCE_MIN_DISTANCE = ConfigurationHelper.get_motion_collision_avoidance_min_distance()

FIRASIM_CONTROL_IP = ConfigurationHelper.get_firasim_control_ip()
FIRASIM_CONTROL_PORT = ConfigurationHelper.get_firasim_control_port()
FIRASIM_VISION_IP = ConfigurationHelper.get_firasim_vision_ip()
FIRASIM_VISION_PORT = ConfigurationHelper.get_firasim_vision_port()

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

        velocities = MotionUtils.go_to_point(robot, currentTargetPosition, error)

        (leftSpeed, rightSpeed, error) = velocities

        teamControl.transmit_robot(0, leftSpeed, rightSpeed)

        currentTargetPosition = targetPosition

        updateVisions(vision, opponentVision)

if __name__ == '__main__':
    main()
