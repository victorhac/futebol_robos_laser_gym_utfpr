import time
from lib.comm.vision import ProtoVision
from lib.core.data import FieldData
from lib.comm.control import ProtoControl
from lib.geometry.geometry import Geometry

from lib.helpers.configuration_helper import ConfigurationHelper
from lib.motion.motion import Motion
from lib.helpers.firasim_helper import FIRASimHelper
from lib.helpers.field_helper import FieldHelper

def main():
    configuration = ConfigurationHelper.getConfiguration()

    fieldData = FieldData()

    isYellowTeam = configuration["team"]["is-yellow-team"]
    isYellowLeftTeam = configuration["team"]["is-yellow-left-team"]
    controlIp = configuration["FIRASim"]["control"]["ip"]
    controlPort = configuration["FIRASim"]["control"]["port"]

    isLeftTeam = FieldHelper.isLeftTeam(isYellowTeam, isYellowLeftTeam)

    vision = ProtoVision(
        team_color_yellow=isYellowTeam, 
        field_data=fieldData)
    
    teamControl = ProtoControl(
        team_color_yellow=isYellowTeam, 
        control_ip=controlIp, 
        control_port=controlPort)
    
    targetPosition = FIRASimHelper.normalizePosition(-0.5, -0.5, isLeftTeam)
    robot = fieldData.robots[0]
    position = robot.position
    error = 0

    vision.update()
    
    while not Geometry.isClose(
        (position.x, position.y),
        targetPosition,
        0.1):
        
        velocities = Motion.goToPoint(robot, targetPosition, error)

        (leftSpeed, rightSpeed, error) = velocities

        teamControl.transmit_robot(0, leftSpeed, rightSpeed)

        vision.update()

    teamControl.transmit_robot(0, 0, 0)

if __name__ == '__main__':
    main()
