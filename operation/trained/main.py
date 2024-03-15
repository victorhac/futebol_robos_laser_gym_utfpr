from lib.comm.vision import ProtoVision
from lib.core.data import FieldData
from lib.comm.control import ProtoControl
from lib.geometry.geometry import Geometry

from lib.helpers.configuration_helper import ConfigurationHelper
from lib.motion.motion import Motion

def main():
    configuration = ConfigurationHelper.getConfiguration()

    fieldData = FieldData()

    isYellowTeam = configuration["team"]["yellow-team"]
    controlIp = configuration["FIRASim"]["control"]["ip"]
    controlPort = configuration["FIRASim"]["control"]["port"]

    vision = ProtoVision(
        team_color_yellow=isYellowTeam, 
        field_data=fieldData)
    
    yellowTeamControl = ProtoControl(
        team_color_yellow=isYellowTeam, 
        control_ip=controlIp, 
        control_port=controlPort)
    
    targetPosition = (-0.5, -0.5)
    robot = fieldData.robots[0]
    position = robot.position
    error = 0
    
    while not Geometry.isClose((position.x, position.y), 
                      targetPosition, 
                      0.1):
        velocities = Motion.pid(robot, targetPosition, error)

        (leftSpeed, rightSpeed, error) = velocities

        yellowTeamControl.transmit_robot(0, leftSpeed, rightSpeed)

        vision.update()

    yellowTeamControl.transmit_robot(0, 0, 0)

if __name__ == '__main__':
    main()
