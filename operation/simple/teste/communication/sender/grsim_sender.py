from configuration.configuration import Configuration

from .sender import Sender
from .socker_sender import SocketSender

from communication.protobuf.grSim_Packet_pb2 import grSim_Packet
from communication.protobuf.grSim_Commands_pb2 import grSim_Commands

import time

class GrSimSender(Sender):
    def __init__(self):
        self.configuration = Configuration.get_object()
        self.socket_sender = SocketSender(
            self.configuration.grsim_control_ip,
            self.configuration.grsim_control_port
        )

    def _transmit(self, packet):
        self.socket_sender.transmit(packet)

    def transmit_robot(self, robot_id, left_speed, right_speed):
        if self.configuration.get_is_left_team():
            packet = self._fill_robot_command_packet(robot_id, left_speed, right_speed)
        else:
            packet = self._fill_robot_command_packet(robot_id, right_speed, left_speed)
        self._transmit(packet)

    def _fill_robot_command_packet(self, robot_id, left_speed, right_speed):
        cmd_packet = grSim_Commands()
        cmd_packet.isteamyellow = self.configuration.team_is_yellow_team
        cmd_packet.timestamp = time.time()

        robot = cmd_packet.robot_commands.add()
        robot.id          = robot_id
        robot.kickspeedx  = 0
        robot.kickspeedz  = 0
        robot.veltangent  = 0
        robot.velnormal   = 0
        robot.velangular  = 0
        robot.spinner     = False
        robot.wheelsspeed = True
        robot.wheel1 = -left_speed
        robot.wheel2 = -left_speed
        robot.wheel3  = right_speed
        robot.wheel4  = right_speed

        packet = grSim_Packet()
        packet.commands.CopyFrom(cmd_packet)

        return packet