import logging

from configuration.configuration import Configuration
from domain.grsim.commands.team_command import TeamCommand

from .sender import Sender
from .socker_sender import SocketSender

from communication.protobuf.grsim.packet_pb2 import Packet
from communication.protobuf.grsim.command_pb2 import Commands

from threading.job import Job

class GrSimSender(Sender):
    def __init__(self):
        self.configuration = Configuration.get_object()
        self.socket_sender = SocketSender(
            self.configuration.grsim_control_ip,
            self.configuration.grsim_control_port
        )

    def transmit(self, packet):
        self.socket_sender.transmit(packet)

    def transmit_robot(self, robot_id, left_speed, right_speed):
        packet = self._fill_robot_command_packet(robot_id, left_speed, right_speed)
        self.transmit(packet)

    def _fill_robot_command_packet(self, robot_id, left_speed, right_speed):
        cmd_packet = Commands()

        robot = cmd_packet.robot_commands.add() # pylint: disable=no-member
        robot.id          = robot_id
        robot.yellowteam  = self.configuration.team_is_yellow_team
        robot.wheel_left  = left_speed
        robot.wheel_right = right_speed

        packet = Packet()
        packet.cmd.CopyFrom(cmd_packet) # pylint: disable=no-member

        return packet

    def transmit_team(self, team_cmd: TeamCommand):
        packet = self._fill_team_command_packet(team_cmd)
        self.transmit(packet)

    def stop_team(self):
        stop_team_cmd = TeamCommand()
        self.transmit_team(stop_team_cmd)

    def _fill_team_command_packet(self, team_command : TeamCommand):
        cmd_packet = Commands()

        for i in range(len(team_command.commands)):
            cmd = cmd_packet.robot_commands.add()
            cmd.id          = i
            cmd.yellowteam  = self.configuration.team_is_yellow_team
            cmd.wheel_left  = team_command.commands[i].left_speed
            cmd.wheel_right = team_command.commands[i].right_speed

        packet = Packet()
        packet.cmd.CopyFrom(cmd_packet)

        return packet

class GrSimSenderThread(Job):
    def __init__(self):
        self.control = GrSimSender()

        super(GrSimSenderThread, self).__init__(self.control.update)

    def pause(self):
        super().pause()
        self.control.stop_team()

    def resume(self):
        super().resume()

    def stop(self):
        super().pause()
        self.control.stop_team()