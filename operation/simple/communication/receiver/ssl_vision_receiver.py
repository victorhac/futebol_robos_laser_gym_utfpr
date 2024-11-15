import logging

import sys
import os

from lib.comm.receiver import Receiver
sys.path.append(os.path.abspath("/home/argenton/Documentos/Futebol_Robos/futebol_robos_laser_gym_utfpr/operation/simple/communication/receiver"))
sys.path.append(os.path.abspath("/home/argenton/Documentos/Futebol_Robos/futebol_robos_laser_gym_utfpr/operation/simple/lib/comm"))
sys.path.append(os.path.abspath("/home/argenton/Documentos/Futebol_Robos/futebol_robos_laser_gym_utfpr/operation/simple/lib/core"))
sys.path.append(os.path.abspath("/home/argenton/Documentos/Futebol_Robos/futebol_robos_laser_gym_utfpr/operation/simple/lib/helpers"))

import messages_robocup_ssl_wrapper_pb2 as wrapper

import json

from google.protobuf.json_format import MessageToJson
import numpy as np

from thread_job import Job
from data import EntityData, FieldData
from configuration_helper import ConfigurationHelper
from field_helper import FieldHelper

from firasim_helper import FIRASimHelper

class SSLVisionReceiver(Receiver):
    def __init__(
        self,
        team_color_yellow: bool,
        field_data: FieldData = None,
        vision_ip='224.5.23.2',
        vision_port=10006
    ):
        super(SSLVisionReceiver, self).__init__(vision_ip, vision_port)

        self.team_color_yellow = team_color_yellow
        self.field_data = field_data
        self.configuration = ConfigurationHelper.getConfiguration()

        self.team_robot_id_mapping ={
                "1": 1,
                "2": 2,
                "0": 0
            }
        self.foe_robot_id_mapping = {
                                     
                "1": 1,
                "2": 2,
                "0": 0
            }

    def receive(self):
        return super().receive()

    def receive_dict(self):
        data = self.receive()

        packet = wrapper.SSL_WrapperPacket()
        packet.ParseFromString(data)

        detection = packet.detection

        return json.loads(MessageToJson(detection))

    def receive_field_data(self) -> FieldData:
        vision_data_dict = self.receive_dict()

        rcv_field_data = FieldData()

        self._field_data_from_dict(rcv_field_data, vision_data_dict)

        return rcv_field_data

    def update(self):
        if self.field_data is None:
            logging.error('FieldData not instantiated', exc_info=True)
        else:
            vision_data_dict = self.receive_dict()

            self._field_data_from_dict(self.field_data, vision_data_dict)

    def _entity_from_dict(
        self,
        data_dict,
        isLeftTeam=False
    ):
        entity_data = EntityData()

        sum_to_angle = 0 if not isLeftTeam else np.pi

        entity_data.position.x, entity_data.position.y = \
            FIRASimHelper.normalizePosition(
                data_dict.get('x', 0), 
                data_dict.get('y', 0),
                isLeftTeam)

        entity_data.position.theta = \
            FIRASimHelper.normalizeAngle(data_dict.get('orientation', 0) + sum_to_angle)

        # TODO: find a way to calculate the speed
        entity_data.velocity.x, entity_data.velocity.y = \
            FIRASimHelper.normalizeSpeed(
                data_dict.get('vx', 0),
                data_dict.get('vy', 0),
                isLeftTeam)

        entity_data.velocity.theta = data_dict.get('vorientation', 0)

        return entity_data
    
    def team_list_of_dicts(self, raw_data_dict, key: str):
        team_list_of_dicts = raw_data_dict.get(key)

        if team_list_of_dicts is None:
            return []
        
        return team_list_of_dicts
    
    def get_index(self, received_robot):
        robot_id = str(received_robot.get("robotId")).lower()
        return self.foe_robot_id_mapping.get(robot_id)

    def _field_data_from_dict(self, field_data: FieldData, raw_data_dict):
        isYellowLeftTeam = self.configuration['team']['is-yellow-left-team']
        isLeftTeam = FieldHelper.isLeftTeam(self.team_color_yellow, isYellowLeftTeam)

        rotate_field = isLeftTeam
        
        if self.team_color_yellow:
            team_list_of_dicts = self.team_list_of_dicts(raw_data_dict, 'robotsYellow')
            foes_list_of_dicts = self.team_list_of_dicts(raw_data_dict, 'robotsBlue')
        else:
            team_list_of_dicts = self.team_list_of_dicts(raw_data_dict, 'robotsBlue')
            foes_list_of_dicts = self.team_list_of_dicts(raw_data_dict, 'robotsYellow')

        # TODO: determine how to choose the correct ball
        ball_index = 0

        fake_ball = {
            "confidence": 0.99282587,
            "area": 52,
            "x": 369.55273,
            "y": -838.1288,
            "pixelX": 389.59616,
            "pixelY": 453.98077
        }

        balls = raw_data_dict.get('balls')

        ball = balls[ball_index] if balls is not None else fake_ball

        field_data.ball = self._entity_from_dict(ball, True)
        field_data.ball.position.x *= 0.0012
        field_data.ball.position.y *= 0.00173

        for received_robot in team_list_of_dicts:
            robot = self._entity_from_dict(received_robot, rotate_field)
            robot.position.x  *= 0.0012
            robot.position.y *= 0.00173

            index = self.get_index(received_robot)

            if index is not None:
                field_data.robots[index] = robot

        for received_robot in foes_list_of_dicts:
            robot = self._entity_from_dict(received_robot, rotate_field)
            robot.position.x *= 0.0012
            robot.position.y *= 0.00173

            index = self.get_index(received_robot)

            if index is not None:
                field_data.foes[index] = robot

class ProtoVisionThread(Job):
    def __init__(
        self,
        team_color_yellow: bool,
        field_data: FieldData = None,
        vision_ip='224.0.0.1',
        vision_port=10002
    ):
        self.vision = SSLVisionReceiver(
            team_color_yellow,
            field_data,
            vision_ip,
            vision_port)

        super(ProtoVisionThread, self).__init__(self.vision.update)

