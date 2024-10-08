import logging

import communication.receiver.messages_robocup_ssl_wrapper_pb2 as wrapper

import json

from google.protobuf.json_format import MessageToJson
import numpy as np

from lib.comm.receiver import Receiver
from lib.comm.thread_job import Job
from lib.core.data import EntityData, FieldData
from lib.helpers.configuration_helper import ConfigurationHelper
from lib.helpers.field_helper import FieldHelper
from lib.helpers.firasim_helper import FIRASimHelper

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

    def _field_data_from_dict(self, field_data: FieldData, raw_data_dict):
        isYellowLeftTeam = self.configuration['team']['is-yellow-left-team']
        isLeftTeam = FieldHelper.isLeftTeam(self.team_color_yellow, isYellowLeftTeam)

        rotate_field = isLeftTeam
        
        if self.team_color_yellow:
            team_list_of_dicts = raw_data_dict.get('robotsYellow')
            foes_list_of_dicts = raw_data_dict.get('robotsBlue')
        else:
            team_list_of_dicts = raw_data_dict.get('robotsBlue')
            foes_list_of_dicts = raw_data_dict.get('robotsYellow')

        if team_list_of_dicts is None:
            team_list_of_dicts = []

        if foes_list_of_dicts is None:
            foes_list_of_dicts = []

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

        for i in range(len(team_list_of_dicts)):
            field_data.robots[i] = self._entity_from_dict(team_list_of_dicts[i], rotate_field)

        for i in range(len(foes_list_of_dicts)):
            field_data.foes[i] = self._entity_from_dict(foes_list_of_dicts[i], rotate_field)

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

