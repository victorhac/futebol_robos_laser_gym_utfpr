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
        entity_data: EntityData,
        data_dict,
        isLeftTeam=False
    ):
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

    def _field_data_from_dict(self, field_data: FieldData, raw_data_dict):
        isYellowLeftTeam = self.configuration['team']['is-yellow-left-team']
        isLeftTeam = FieldHelper.isLeftTeam(self.team_color_yellow, isYellowLeftTeam)

        rotate_field = isLeftTeam
        
        if self.team_color_yellow == True:
            team_list_of_dicts = raw_data_dict.get('robots_yellow')
            foes_list_of_dicts = raw_data_dict.get('robots_blue')
        else:
            team_list_of_dicts = raw_data_dict.get('robots_blue')
            foes_list_of_dicts = raw_data_dict.get('robots_yellow')

        # TODO: determine how to choose the correct ball
        ball_index = 0
        ball = raw_data_dict['balls'][ball_index]

        self._entity_from_dict(field_data.ball, ball, True)

        for i in range(len(team_list_of_dicts)):
            self._entity_from_dict(field_data.robots[i], team_list_of_dicts[i], rotate_field)

        for i in range(len(foes_list_of_dicts)):
            self._entity_from_dict(field_data.foes[i], foes_list_of_dicts[i], rotate_field)

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

