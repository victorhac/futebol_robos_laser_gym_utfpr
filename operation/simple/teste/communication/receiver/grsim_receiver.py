import logging

from teste.communication.protobuf.grsim.packet_pb2 import Environment
from teste.configuration.configuration import Configuration
from teste.domain.entity import Entity
from teste.domain.field import Field
from teste.threading.job import Job
from teste.utils.grsim_utils import GrSimUtils

from .receiver import Receiver

import json
from google.protobuf.json_format import MessageToJson
import numpy as np

class GrSimReceiver(Receiver):
    def __init__(
        self,
        field: Field
    ):
        self.configuration = Configuration.get_object()

        super(Receiver, self).__init__(
            self.configuration.grsim_vision_ip,
            self.configuration.grsim_vision_port
        )

        self.field = field

    def receive(self):
        return super().receive()

    def receive_dict(self):
        data = self.receive()
        decoded_data = Environment().FromString(data) # pylint: disable=no-member
        vision_frame = decoded_data.frame
        return json.loads(MessageToJson(vision_frame))

    def receive_field_data(self):
        vision_data_dict = self.receive_dict()

        rcv_field_data = Field()

        self._field_data_from_dict(rcv_field_data, vision_data_dict)

        return rcv_field_data

    def update(self):
        if self.field is None:
            logging.error('FieldData not instantiated', exc_info=True)
        else:
            vision_data_dict = self.receive_dict()

            self._field_data_from_dict(self.field, vision_data_dict)

    def _entity_from_dict(
        self,
        entity_data: Entity,
        data_dict: dict,
        is_left_team: bool,
        foes=False
    ):
        sum_to_angle = 0 if not is_left_team else np.pi

        entity_data.position.x, entity_data.position.y = \
            GrSimUtils.normalize_position(
                data_dict.get('x', 0), 
                data_dict.get('y', 0),
                is_left_team)

        if not foes:
            entity_data.position.theta = \
                GrSimUtils.normalize_angle(
                    self._assert_angle(data_dict.get('orientation', 0) + sum_to_angle))
        else:
            entity_data.position.theta = \
                GrSimUtils.normalize_angle(
                    self._assert_angle(data_dict.get('orientation', 0)))

        entity_data.velocity.x, entity_data.velocity.y = \
            GrSimUtils.normalizeSpeed(
                data_dict.get('vx', 0),
                data_dict.get('vy', 0),
                is_left_team)

        entity_data.velocity.theta = data_dict.get('vorientation', 0)

    def get_is_left_team(self):
        return self.configuration.team_is_yellow_left_team == self.configuration.team_is_yellow_team

    def _field_data_from_dict(self, field: Field, raw_data_dict: dict):
        isLeftTeam = self.get_is_left_team()

        rotate_field = isLeftTeam
        
        if self.configuration.team_is_yellow_team:
            team_list_of_dicts = raw_data_dict.get('robotsYellow')
            foes_list_of_dicts = raw_data_dict.get('robotsBlue')
        else:
            team_list_of_dicts = raw_data_dict.get('robotsBlue')
            foes_list_of_dicts = raw_data_dict.get('robotsYellow')

        if 'ball' in raw_data_dict:
            self._entity_from_dict(field.ball, raw_data_dict['ball'], rotate_field)

        for i in range(len(team_list_of_dicts)):
            self._entity_from_dict(field.robots[i], team_list_of_dicts[i], rotate_field)

        for i in range(len(foes_list_of_dicts)):
            self._entity_from_dict(field.foes[i], foes_list_of_dicts[i], rotate_field, True)

    def _assert_angle(self, angle):
        angle = angle % (2 * np.pi)

        if angle > np.pi:
            angle -= 2 * np.pi

        return angle

class GrSimReceiverThread(Job):
    def __init__(self, field: Field):
        self.vision = GrSimReceiver(field)

        super(GrSimReceiverThread, self).__init__(self.vision.update)