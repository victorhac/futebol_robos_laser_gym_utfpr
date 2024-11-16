import logging

from utils import GrSimUtils
from communication.protobuf.ssl_vision_wrapper_pb2 import SSL_WrapperPacket
from configuration.configuration import Configuration
from domain.entity import Entity
from domain.field import Field
from threads.job import Job

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

        super(GrSimReceiver, self).__init__(
            self.configuration.grsim_vision_ip,
            self.configuration.grsim_vision_port,
            self.configuration.sslvision_receiver_buffer_size
        )

        self.field = field

    def receive(self):
        return super().receive()

    def receive_dict(self):
        data = self.receive()
        decoded_data = SSL_WrapperPacket().FromString(data)
        vision_frame = decoded_data.detection
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
        data_dict: dict
    ):
        is_left_team = self.configuration.get_is_left_team()
        sum_to_angle = 0 if not is_left_team else np.pi

        entity_data.position.x, entity_data.position.y = \
            GrSimUtils.normalize_position(
                data_dict.get('x', 0), 
                data_dict.get('y', 0),
                is_left_team)

        entity_data.position.x /= 1000
        entity_data.position.y /= 1000

        if not is_left_team:
            entity_data.position.theta = \
                GrSimUtils.normalize_angle(
                    self._assert_angle(data_dict.get('orientation', 0) + sum_to_angle))
        else:
            entity_data.position.theta = data_dict.get('orientation', 0)

        entity_data.velocity.x, entity_data.velocity.y = \
            GrSimUtils.normalize_speed(
                data_dict.get('vx', 0),
                data_dict.get('vy', 0),
                is_left_team)

        entity_data.velocity.theta = data_dict.get('vorientation', 0)

    def get_is_left_team(self):
        return self.configuration.get_is_left_team()

    def _field_data_from_dict(self, field: Field, raw_data_dict: dict):        
        if self.configuration.team_is_yellow_team:
            team_list_of_dicts = raw_data_dict.get('robotsYellow', [])
            foes_list_of_dicts = raw_data_dict.get('robotsBlue', [])
        else:
            team_list_of_dicts = raw_data_dict.get('robotsBlue', [])
            foes_list_of_dicts = raw_data_dict.get('robotsYellow', [])

        balls = raw_data_dict.get('balls')

        ball = None if balls is None else balls[0]

        if ball:
            self._entity_from_dict(field.ball, ball)

        for received_robot in team_list_of_dicts:
            robot_id = received_robot.get('robotId')
            robot = field.robots.get(robot_id)

            if robot:
                self._entity_from_dict(robot, received_robot)

        for received_robot in foes_list_of_dicts:
            robot_id = received_robot.get('robotId')
            foe = field.foes.get(robot_id)

            if foe:
                self._entity_from_dict(foe, received_robot)

    def _assert_angle(self, angle):
        angle = angle % (2 * np.pi)

        if angle > np.pi:
            angle -= 2 * np.pi

        return angle

class GrSimReceiverThread(Job):
    def __init__(self, field: Field):
        self.vision = GrSimReceiver(field)

        super(GrSimReceiverThread, self).__init__(self.vision.update)