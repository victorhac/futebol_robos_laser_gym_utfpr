import logging
import json

from .receiver import Receiver
import communication.protobuf.ssl_vision_wrapper_pb2 as wrapper

from google.protobuf.json_format import MessageToJson

from domain.field import Field
from configuration.configuration import Configuration
from domain.entity import Entity
from threads.job import Job

class SSLVisionReceiver(Receiver):
    def __init__(
        self,
        field: Field
    ):
        self.configuration = Configuration.get_object()

        super(SSLVisionReceiver, self).__init__(
            self.configuration.sslvision_ip,
            self.configuration.sslvision_port,
            self.configuration.sslvision_receiver_buffer_size
        )

        self.field = field

    def get_team_robot_id_mapping(self):
        return {
            str(self.configuration.sslvision_team_robot_id_mapping_0): 0,
            str(self.configuration.sslvision_team_robot_id_mapping_1): 1,
            str(self.configuration.sslvision_team_robot_id_mapping_2): 2
        }

    def receive(self):
        return super().receive()

    def receive_dict(self):
        data = self.receive()

        packet = wrapper.SSL_WrapperPacket()
        packet.ParseFromString(data)

        detection = packet.detection

        return json.loads(MessageToJson(detection))

    def receive_field_data(self):
        vision_data_dict = self.receive_dict()
        self._field_data_from_dict(vision_data_dict)

    def update(self):
        if self.field is None:
            logging.error('FieldData not instantiated', exc_info=True)
        else:
            vision_data_dict = self.receive_dict()

            self._field_data_from_dict(vision_data_dict)

    def _entity_from_dict(
        self,
        data_dict: dict,
        entity_data: Entity
    ):
        entity_data.position.x = data_dict.get('x', 0) / 1000
        entity_data.position.y = data_dict.get('y', 0) / 1000

        entity_data.position.theta = data_dict.get('orientation', 0)

        entity_data.velocity.x = data_dict.get('vx', 0)
        entity_data.velocity.y = data_dict.get('vy', 0)

        entity_data.velocity.theta = data_dict.get('vorientation', 0)

        return entity_data
    
    def team_list_of_dicts(self, raw_data_dict, key: str):
        team_list_of_dicts = raw_data_dict.get(key)

        if team_list_of_dicts is None:
            return []
        
        return team_list_of_dicts
    
    def get_index(self, received_robot):
        robot_id = str(received_robot.get("robotId"))
        return self.get_team_robot_id_mapping().get(robot_id)

    def _field_data_from_dict(self, raw_data_dict):
        if self.configuration.team_is_yellow_team:
            team_list_of_dicts = self.team_list_of_dicts(raw_data_dict, 'robotsYellow')
            foes_list_of_dicts = self.team_list_of_dicts(raw_data_dict, 'robotsBlue')
        else:
            team_list_of_dicts = self.team_list_of_dicts(raw_data_dict, 'robotsBlue')
            foes_list_of_dicts = self.team_list_of_dicts(raw_data_dict, 'robotsYellow')

        balls = raw_data_dict.get('balls')

        ball = None if balls is None else balls[0]

        if ball:
            self._entity_from_dict(ball, self.field.ball)

        for received_robot in team_list_of_dicts:
            index = self.get_index(received_robot)

            if index is not None:
                self._entity_from_dict(received_robot, self.field.robots[index])

        #TODO: testar
        received_goalkeeper_robot = None
        sslvision_foe_team_desired_goalkeeper_mapped_id = self.configuration.sslvision_foe_team_desired_goalkeeper_mapped_id

        for received_robot in foes_list_of_dicts:
            robot_id = received_robot.get("robotId")

            if robot_id == self.configuration.game_controller_foe_team_goalkeeper_desired_id:
                received_goalkeeper_robot = received_robot

        if received_goalkeeper_robot is not None:
            self._entity_from_dict(
                received_goalkeeper_robot,
                self.field.foes[sslvision_foe_team_desired_goalkeeper_mapped_id])
            
        counter = sslvision_foe_team_desired_goalkeeper_mapped_id

        for received_robot in foes_list_of_dicts:
            robot_id = received_robot.get("robotId")
            counter = (counter + 1) % 3

            if robot_id != sslvision_foe_team_desired_goalkeeper_mapped_id:
                self._entity_from_dict(received_robot, self.field.foes[counter])
            

class ProtoVisionThread(Job):
    def __init__(
        self,
        field_data: Field
    ):
        self.vision = SSLVisionReceiver(field_data)

        super(ProtoVisionThread, self).__init__(self.vision.update)