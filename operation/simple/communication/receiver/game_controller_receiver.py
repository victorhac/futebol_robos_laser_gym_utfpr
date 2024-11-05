import json
import protobuf.SSL_vision.messages_robocup_ssl_wrapper_pb2 as wrapper

from domain.entity import Entity
from domain.field import Field

from google.protobuf.json_format import MessageToJson
from receiver import Receiver
from configuration_helper import ConfigurationHelper

class GameControllerReceiver(Receiver):
    def __init__(
        self,
        team_color_yellow: bool
    ):
        self.configuration = ConfigurationHelper.getConfiguration()

        vision_ip='224.5.23.2'
        vision_port=10006

        super(GameControllerReceiver, self).__init__(vision_ip, vision_port)

        self.team_color_yellow = team_color_yellow

        self.team_robot_id_mapping: dict =\
            self.configuration["SSLVision"]["team"]["robot-id-mapping"]
        self.foe_robot_id_mapping: dict =\
            self.configuration["SSLVision"]["foe-team"]["robot-id-mapping"]

    def receive(self):
        return super().receive()

    def receive_dict(self):
        data = self.receive()

        packet = wrapper.SSL_WrapperPacket()
        packet.ParseFromString(data)

        detection = packet.detection

        return json.loads(MessageToJson(detection))

    def update(self) :
        vision_data_dict = self.receive_dict()
        return self._field_data_from_dict(vision_data_dict)

    def _entity_from_dict(
        self,
        data_dict:dict
    ):
        entity_data = Entity()

        entity_data.position.x = data_dict.get('x', 0) * 0.0012
        entity_data.position.y = data_dict.get('y', 0) * 0.00173

        entity_data.position.theta = data_dict.get('orientation', 0)

        entity_data.velocity.x = data_dict.get('vx', 0)
        entity_data.velocity.y = data_dict.get('vy', 0)

        entity_data.velocity.theta = data_dict.get('vorientation', 0)

        return entity_data
    
    def team_list_of_dicts(
        self,
        raw_data_dict: dict,
        key: str
    ):
        team_list_of_dicts = raw_data_dict.get(key)

        if team_list_of_dicts is None:
            return []
        
        return team_list_of_dicts
    
    def get_index(self, received_robot: dict):
        robot_id = str(received_robot.get("robotId")).lower()
        return self.foe_robot_id_mapping.get(robot_id)

    def _field_data_from_dict(self, raw_data_dict: dict):
        field = Field()

        team_list_of_dicts = self._get_team_list_of_dicts(raw_data_dict)
        foes_list_of_dicts = self._get_foe_list_of_dicts(raw_data_dict)

        field.ball = self._get_ball(raw_data_dict)

        field.robots = self._get_robots(team_list_of_dicts, 3)
        field.foes = self._get_robots(foes_list_of_dicts, 3)

        return field
    
    def _get_team_list_of_dicts(self, raw_data_dict: dict):
        if self.team_color_yellow:
            return self.team_list_of_dicts(raw_data_dict, 'robotsYellow')
        else:
            return self.team_list_of_dicts(raw_data_dict, 'robotsBlue')
    
    def _get_foe_list_of_dicts(self, raw_data_dict: dict):
        if self.team_color_yellow:
            return self.team_list_of_dicts(raw_data_dict, 'robotsBlue')
        else:
            return self.team_list_of_dicts(raw_data_dict, 'robotsYellow')
    
    def _get_ball(self, raw_data_dict: dict):
        ball_index = 0

        fake_ball = self._get_fake_ball()

        raw_data_dict_balls = raw_data_dict.get('balls')

        if raw_data_dict_balls is not None:
            raw_data_dict_ball = raw_data_dict_balls[ball_index]
        else:
            raw_data_dict_ball = fake_ball

        return self._entity_from_dict(raw_data_dict_ball)
    
    def _get_fake_ball(self):
        return {
            "confidence": 0.99282587,
            "area": 52,
            "x": 369.55273,
            "y": -838.1288,
            "pixelX": 389.59616,
            "pixelY": 453.98077
        }
    
    def _get_robots(
        self,
        robot_dicts: list,
        number_of_robots: int
    ):
        robots = [Entity() for i in range(number_of_robots)]
        
        for received_robot in robot_dicts:
            robot = self._entity_from_dict(received_robot)
            index = self.get_index(received_robot)

            if index is not None:
                robots[index] = robot

        return robots