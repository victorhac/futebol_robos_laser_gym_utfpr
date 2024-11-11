from communication.receiver.grsim_receiver import GrSimReceiver
from communication.receiver.ssl_vision_receiver import SSLVisionReceiver
from communication.sender.grsim_sender import GrSimSender
from communication.sender.ros_sender import RosSender
from configuration.configuration import Configuration
from domain.field import Field
from threads.thread_common_objects import ThreadCommonObjects
from communication.protobuf.ssl_gc_referee_message_pb2 import Referee
import time

class Executor:
    def __init__(self):
        self.configuration = Configuration.get_object()
        self.field = Field()

        if self.configuration.environment_mode == "REAL":
            self.receiver = SSLVisionReceiver(self.field)
            self.sender = RosSender()
        else:
            self.receiver = GrSimReceiver(self.field)
            self.sender = GrSimSender()
    
    def main(self):
        while True:
            message = ThreadCommonObjects.get_gc_to_executor_message()
            
            time.sleep(2)
            
            atacker_id = self.configuration.team_roles_attacker_id
            defender_id = self.configuration.team_roles_defensor_id
            goalkeeper_id = self.configuration.team_roles_goalkeeper_id

            atacker = self.field.robots[atacker_id]
            defender = self.field.robots[defender_id]
            goleiro = self.field.robots[goalkeeper_id]
            ball = self.field.ball
            
            if message.command == 0:
                self.sender.transmit_robot(0, 0, atacker_id)
                self.sender.transmit_robot(0, 0, defender_id)
                self.sender.transmit_robot(0, 0, goalkeeper_id)
