from communication.receiver.grsim_receiver import GrSimReceiver
from communication.sender.grsim_sender import GrSimSender
from configuration.configuration import Configuration
from domain.field import Field
from threads.thread_common_objects import ThreadCommonObjects
from communication.protobuf.ssl_gc_referee_message_pb2 import Referee
import time

class Executor:
    def __init__(self):
        self.configuration = Configuration.get_object()
        field = Field()

        self.receiver = GrSimReceiver(field)
        self.sender = GrSimSender()
    
    def main(self):
        while True:
            message = ThreadCommonObjects.get_gc_to_executor_message()
            print(message.command)
            time.sleep(2)