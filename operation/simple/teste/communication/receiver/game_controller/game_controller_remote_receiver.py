import socket
import pickle
from communication.protobuf.ssl_gc_referee_message_pb2 import Referee

from configuration.configuration import Configuration

class GameControllerRemoteReceiver:
    def __init__(self):
        self.configuration = Configuration.get_object()

        self.server: socket.socket = None
        self.client: socket.socket = None

        self.connect()

    def connect(self):
        self.server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.server.bind((self.configuration.referee_address, self.configuration.remote_computer_bluetooth_game_controller_channel))
        self.server.listen(1)

        self.client, addr = self.server.accept()

    def receive(self):
        try:
            message = Referee()

            data = self.client.recv(2048)

            message.ParseFromString(data)

            return message
        except:
            return None