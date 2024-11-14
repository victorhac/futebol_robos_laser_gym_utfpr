import socket
import pickle
from communication.protobuf.ssl_gc_referee_message_pb2 import Referee

from communication.utils.game_controller.sslconn import receive_message
from configuration.configuration import Configuration

class GameControllerRemoteReceiver:
    def __init__(self):
        self.configuration = Configuration.get_object()

        self.server: socket.socket = None
        self.client: socket.socket = None

        self.buffer_size = self.configuration.remote_computer_game_controller_receiver_buffer_size

        self.connect()

    def connect(self):
        self.server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

        self.server.bind((
            self.configuration.remote_computer_bluetooth_address,
            self.configuration.remote_computer_bluetooth_game_controller_channel
        ))
        
        self.server.listen(1)

        self.client, addr = self.server.accept()
        self.reader = self.client.makefile("rb")
    
    def close_connection(self):
        try:
            self.client.close()
            self.server.close()
        except:
            print("Não foi possível desconectar")

    def receive(self):
        try:
            message = Referee()
            receive_message(self.reader, message)
            return message
        except:
            self.close_connection()
            self.connect()