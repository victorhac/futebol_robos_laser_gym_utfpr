import socket
import os

from communication.receiver.receiver import Receiver
import pickle

from configuration.configuration import Configuration
from domain.field import Field

class SSLVisionRemoteReceiver(Receiver):
    def __init__(self, field: Field):
        self.configuration = Configuration.get_object()
        self.connect()
        self.field = field
        self.buffer_size = self.configuration.remote_computer_game_controller_receiver_buffer_size

    def connect(self):
        self.server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

        self.server.bind((
            self.configuration.remote_computer_bluetooth_address,
            self.configuration.remote_computer_bluetooth_channel))
        
        self.server.listen(1)

        self.client, addr = self.server.accept()

    def close_connection(self):
        try:
            self.client.close()
            self.server.close()
        except:
            print("Não foi possível desconectar")

    def update(self):
        try:
            data = self.client.recv(self.buffer_size)
            if data:      
                field: Field = pickle.loads(data)
                self.field.update(field)
        except KeyboardInterrupt:
            self.close_connection()
            os._exit(1)
        except:
            self.close_connection()
            self.connect()

    def receive(self):
        pass