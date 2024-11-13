import socket
import os

from communication.receiver.receiver import Receiver
import pickle

from configuration.configuration import Configuration
from domain.field import Field

class RemoteComputerReceiver(Receiver):
    def __init__(self, field: Field):
        self.configuration = Configuration.get_object()
        self.connect()
        self.field = field

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
            pass

    def update(self):
        try:
            data = self.client.recv(2048)
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