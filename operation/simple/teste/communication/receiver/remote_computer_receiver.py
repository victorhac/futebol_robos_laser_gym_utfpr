import socket

from communication.receiver.receiver import Receiver
import pickle

from configuration.configuration import Configuration
from domain.field import Field

class RemoteComputerReceiver(Receiver):
    def __init__(self, field: Field):
        self.configuration = Configuration.get_object()
        self.server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.server.bind((self.configuration.referee_address, self.configuration.referee_channel))
        self.server.listen(1)

        self.client, addr = self.server.accept()

        self.field = field

    def update(self):
        data = self.client.recv(2048)        
        field: Field = pickle.loads(data)
        self.field.update(field)