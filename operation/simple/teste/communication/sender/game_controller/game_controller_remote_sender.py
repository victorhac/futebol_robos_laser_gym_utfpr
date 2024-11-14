import pickle
from configuration.configuration import Configuration
import socket
import time

from domain.referee_message_domain import RefereeMessageDomain
from game_controller.clients.ssl_referee_client import SSLRefereeClient

class GameControllerRemoteSender:
    def __init__(self):
        self.configuration = Configuration.get_object()

        self.ssl_referee_client = SSLRefereeClient()

        self.client: socket.socket = None
        self.server_address = self.configuration.remote_computer_bluetooth_address
        self.channel = self.configuration.remote_computer_bluetooth_game_controller_channel

    def connect(self, max_retries=3):
        if self.client:
            self.close_socket()
        
        for attempt in range(max_retries):
            try:
                self.client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self.client.connect((self.server_address, self.channel))
                print(f"GameController: Connected to {self.server_address} on channel {self.channel}")
                return
            except Exception as e:
                print(f"GameController: Connection attempt {attempt + 1} failed: {e}")
                time.sleep(2)

        raise Exception("GameController: Failed to connect after multiple attempts.")
    
    def close_socket(self):
        if self.client:
            try:
                self.client.close()
                print("GameController: Socket connection closed successfully.")
            except Exception as e:
                print(f"GameController: Error while closing socket: {e}")
            finally:
                self.client = None

    def send_message(self):
        game_controller_message, error = self.ssl_referee_client.consume()
        if not error:
            message = pickle.dumps(RefereeMessageDomain(game_controller_message.command))
            self.client.send(message)

    def main(self):
        try:
            self.connect()
        except Exception as e:
            print(f"GameController: Error while connecting: {e}")
            exit(1)

        while True:
            try:
                self.send_message()
            except Exception as e:
                print(f"GameController: Erro ao enviar mensagem: {e}")

                print("GameController: Tentando reconectar...")
                self.connect()