from configuration.configuration import Configuration
import socket
import time

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
                print(f"Connected to {self.server_address} on channel {self.channel}")
                return
            except Exception as e:
                print(f"Connection attempt {attempt + 1} failed: {e}")
                time.sleep(2)

        raise Exception("Failed to connect after multiple attempts.")
    
    def close_socket(self):
        if self.client:
            try:
                self.client.close()
                print("Socket connection closed successfully.")
            except Exception as e:
                print(f"Error while closing socket: {e}")
            finally:
                self.client = None

    def send_message(self):
        game_controller_message, error = self.ssl_referee_client.consume()
        if not error:
            message = game_controller_message.SerializeToString()
            self.client.sendall(message)

    def main(self):
        while True:
            try:
                self.connect()
            except Exception as e:
                print(f"Error while connecting: {e}")
                exit(1)
                
            try:
                self.send_message()
            except Exception as e:
                print(f"Erro ao enviar mensagem: {e}")

                print("Tentando reconectar...")
                self.connect()