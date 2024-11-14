import socket
import time
import pickle

from communication.receiver.ssl_vision_receiver import SSLVisionReceiver
from configuration.configuration import Configuration
from domain.field import Field

class SSLVisionRemoteSender:
    def __init__(self):
        self.configuration = Configuration.get_object()

        self.field = Field()
        self.ssl_vision = SSLVisionReceiver(self.field)

        self.client: socket.socket = None
        self.server_address = self.configuration.remote_computer_bluetooth_address
        self.channel = self.configuration.remote_computer_bluetooth_channel

    def connect(self, max_retries=3):
        if self.client:
            self.close_socket()
        
        for attempt in range(max_retries):
            try:
                self.client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self.client.connect((self.server_address, self.channel))
                print(f"SSLVision: Connected to {self.server_address} on channel {self.channel}")
                return
            except Exception as e:
                print(f"SSLVision: Connection attempt {attempt + 1} failed: {e}")
                time.sleep(2)

        raise Exception("SSLVision: Failed to connect after multiple attempts.")
    
    def close_socket(self):
        if self.client:
            try:
                self.client.close()
                print("SSLVision: Socket connection closed successfully.")
            except Exception as e:
                print(f"SSLVision: Error while closing socket: {e}")
            finally:
                self.client = None

    def send_message(self):
        self.ssl_vision.update()
        message = pickle.dumps(self.field)
        self.client.send(message)

    def main(self):
        while True:
            try:
                self.connect()
            except Exception as e:
                print(f"SSLVision: Error while connecting: {e}")
                exit(1)

            while True:
                try:
                    self.send_message()
                except Exception as e:
                    print(f"SSLVision: Unexpected error: {e}")
                    self.close_socket()
                    break