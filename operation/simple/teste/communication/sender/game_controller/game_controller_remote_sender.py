from configuration.configuration import Configuration
import socket

from game_controller.clients.ssl_referee_client import SSLRefereeClient

class GameControllerRemoteSender:
    def __init__(self):
        self.configuration = Configuration.get_object()

        self.client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

        self.server_address = self.configuration.remote_computer_bluetooth_address
        self.channel = self.configuration.remote_computer_bluetooth_game_controller_channel

        self.ssl_referee_client = SSLRefereeClient()

        self.connect()

    def connect(self):
        try:
            self.client.connect((self.server_address, self.channel))
            print(f"Conectado ao servidor Bluetooth {self.server_address} no canal {self.channel}")
        except Exception as e:
            print(f"Erro ao conectar: {e}")
            self.client.close()
            exit(1)

    def main(self):
        while True:
            try:
                game_controller_message, error = self.ssl_referee_client.consume()

                if not error:
                    print(game_controller_message)
                    message = game_controller_message.SerializeToString()
                    self.client.sendall(message)
            except Exception as e:
                print(f"Erro ao enviar mensagem: {e}")

                print("Tentando reconectar...")
                self.connect()