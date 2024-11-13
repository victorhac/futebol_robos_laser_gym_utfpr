from configuration.configuration import Configuration
import socket

from game_controller.game_controller import GameController

class GameControllerRemoteSender:
    def __init__(self):
        self.configuration = Configuration.get_object()

        self.client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

        self.server_address = self.configuration.remote_computer_bluetooth_address
        self.channel = self.configuration.remote_computer_bluetooth_game_controller_channel

        self.game_controller = GameController()

        self.connect()

    # def try_close_socket_connection(self):
    #     try:
    #         if self.client:
    #             self.client.close()
    #     except Exception as e:
    #         print(f"Error while closing socket connection: {e}")

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
                game_controller_message = self.game_controller.consume()

                if game_controller_message is not None:
                    message = game_controller_message.SerializeToString()
                    self.client.sendall(message)
            except Exception as e:
                print(f"Erro ao enviar mensagem: {e}")

                print("Tentando reconectar...")
                self.connect()