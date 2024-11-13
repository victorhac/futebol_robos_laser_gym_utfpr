from communication.sender.game_controller.game_controller_remote_sender import GameControllerRemoteSender
from communication.sender.ssl_vision.ssl_vision_remote_sender import SSLVisionRemoteSender


class Teste:
    def __init__(self):
        self.game_controller_remote_sender = GameControllerRemoteSender()

    def main(self):
        self.game_controller_remote_sender.main()

def main():
    teste = Teste()
    teste.main()

if __name__ == '__main__':
    main()