from communication.receiver.game_controller.game_controller_remote_receiver import GameControllerRemoteReceiver

class Teste:
    def __init__(self):
        self.game_controller_remote_receiver = GameControllerRemoteReceiver()

    def main(self):
        while True:
            message = self.game_controller_remote_receiver.receive()
            print(message)