from communication.receiver.game_controller.game_controller_remote_receiver import GameControllerRemoteReceiver


if __name__ == '__main__':
    game_controller_remote_receiver = GameControllerRemoteReceiver()

    while True:
        print(game_controller_remote_receiver.receive())