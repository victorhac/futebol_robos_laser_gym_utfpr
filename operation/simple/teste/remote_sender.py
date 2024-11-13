from communication.sender.game_controller.game_controller_remote_sender import GameControllerRemoteSender
from communication.sender.ssl_vision.ssl_vision_remote_sender import SSLVisionRemoteSender
from configuration.configuration import Configuration
import threading

class RemoteSender:
    def __init__(self):
        self.configuration = Configuration.get_object()
        self.threads = []
        self.game_controller_remote_sender = GameControllerRemoteSender()
        self.ssl_vision_remote_sender = SSLVisionRemoteSender()

    def main(self):
        game_controller_remote_sender_thread = threading.Thread(target=self.game_controller_remote_sender.main)
        self.threads.append(game_controller_remote_sender_thread)
        game_controller_remote_sender_thread.start()

        ssl_vision_remote_sender_thread = threading.Thread(target=self.ssl_vision_remote_sender.main)
        self.threads.append(ssl_vision_remote_sender_thread)
        ssl_vision_remote_sender_thread.start()

        for item in self.threads:
            item.join()

        self.ssl_vision_remote_sender.main()

if __name__ == '__main__':
    RemoteSender().main()