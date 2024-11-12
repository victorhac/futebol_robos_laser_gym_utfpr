from configuration.configuration import Configuration
from game_controller.game_controller import GameController
# from communication.sender.ros_sender import RosSender
import threading
import signal
import os

from executor import Executor

threads = []

configuration = Configuration.get_object()

executor = Executor()
#game_controller = GameController()

def main():
    # field = Field()

    # receiver = GrSimReceiver(field)
    # sender = GrSimSender()

    # if configuration.mode == "MANUAL":
    #     receiver = GrSimReceiver(field)
    #     sender = GrSimSender()
    # else:
    #     receiver = SSLVisionReceiver(field)
    #     sender = RosSender()

    # game_controller_thread = threading.Thread(target=game_controller.main)
    # threads.append(game_controller_thread)
    # game_controller_thread.start()

    executor_thread = threading.Thread(target=executor.main)
    threads.append(executor_thread)
    executor_thread.start()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    for item in threads:
        item.join()

def handle_signal(signum, frame):
    #game_controller.close_socket()
    os._exit(0)

if __name__ == "__main__":
    main()