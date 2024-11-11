from configuration.configuration import Configuration
from game_controller import GameController
# from communication.sender.ros_sender import RosSender
import threading

from executor import Executor

configuration = Configuration.get_object()

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

    threads = []

    executor = Executor()
    game_controller = GameController()

    game_controller_thread = threading.Thread(target=game_controller.main)
    threads.append(game_controller_thread)
    game_controller_thread.start()

    executor_thread = threading.Thread(target=executor.main)
    threads.append(executor_thread)
    executor_thread.start()

    for item in threads:
        item.join()

if __name__ == "__main__":
    main()