from configuration.configuration import Configuration
from domain.field import Field
from game_controller.game_controller import GameController
import threading
import signal
import os

from executor import Executor
from gui.control_center import ControlCenter
import tkinter as tk

configuration = Configuration.get_object()

threads = []

root = tk.Tk()

executor = Executor()
game_controller = GameController()
control_center = ControlCenter(root)

def main():
    control_center_thread = threading.Thread(target=control_center.main)
    threads.append(control_center_thread)
    control_center_thread.start()

    game_controller_thread = threading.Thread(target=game_controller.main)
    threads.append(game_controller_thread)
    game_controller_thread.start()

    executor_thread = threading.Thread(target=executor.main)
    threads.append(executor_thread)
    executor_thread.start()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    for item in threads:
        item.join()

def handle_signal(signum, frame):
    game_controller.close_socket()
    os._exit(0)

if __name__ == "__main__":
    main()