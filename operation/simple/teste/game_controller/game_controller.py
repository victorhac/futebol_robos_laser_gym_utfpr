import time, random, os

from collections import deque
import threading

from communication.protobuf.ssl_gc_rcon_team_pb2 import (
    AdvantageChoice
)

from configuration.configuration import Configuration
from game_controller.clients.ssl_referee_client import SSLRefereeClient
from game_controller.clients.ssl_team_client import SSLTeamClient

from threads.thread_common_objects import ThreadCommonObjects

history = []

class GameController:
    def __init__(self):
        self.configuration = Configuration.get_object()
        self.ssl_referee_client = SSLRefereeClient()
        self.ssl_team_client = SSLTeamClient()
        self.threads = []
        self.send_advantage_choice_thread = None
        self.history = deque(maxlen=10)

    def close_socket(self):
        self.ssl_referee_client.close_socket()
        self.ssl_team_client.close_socket()

    def send_advantage_choice(self):
        choice = AdvantageChoice.CONTINUE
        self.ssl_team_client.send_advantage_choice(choice)
        time.sleep(random.uniform(1, 5))

    def main(self):
        while True:
            if self.ssl_team_client.register():
                self.ssl_team_client.send_desired_keeper()

                self.send_advantage_choice_thread = threading.Thread(target=self.send_advantage_choice)
                self.threads.append(self.send_advantage_choice_thread)
                self.send_advantage_choice_thread.start()

                while True:
                    message, error = self.ssl_referee_client.consume()

                    if len(self.history) == 0 or message.command != self.history[-1]:
                        self.history.append(message.command)
                        ThreadCommonObjects.set_gc_to_executor_message(message)

                    time.sleep(1)