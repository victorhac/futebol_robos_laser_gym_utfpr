import time, random, os

from collections import deque
import threading

from communication.protobuf.ssl_gc_rcon_team_pb2 import (
    AdvantageChoice
)

from communication.receiver.game_controller.game_controller_remote_receiver import GameControllerRemoteReceiver
from configuration.configuration import Configuration
from game_controller.clients.ssl_referee_client import SSLRefereeClient
from game_controller.clients.ssl_team_client import SSLTeamClient

from threads.thread_common_objects import ThreadCommonObjects

history = []

class GameController:
    def __init__(self):
        self.configuration = Configuration.get_object()

        if self.configuration.receive_data_from_remote:
            self.ssl_referee_client = None
            self.game_controller_remote_receiver = GameControllerRemoteReceiver()
        else:
            self.ssl_referee_client = SSLRefereeClient()
            self.game_controller_remote_receiver = None

        if self.configuration.game_controller_register_as_team:
            self.ssl_team_client = SSLTeamClient()

        self.threads = []
        self.send_advantage_choice_thread = None
        self.history = deque(maxlen=10)

    def close_socket(self):
        self.ssl_referee_client.close_socket()

        if self.configuration.game_controller_register_as_team:
            self.ssl_team_client.close_socket()

    def send_advantage_choice(self):
        choice = AdvantageChoice.CONTINUE
        self.ssl_team_client.send_advantage_choice(choice)
        time.sleep(random.uniform(1, 5))

    def register_as_team(self):
        self.ssl_team_client.send_desired_keeper()

        self.send_advantage_choice_thread = threading.Thread(target=self.send_advantage_choice)
        self.threads.append(self.send_advantage_choice_thread)
        self.send_advantage_choice_thread.start()

    def consume(self):
        if self.configuration.receive_data_from_remote:
            message = self.game_controller_remote_receiver.receive()
        else:
            message, error = self.ssl_referee_client.consume()

        return message
    
    def try_register_as_team(self):
        if self.configuration.game_controller_register_as_team:
            if self.ssl_team_client.register():
                self.register_as_team()
                return True
            else:
                return False
            
        return True

    def main(self):
        while True:
            if not self.try_register_as_team():
                continue

            while True:
                message = self.consume()

                if message and (len(self.history) == 0 or message.command != self.history[-1]):
                    self.history.append(message.command)
                    ThreadCommonObjects.set_gc_to_executor_message(message)