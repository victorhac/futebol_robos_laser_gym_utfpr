from communication.protobuf.ssl_gc_rcon_team_pb2 import (
    ControllerToTeam,
    TeamRegistration,
    TeamToController,
    AdvantageChoice
)

from communication.protobuf.ssl_gc_rcon_pb2 import ControllerReply

from communication.protobuf.ssl_gc_common_pb2 import Team

import logging
import random
import socket
import time
from communication.utils.game_controller.sslconn import send_message, receive_message
from communication.utils.game_controller.client import insert_signature, load_private_key
from configuration.configuration import Configuration

class SSLTeamClient:
    def __init__(self):
        self.configuration = Configuration.get_object()

        self.team_name = self.configuration.team_name

        self.address = self.configuration.game_controller_address
        self.port = self.configuration.game_controller_port

        self.conn = self.connect()
        self.reader = self.conn.makefile("rb")
        self.token = ""
        self.private_key = load_private_key(self.configuration.keys_private_key_path)

    def close_socket(self):
        self.conn.close()

    def connect(self):
        return socket.create_connection((self.address, self.port))

    def register(self):
        reply = ControllerToTeam()
        receive_message(self.reader, reply)

        registration = TeamRegistration()
        registration.team_name = self.team_name

        if self.configuration.team_is_yellow_team:
            registration.team = Team.YELLOW
        else:
            registration.team = Team.BLUE

        if self.private_key:
            insert_signature(
                registration,
                reply.controller_reply.next_token,
                self.private_key
            )

        send_message(self.conn, registration)

        reply = ControllerToTeam()

        receive_message(self.reader, reply)

        if reply.controller_reply.status_code != ControllerReply.StatusCode.OK:
            logging.error(f"Registration rejected: {reply.controller_reply.reason}")
            return False

        logging.info(f"Successfully registered as {self.team_name}")
        self.token = reply.controller_reply.next_token or ""
        return True

    def send_desired_keeper(self):
        request = TeamToController(
            desired_keeper=self\
                .configuration\
                .get_robot_id_to_external_mapped(self.configuration.team_roles_goalkeeper_id)
        )
        return self.send_request(request)

    def send_advantage_choice(self, choice):
        request = TeamToController(advantage_choice=choice)
        return self.send_request(request)

    def send_request(self, request):
        if self.private_key:            
            insert_signature(
                request,
                self.token,
                self.private_key
            )

        send_message(self.conn, request)

        reply = ControllerToTeam()

        receive_message(self.reader, reply)

        if reply.controller_reply.status_code != ControllerReply.OK:
            logging.error(f"Message rejected: {reply.controller_reply.reason}")
            return False, False

        self.token = reply.controller_reply.next_token or ""
        return True, False

def main():
    client = SSLTeamClient()

    while True:
        if client.register():
            client.send_desired_keeper()

            choice = AdvantageChoice.CONTINUE
            while True:
                accepted, failed = client.send_advantage_choice(choice)
                if failed:
                    break
                choice = AdvantageChoice.STOP if choice == AdvantageChoice.CONTINUE else AdvantageChoice.CONTINUE
                time.sleep(random.uniform(1, 5))

        time.sleep(5)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()