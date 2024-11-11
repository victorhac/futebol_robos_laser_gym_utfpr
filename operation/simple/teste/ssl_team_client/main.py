from communication.protobuf.ssl_gc_rcon_team_pb2 import (
    ControllerToTeam,
    TeamRegistration,
    TeamToController,
    AdvantageChoice
)

from communication.protobuf.ssl_gc_rcon_pb2 import ControllerReply

from communication.protobuf.ssl_gc_common_pb2 import Team

import argparse
import logging
import random
import socket
import time
from communication.utils.game_controller.sslconn import send_message, receive_message
from communication.utils.game_controller.client import insert_signature, load_private_key

default_team_name = "UTBots"

parser = argparse.ArgumentParser()

parser.add_argument(
    "--address",
    default="localhost:10008",
    help="Address to connect to"
)
parser.add_argument(
    "--privateKey",
    default=f"./communication/keys/{default_team_name}.key.pem",
    help="Path to the private key used for signing messages"
)
parser.add_argument(
    "--teamName",
    default=default_team_name,
    help="The name of the team"
)
parser.add_argument(
    "--teamColor",
    default="YELLOW",
    help="The color of the team (YELLOW or BLUE)"
)
parser.add_argument(
    "--verbose",
    default=True,
    action="store_true",
    help="Enable verbose logging"
)

args = parser.parse_args()

team_color = args.teamColor

class Client:
    def __init__(self):
        self.conn = None
        self.reader = None
        self.token = ""
        self.private_key = None

    def connect(self, address: str):
        try:
            self.conn = socket.create_connection((address.split(":")[0], int(address.split(":")[1])))
            self.reader = self.conn.makefile("rb")
            logging.info(f"Connected to game-controller at {address}")
        except Exception as e:
            logging.error(f"Could not connect to game-controller at {address}: {e}")
            return False
        return True

    def register(self):
        reply = ControllerToTeam()
        receive_message(self.reader, reply)

        registration = TeamRegistration()
        registration.team_name=args.teamName

        if team_color == "YELLOW":
            registration.team = Team.YELLOW
        elif team_color == "BLUE":
            registration.team = Team.BLUE
        else:
            registration.team = Team.UNKNOWN

        if self.private_key:
            insert_signature(
                registration,
                reply.controller_reply.next_token,
                self.private_key
            )

        if args.verbose:
            logging.info(f"Sending registration: {registration}")

        send_message(self.conn, registration)

        reply = ControllerToTeam()

        receive_message(self.reader, reply)

        if reply.controller_reply.status_code != ControllerReply.StatusCode.OK:
            logging.error(f"Registration rejected: {reply.controller_reply.reason}")
            return False

        logging.info(f"Successfully registered as {args.teamName}")
        self.token = reply.controller_reply.next_token or ""
        return True

    def send_desired_keeper(self, id):
        request = TeamToController(desired_keeper=id)
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

        if args.verbose:
            logging.info(f"Sending {request}")

        send_message(self.conn, request)

        reply = ControllerToTeam()

        receive_message(self.reader, reply)

        if args.verbose:
            logging.info(f"Received reply: {reply}")

        if reply.controller_reply.status_code != ControllerReply.OK:
            logging.error(f"Message rejected: {reply.controller_reply.reason}")
            return False, False

        self.token = reply.controller_reply.next_token or ""
        return True, False

def main():
    client = Client()

    client.private_key = load_private_key(args.privateKey)

    while True:
        if client.connect(args.address):
            if client.register():
                client.send_desired_keeper(3)

                choice = AdvantageChoice.CONTINUE
                while True:
                    accepted, failed = client.send_advantage_choice(choice)
                    if failed:
                        break
                    choice = AdvantageChoice.STOP if choice == AdvantageChoice.CONTINUE else AdvantageChoice.CONTINUE
                    time.sleep(random.uniform(1, 5))

        time.sleep(5)

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)
    main()