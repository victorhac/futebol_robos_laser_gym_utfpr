from communication.protobuf.game_controller.ssl_gc_rcon_team_pb2 import (
    ControllerToTeam,
    TeamRegistration,
    TeamToController,
    AdvantageChoice
)

from communication.protobuf.game_controller.ssl_gc_rcon_pb2 import Signature, ControllerReply

import argparse
import logging
import random
import socket
import time
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import serialization, hashes
from ssl_team_client.sslconn import send_message, receive_message
from ssl_team_client.state import Team

parser = argparse.ArgumentParser()
parser.add_argument("--address", default="localhost:10008", help="Address to connect to")
parser.add_argument("--privateKey", default="/home/victor/repos/futebol_robos_laser_gym_utfpr/operation/simple/teste/ssl_team_client/Test Team.key.pem", help="Path to the private key used for signing messages")
parser.add_argument("--teamName", default="Test Team", help="The name of the team")
parser.add_argument("--teamColor", default="YELLOW", help="The color of the team (YELLOW or BLUE)")
parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")
args = parser.parse_args()

private_key = None
team_color = args.teamColor

class Client:
    def __init__(self):
        self.conn = None
        self.reader = None
        self.token = ""

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
        if not receive_message(self.reader, reply):
            logging.error("Failed receiving controller reply.")
            return False

        registration = TeamRegistration()
        registration.team_name=args.teamName

        if team_color == "YELLOW":
            registration.team = Team.YELLOW
        elif team_color == "BLUE":
            registration.team = Team.BLUE
        else:
            registration.team = Team.UNKNOWN

        if private_key:
            signature = Signature(token=reply.controller_reply.next_token)
            signature.pkcs1v15 = sign_data(private_key, registration)
            registration.signature.CopyFrom(signature)

        if args.verbose:
            logging.info(f"Sending registration: {registration}")

        if not send_message(self.conn, registration):
            logging.error("Failed sending registration.")
            return False

        reply = ControllerToTeam()
        if not receive_message(self.reader, reply):
            logging.error("Failed receiving controller reply.")
            return False

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
        if private_key:
            request.signature = Signature(token=self.token, pkcs1v15=sign_data(private_key, request))

        if args.verbose:
            logging.info(f"Sending {request}")

        if not send_message(self.conn, request):
            logging.error(f"Failed sending request: {request}")
            return False, True

        reply = ControllerToTeam()
        if not receive_message(self.reader, reply):
            logging.error("Failed receiving controller reply.")
            return False, True

        if args.verbose:
            logging.info(f"Received reply: {reply}")

        if reply.controller_reply.status_code != ControllerReply.OK:
            logging.error(f"Message rejected: {reply.controller_reply.reason}")
            return False, False

        self.token = reply.controller_reply.next_token or ""
        return True, False

def sign_data(private_key, data):
    data_bytes = data.SerializeToString()
    return private_key.sign(
        data_bytes,
        padding.PKCS1v15(),
        hashes.SHA256()
    )

def load_private_key(path):
    if not path:
        return None
    with open(path, "rb") as key_file:
        return serialization.load_pem_private_key(key_file.read(), password=None)

def main():
    global private_key
    private_key = load_private_key(args.privateKey)

    client = Client()

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