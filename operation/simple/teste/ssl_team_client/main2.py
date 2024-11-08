import argparse
import random
import socket
import time
from ssl_team_client.client import load_private_key, detect_host, sign, get_connection_string
import logging

from communication.protobuf.game_controller.ssl_gc_rcon_team_pb2 import (
    ControllerToTeam,
    TeamRegistration,
    TeamToController,
    AdvantageChoice
)

from communication.protobuf.game_controller.ssl_gc_rcon_pb2 import ControllerReply

from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import serialization, hashes
from ssl_team_client.sslconn import send_message, receive_message
from ssl_team_client.state import Team

logging.basicConfig(level=logging.INFO)

parser = argparse.ArgumentParser()
parser.add_argument("--udpAddress", default="224.5.23.1:10003", help="The multicast address of ssl-game-controller")
parser.add_argument("--autoDetectHost", default=True, action="store_true", help="Automatically detect the game-controller host")
parser.add_argument("--address", default="localhost:10008", help="Address to connect to")
parser.add_argument("--privateKey", default="", help="Private key for signing messages")
parser.add_argument("--teamName", default="Test Team", help="The name of the team")
parser.add_argument("--teamColor", default="./Test Team.key.pem", help="Team color (YELLOW or BLUE)")
parser.add_argument("--verbose", action="store_true", help="Verbose logging")
args = parser.parse_args()

private_key = load_private_key(args.privateKey)

class Client:
    def __init__(self):
        self.conn = None
        self.reader = None
        self.token = ""
    
    def connect(self, address):
        try:
            self.conn = socket.create_connection((address.split(":")[0], int(address.split(":")[1])))
            self.reader = self.conn.makefile("rb")
            logging.info("Connected to game-controller at %s", address)
            return True
        except socket.error as e:
            logging.error("Could not connect to game-controller at %s: %s", address, e)
            return False
    
    def close(self):
        if self.conn:
            self.conn.close()

    def register(self):
        reply = ControllerToTeam()
        if not receive_message(self.reader, reply):
            logging.error("Failed receiving controller reply")
            return False

        if not reply.controller_reply or not reply.controller_reply.next_token:
            logging.error("Missing next token")
            return False

        registration = TeamRegistration()
        registration.team_name = args.teamName

        if args.teamColor in Team:
            registration.team = Team[args.teamColor]

        if private_key:
            registration.signature = sign(private_key, registration)

        logging.info("Sending registration: %s", registration)
        if not send_message(self.conn, registration):
            logging.error("Failed sending registration")
            return False

        reply = ControllerToTeam()
        if not receive_message(self.reader, reply):
            logging.error("Failed receiving controller reply")
            return False

        if not reply.controller_reply or reply.controller_reply.status_code != ControllerReply.OK:
            logging.error("Registration rejected: %s", reply.controller_reply.reason or "Unknown reason")
            return False

        logging.info("Successfully registered as %s", args.teamName)
        self.token = reply.controller_reply.next_token or ""
        return True

    def send_desired_keeper(self, keeper_id):
        request = TeamToController()
        request.msg = TeamToController.DesiredKeeper(desired_keeper=keeper_id)
        return self.send_request(request)

    def send_advantage_choice(self, choice):
        request = TeamToController()
        request.msg = TeamToController.AdvantageChoice(advantage_choice=choice)
        return self.send_request(request)

    def send_request(self, request):
        if private_key:
            request.signature = sign(private_key, request)
            request.signature.token = self.token

        if args.verbose:
            logging.info("Sending %s", request)

        if not send_message(self.conn, request):
            logging.error("Failed sending request: %s", request)
            return False, True

        reply = ControllerToTeam()
        if not receive_message(self.reader, reply):
            logging.error("Failed receiving controller reply")
            return False, True

        if reply.controller_reply.status_code != ControllerReply.OK:
            logging.error("Message rejected: %s", reply.controller_reply.reason or "Unknown reason")
            return False, False

        if reply.controller_reply.next_token:
            self.token = reply.controller_reply.next_token
        else:
            self.token = ""

        return True, False

def main():
    if args.autoDetectHost:
        logging.info("Trying to detect host based on incoming referee messages...")
        host = detect_host(args.udpAddress)
        if host:
            logging.info("Detected game-controller host: %s", host)
            args.address = get_connection_string(args.address, host)
        else:
            logging.warning("No host detected")

    while True:
        client = Client()
        if not client.connect(args.address):
            time.sleep(5)
            continue

        if not client.register():
            client.close()
            time.sleep(5)
            continue

        client.send_desired_keeper(3)

        advantage_choice = AdvantageChoice.CONTINUE
        while True:
            _, failed = client.send_advantage_choice(advantage_choice)
            if failed:
                break
            advantage_choice = AdvantageChoice.STOP if advantage_choice == AdvantageChoice.CONTINUE else AdvantageChoice.CONTINUE
            time.sleep(random.uniform(1, 5))

        client.close()
        time.sleep(5)

if __name__ == "__main__":
    main()