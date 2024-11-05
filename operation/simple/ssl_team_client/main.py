import argparse
import logging
import random
import socket
import time
from client import load_private_key, detect_host, get_connection_string, sign
from sslconn import receive_message, send_message

from ..communication.protobuf.game_controller.ssl_gc_rcon_team_pb2\
    import ControllerToTeam, TeamRegistration, Signature, TeamToController, AdvantageChoice

# Command-line arguments
parser = argparse.ArgumentParser(description="SSL Game Controller Client")
parser.add_argument("--udpAddress", default="224.5.23.1:10003", help="The multicast address of ssl-game-controller")
parser.add_argument("--autoDetectHost", type=bool, default=True, help="Automatically detect the game-controller host")
parser.add_argument("--address", default="localhost:10008", help="Address to connect to")
parser.add_argument("--privateKey", default="", help="A private key to be used to sign messages")
parser.add_argument("--teamName", default="Test Team", help="The name of the team as it is sent by the referee")
parser.add_argument("--teamColor", default="", help="The color of the team as it is sent by the referee (YELLOW or BLUE)")
parser.add_argument("--verbose", type=bool, default=False, help="Verbose logging")
args = parser.parse_args()

# Logger configuration
logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

# Load private key if provided
private_key = load_private_key(args.privateKey) if args.privateKey else None

class Client:
    def __init__(self, conn, reader, token=""):
        self.conn = conn
        self.reader = reader
        self.token = token

    def register(self):
        reply = ControllerToTeam()
        if not receive_message(self.reader, reply):
            logging.error("Failed receiving controller reply")
            return False

        if reply.get_controller_reply().next_token is None:
            logging.error("Missing next token")
            return False

        registration = TeamRegistration(team_name=args.teamName)
        if args.teamColor in Team_value:
            registration.team = Team_value[args.teamColor]

        if private_key:
            registration.signature = Signature(token=reply.get_controller_reply().next_token, pkcs1_v15=sign(private_key, registration))

        logging.info(f"Sending registration: {registration}")
        if not send_message(self.conn, registration):
            logging.error("Failed sending registration")
            return False

        reply = ControllerToTeam()
        if not receive_message(self.reader, reply):
            logging.error("Failed receiving controller reply")
            return False

        if reply.get_controller_reply().status_code != "OK":
            reason = reply.get_controller_reply().reason or "Unknown reason"
            logging.error(f"Registration rejected: {reason}")
            return False

        logging.info(f"Successfully registered as {args.teamName}")
        self.token = reply.get_controller_reply().next_token or ""
        return True

    def send_desired_keeper(self, id):
        message = TeamToController_DesiredKeeper(desired_keeper=id)
        request = TeamToController(msg=message)
        return self.send_request(request)

    def send_advantage_choice(self, choice):
        reply = TeamToController_AdvantageChoice(advantage_choice=choice)
        request = TeamToController(msg=reply)
        return self.send_request(request)

    def send_request(self, request):
        if private_key:
            request.signature = Signature(token=self.token, pkcs1_v15=sign(private_key, request))

        if args.verbose:
            logging.info(f"Sending {request}")

        if not send_message(self.conn, request):
            logging.error(f"Failed sending request: {request}")
            return False, True

        reply = ControllerToTeam()
        if not receive_message(self.reader, reply):
            logging.error("Failed receiving controller reply")
            return False, True

        if reply.get_controller_reply().status_code != "OK":
            logging.error(f"Message rejected: {reply.get_controller_reply().reason}")
            return False, False

        self.token = reply.get_controller_reply().next_token or ""
        return True, False

def main():
    # Detect host if needed
    if args.autoDetectHost:
        logging.info("Trying to detect host based on incoming referee messages...")
        host = detect_host(args.udpAddress)
        if host:
            logging.info(f"Detected game-controller host: {host}")
            args.address = get_connection_string(args.address, host)
        else:
            logging.warning("No host detected")

    # Main loop
    while True:
        run()
        time.sleep(5)

def run():
    try:
        conn = socket.create_connection((args.address.split(":")[0], int(args.address.split(":")[1])))
        reader = conn.makefile("rb")
        logging.info(f"Connected to game-controller at {args.address}")

        client = Client(conn, reader)

        if not client.register():
            return

        client.send_desired_keeper(3)

        advantage_choice = AdvantageChoice.CONTINUE
        while True:
            accepted, failed = client.send_advantage_choice(advantage_choice)
            if failed:
                return

            advantage_choice = AdvantageChoice.STOP if advantage_choice == AdvantageChoice.CONTINUE else AdvantageChoice.CONTINUE
            time.sleep(random.uniform(5, 6))
    except socket.error:
        logging.error(f"Could not connect to game-controller at {args.address}")
    finally:
        try:
            conn.close()
        except Exception as e:
            logging.error(f"Could not close connection: {e}")

if __name__ == "__main__":
    main()