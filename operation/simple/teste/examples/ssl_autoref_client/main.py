import socket
import argparse
import logging
import time

from communication.utils.game_controller.client import (
    insert_signature,
    load_private_key,
    detect_host,
    get_connection_string
)

from communication.utils.game_controller.sslconn import send_message, receive_message

from communication.protobuf.ssl_gc_rcon_pb2 import ControllerReply
from communication.protobuf.ssl_gc_rcon_autoref_pb2 import (
    AutoRefToController,
    ControllerToAutoRef,
    AutoRefRegistration
)

from communication.protobuf.ssl_gc_game_event_pb2 import (
    GameEvent
)

from communication.protobuf.ssl_gc_common_pb2 import (
    Team
)

from communication.protobuf.ssl_gc_geometry_pb2 import Vector2

default_team_name = "UTBots"

parser = argparse.ArgumentParser(description="SSL Game Controller Client")

parser.add_argument(
    "--udpAddress",
    default="224.5.23.1:11003",
    help="The multicast address of ssl-game-controller"
)
parser.add_argument(
    "--autoDetectHost",
    type=bool,
    default=True,
    help="Automatically detect the game-controller host"
)
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
    "--identifier",
    default=default_team_name,
    help="Client identifier"
)

args = parser.parse_args()

class Client:
    def __init__(self, conn, private_key):
        self.conn = conn
        self.reader = conn.makefile('rb')
        self.token = ""
        self.private_key = private_key

    def register(self):
        reply = ControllerToAutoRef()
        
        receive_message(self.reader, reply)

        if not reply.controller_reply or not reply.controller_reply.next_token:
            logging.error("Missing next token")
            return False

        registration = AutoRefRegistration(identifier=args.identifier)

        if self.private_key:
            insert_signature(
                registration,
                reply.controller_reply.next_token,
                self.private_key
            )
        
        logging.info("Sending registration")
        
        send_message(self.conn, registration)

        logging.info("Sent registration, waiting for reply")
        
        receive_message(self.reader, reply)

        if not reply.controller_reply or reply.controller_reply.status_code != ControllerReply.StatusCode.OK:
            reason = reply.controller_reply.reason or "Unknown reason"
            logging.error(f"Registration rejected: {reason}")
            return False

        logging.info(f"Successfully registered as {args.identifier}")
        self.token = reply.controller_reply.next_token or ""
        return True

    def send_request(self, request, do_log):
        if self.private_key:
            insert_signature(
                request,
                self.token,
                self.private_key
            )

        if do_log:
            logging.info(f"Sending {request}")

        send_message(self.conn, request)

        if do_log:
            logging.info("Waiting for reply...")

        reply = ControllerToAutoRef()
        
        receive_message(self.reader, reply)

        if do_log:
            logging.info(f"Received reply: {reply}")
        if not reply.controller_reply or reply.controller_reply.status_code != ControllerReply.StatusCode.OK:
            logging.error(f"Message rejected: {reply.controller_reply.reason}")
        self.token = reply.controller_reply.next_token or ""

    def keep_alive(self):
        while True:
            time.sleep(1)
            self.send_request(AutoRefToController(), False)

    def send_ball_left_field(self):
        event = GameEvent.BallLeftField()
        event.by_bot = 2
        event.by_team = Team.BLUE
        event.location.CopyFrom(Vector2(x=1, y=4.5))
        game_event = GameEvent(event=event, type=GameEvent.BALL_LEFT_FIELD_TOUCH_LINE)
        request = AutoRefToController(game_event=game_event)
        self.send_request(request, True)

    def send_double_touch(self):
        event = GameEvent.AttackerDoubleTouchedBall()
        event.by_bot = 2
        event.by_team = Team.BLUE
        event.location.CopyFrom(Vector2(x=1, y=4.5))
        game_event = GameEvent(attacker_double_touched_ball=event, type=GameEvent.ATTACKER_DOUBLE_TOUCHED_BALL)
        request = AutoRefToController(game_event=game_event)
        self.send_request(request, True)

    def send_bot_crash_unique(self):
        event = GameEvent.BotCrashUnique()
        event.violator = 2
        event.victim = 5
        event.by_team = Team.BLUE
        event.location.CopyFrom(Vector2(x=1, y=4.5))
        game_event = GameEvent(bot_crash_unique=event, type=GameEvent.BOT_CRASH_UNIQUE)
        request = AutoRefToController(game_event=game_event)
        self.send_request(request, True)

def main():
    global args
    private_key = load_private_key(args.privateKey)
    if args.autoDetectHost:
        logging.info("Trying to detect host based on incoming referee messages...")
        host = detect_host(args.udpAddress)
        if host:
            logging.info(f"Detected game-controller host: {host}")
            args.address = get_connection_string(args.address, host)

    try:
        conn = socket.create_connection((args.address.split(':')[0], int(args.address.split(':')[1])))
    except Exception as e:
        logging.error(f"Could not connect to game-controller at {args.address}: {e}")
        return
    
    logging.info(f"Connected to game-controller at {args.address}")
    client = Client(conn, private_key)

    if not client.register():
        return

    from threading import Thread
    Thread(target=client.keep_alive, daemon=True).start()

    commands = {
        "ballLeftField": client.send_ball_left_field,
        "botCrashUnique": client.send_bot_crash_unique,
        "doubleTouch": client.send_double_touch
    }

    while True:
        cmd_input = input("-> ").strip()
        cmd_parts = cmd_input.split()
        cmd_name, args = cmd_parts[0], cmd_parts[1:]
        if cmd_name in commands:
            commands[cmd_name]()
        else:
            print("Available commands:")
            for cmd in commands.keys():
                print(f"  {cmd:<20}")

if __name__ == "__main__":
    main()