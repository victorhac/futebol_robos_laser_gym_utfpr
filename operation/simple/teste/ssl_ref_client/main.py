import argparse
import logging
import os
import signal
import socket
import struct
from google.protobuf import json_format
from google.protobuf import text_format
from google.protobuf.message import DecodeError

from communication.protobuf.ssl_gc_referee_message_pb2 import Referee

history = []
referee_address = "224.5.23.1"
referee_port = 11003

parser = argparse.ArgumentParser(description="SSL Game Controller")
parser.add_argument(
    "--address",
    default=f"{referee_address}:{referee_port}",
    help="The multicast address of ssl-game-controller"
)
parser.add_argument(
    "--fullScreen",
    default=True,
    action="store_true",
    help="Print the formatted message to the console, clearing the screen during print"
)
parser.add_argument(
    "--verbose",
    action="store_true",
    help="Verbose output"
)
args = parser.parse_args()


def main():
    multicast_ip, port = args.address.split(":")
    port = int(port)

    sock = setup_multicast_socket(multicast_ip, port)
    
    def handle_signal(signum, frame):
        print("Stopping server...")
        sock.close()
        os._exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    while True:
        try:
            data, _ = sock.recvfrom(1024)
            consume(data)
        except Exception as e:
            logging.error(f"Error receiving data: {e}")
            continue


def setup_multicast_socket(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    mreq = struct.pack("4sl", socket.inet_aton(ip), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    return sock

def consume(data):
    global history

    ref_msg = Referee()
    try:
        ref_msg.ParseFromString(data)
    except DecodeError:
        logging.error("Could not unmarshal referee message")
        return

    if len(history) == 0 or ref_msg.command != history[-1]:
        history.append(ref_msg.command)

    if args.fullScreen:
        print("\033[H\033[2J", end="")
        
        print("Last commands: ", end="")
        n = min(len(history), 5)
        for i in range(n):
            print(history[-1 - i], end=", " if i < n - 1 else "\n")
            
        print(text_format.MessageToString(ref_msg, as_utf8=True))
    else:
        json_msg = json_format.MessageToJson(ref_msg)
        print(json_msg)


if __name__ == "__main__":
    main()