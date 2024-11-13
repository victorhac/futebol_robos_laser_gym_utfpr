import logging
import os
import signal
import socket
import struct
from google.protobuf.message import DecodeError

from communication.protobuf.ssl_gc_referee_message_pb2 import Referee
from configuration.configuration import Configuration

class SSLRefereeClient:
    def __init__(self):
        self.configuration = Configuration.get_object()
        self.address = self.configuration.referee_address
        self.port = self.configuration.referee_port
        self.sock = self.setup_multicast_socket()
        self.buffer_size = self.configuration.referee_receiver_buffer_size
        self.history = []

    def close_socket(self):
        self.sock.close()

    def setup_multicast_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", self.port))
        mreq = struct.pack("4sl", socket.inet_aton(self.address), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        return sock

    def consume(self):
        data, _ = self.sock.recvfrom(self.buffer_size)
        ref_msg = Referee()
        try:
            ref_msg.ParseFromString(data)
        except DecodeError:
            return None, False

        if len(self.history) == 0 or ref_msg.command != self.history[-1]:
            self.history.append(ref_msg.command)

        return ref_msg, False

def main():
    client = SSLRefereeClient()
    def handle_signal(signum, frame):
        print("Stopping server...")
        client.close_socket()
        os._exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    while True:
        try:
            message, error = client.consume()
        except Exception as e:
            logging.error(f"Error receiving data: {e}")
            continue

if __name__ == "__main__":
    main()