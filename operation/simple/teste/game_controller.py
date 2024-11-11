import time
import logging
import os
import socket
import struct
from google.protobuf.message import DecodeError

from communication.protobuf.ssl_gc_referee_message_pb2 import Referee
from configuration.configuration import Configuration
from threads.thread_common_objects import ThreadCommonObjects

history = []

class GameController:
    def __init__(self):
        self.configuration = Configuration.get_object()
        self.history = []
        self.address = "224.5.23.1"
        self.port = 11003
        self.sock: socket.socket = None

    def main(self):
        self.sock = self.setup_multicast_socket()
        
        def handle_signal(signum, frame):
            print("Stopping server...")
            self.sock.close()
            os._exit(0)

        # TODO: ver depois
        # signal.signal(signal.SIGINT, handle_signal)
        # signal.signal(signal.SIGTERM, handle_signal)

        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                self.consume(data)
            except Exception as e:
                logging.error(f"Error receiving data: {e}")
                continue

    def setup_multicast_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", self.port))
        mreq = struct.pack("4sl", socket.inet_aton(self.address), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        return sock

    def consume(self, data):
        ref_msg = Referee()
        try:
            ref_msg.ParseFromString(data)
            ThreadCommonObjects.set_gc_to_executor_message(ref_msg)

            time.sleep(1)
        except DecodeError:
            logging.error("Could not unmarshal referee message")
            return

        if len(history) == 0 or ref_msg.command != history[-1]:
            history.append(ref_msg.command)