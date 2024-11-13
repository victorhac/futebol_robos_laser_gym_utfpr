import socket
import struct
from abc import ABC, abstractmethod

class Receiver(ABC):
    def __init__(self, receiver_ip, receiver_port, buffer_size):
        self.receiver_ip = receiver_ip
        self.receiver_port = receiver_port
        self.receiver_socket = self._create_socket()
        self.buffer_size = buffer_size

    @abstractmethod
    def receive(self):
        return self.receiver_socket.recv(self.buffer_size)
    
    @abstractmethod
    def update(self):
        pass

    def _create_socket(self):
        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
            socket.IPPROTO_UDP
        )

        sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR, 1
        )

        sock.bind((self.receiver_ip, self.receiver_port))

        mreq = struct.pack(
            "4sl",
            socket.inet_aton(self.receiver_ip),
            socket.INADDR_ANY
        )

        sock.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            mreq
        )

        return sock