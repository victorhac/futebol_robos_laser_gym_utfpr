import hashlib
import logging
import socket
import os
from google.protobuf import message
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.primitives import serialization

logging.basicConfig(level=logging.INFO)

def detect_host(address: str) -> str:
    multicast_ip, port = address.split(':')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', int(port)))
    group = socket.inet_aton(multicast_ip) + socket.inet_aton("0.0.0.0")
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, group)
    
    try:
        data, addr = sock.recvfrom(1024)
        return addr[0]
    finally:
        sock.close()

def get_connection_string(address: str, host: str) -> str:
    _, port = address.split(':')
    return f"{host}:{port}"

def load_private_key(private_key_location: str) -> rsa.RSAPrivateKey:
    if private_key_location:
        private_key = read_private_key(private_key_location)
        if private_key:
            logging.info("Found private key")
            return private_key
        else:
            logging.warning("No private key available")
    return None

def read_private_key(private_key_location: str) -> rsa.RSAPrivateKey:
    try:
        with open(private_key_location, "rb") as key_file:
            private_key = serialization.load_pem_private_key(
                key_file.read(),
                password=None,
            )
            if not isinstance(private_key, rsa.RSAPrivateKey):
                raise ValueError("Private key type is wrong")
            return private_key
    except Exception as e:
        logging.error(f"Could not read private key at {private_key_location}: {e}")
        raise

def sign(private_key: rsa.RSAPrivateKey, message_proto: message.Message) -> bytes:
    message_bytes = message_proto.SerializeToString()
    hash_value = hashlib.sha256(message_bytes).digest()
    
    signature = private_key.sign(
        hash_value,
        padding.PKCS1v15(),
        hashes.SHA256()
    )
    return signature
