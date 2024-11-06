import struct
import base64
from google.protobuf import message
from google.protobuf.internal.encoder import _VarintBytes
from google.protobuf import text_format
from io import BufferedReader
import socket

def read_data_length(reader: BufferedReader) -> int:
    length_bytes = reader.read(4)
    if len(length_bytes) < 4:
        raise ValueError("Failed to read 4 bytes for data length header")
    length = struct.unpack(">I", length_bytes)[0]  # '>I' is for big-endian uint32
    return length

def send_message(conn: socket.socket, message_proto: message.Message) -> None:
    data = message_proto.SerializeToString()
    size = len(data)
    conn.sendall(struct.pack(">I", size) + data)

def receive(reader: BufferedReader) -> bytes:
    data_length = read_data_length(reader)
    data = reader.read(data_length)
    if len(data) < data_length:
        raise ValueError("Failed to read the full data length")
    return data

def unmarshal(data: bytes, message_proto: message.Message) -> None:
    try:
        message_proto.ParseFromString(data)
    except message.DecodeError as e:
        encoded_message = base64.b64encode(data).decode('utf-8')
        raise ValueError(f"Could not unmarshal data: {encoded_message}") from e

def receive_message(reader: BufferedReader, message_proto: message.Message) -> None:
    data = receive(reader)
    return unmarshal(data, message_proto)
